/**
 * ================================================================================
 * @file       : AI_Pump_Protect_Firmware.ino
 * @brief      : Sistem Proteksi Pompa Air Berbasis TinyML & Dual-Core RTOS
 * @version    : 4.0 (Stable)
 * @author     : Muhammad Raslan - POLITALA
 * * @architecture:
 * - Core 0 : NetworkTask (WiFiManager, Blynk, Telegram Bot)
 * - Core 1 : Main Loop (PZEM Sensor, AI Inference, Relay Control, UI/LCD)
 * * @features   : 
 * - Edge Impulse TinyML (Normal, Dry Run, Pipe Blockage)
 * - Mutex-protected Shared Memory (Thread-Safe)
 * - 3-Tier Hardware Auto-Recovery (PZEM, I2C LCD, Hard Reset PB)
 * - Asynchronous Telegram Queue & Critical Alert Bypass
 * ================================================================================
 */

// ==========================================
// 1. CLOUD & NETWORK CREDENTIALS
// ==========================================

#define BLYNK_TEMPLATE_ID "ur template id"
#define BLYNK_TEMPLATE_NAME "ur template name"
#define BLYNK_AUTH_TOKEN "ur auth token"

#include <waterpump_v2_inferencing.h> 
#include <PZEM004Tv30.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <WiFiClientSecure.h>
#include <BlynkSimpleEsp32.h>
#include <UniversalTelegramBot.h>
#include <ctype.h>
#include <freertos/queue.h> 

#define BOT_TOKEN "ur bot token"
#define CHAT_ID "ur chat id"

WiFiClientSecure client;
UniversalTelegramBot bot(BOT_TOKEN, client);

// ==========================================
// 2. PIN CONFIGURATION 
// ==========================================
#define PZEM_RX_PIN 18 
#define PZEM_TX_PIN 19
#define LCD_SDA_PIN 8
#define LCD_SCL_PIN 9

#define RELAY_PIN 4      
#define BUZZER_PIN 5     
#define BUTTON_PIN 10    

#define LED_R 6          
#define LED_Y 7          
#define LED_G 15         

PZEM004Tv30 pzem(Serial1, PZEM_RX_PIN, PZEM_TX_PIN);
LiquidCrystal_I2C lcd(0x27, 20, 4); 

// ==========================================
// 3. SYSTEM VARIABLES & RTOS MUTEX
// ==========================================
TaskHandle_t NetworkTask; 
SemaphoreHandle_t dataMutex; 
QueueHandle_t telegramQueue; 

float global_v = 0.0, global_i = 0.0, global_p = 0.0, global_pf = 0.0;
bool is_ai_mode = false; 
bool system_tripped = false; 
bool simulate_short_circuit = false; 
bool global_relay_state = true; 

char global_status_msg[32] = "NORMAL"; 
char global_wifi_ssid[32] = ""; 

bool sync_blynk_v5 = false; int blynk_v5_val = 0;
bool sync_blynk_v6 = false; int blynk_v6_val = 0;
bool sync_blynk_v7_reset = false;
bool sync_blynk_v8_reset = false;
bool req_system_reset = false; 
int req_ui_transition = 0; 

volatile bool critical_alert_pending = false;
char critical_alert_msg[512] = ""; 

bool last_button_state = HIGH;
unsigned long last_debounce_time = 0;
unsigned long button_pressed_time = 0;
bool is_button_held = false;
bool long_press_triggered = false;
bool hard_reset_triggered = false; 

#define NUM_SAMPLES 6        
#define NUM_FEATURES 2      
float feature_buffer[NUM_SAMPLES * NUM_FEATURES];
int sample_count = 0;
unsigned long last_read_time = 0;

const float SHORT_CIRCUIT_LIMIT = 30.0; 
const float MIN_ACTIVE_CURRENT = 0.03;  
bool is_dry_run_warning = false;
unsigned long dry_run_start_time = 0;
unsigned long last_beep_time = 0;
bool buzzer_state = false;
const unsigned long DRY_RUN_TIMEOUT = 120000; 

bool pump_is_running = false;
unsigned long pump_start_time = 0;
const unsigned long INRUSH_BYPASS_TIME = 5000; 

// ==========================================
// 4. CENTRALIZED UTILITY FUNCTIONS
// ==========================================
void setTrafficLight(bool red, bool yellow, bool green) {
  digitalWrite(LED_R, red ? HIGH : LOW);
  digitalWrite(LED_Y, yellow ? HIGH : LOW);
  digitalWrite(LED_G, green ? HIGH : LOW);
}

void setRelayState(bool state) { 
  digitalWrite(RELAY_PIN, state ? LOW : HIGH);
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) { 
    global_relay_state = state;
    sync_blynk_v6 = true; 
    blynk_v6_val = state ? 1 : 0;
    xSemaphoreGive(dataMutex);
  }
}

void setAIMode(bool enable) {
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) {
    is_ai_mode = enable;
    if (enable) {
      sample_count = 0;
      pump_is_running = false;
      strncpy(global_status_msg, "AI STANDBY", 31);
    } else {
      is_dry_run_warning = false;
      digitalWrite(BUZZER_PIN, HIGH);
      sample_count = 0;
      strncpy(global_status_msg, "MODE MONITORING", 31);
    }
    global_status_msg[31] = '\0';
    sync_blynk_v5 = true; 
    blynk_v5_val = enable ? 1 : 0;
    xSemaphoreGive(dataMutex);
  }
  if (!enable) { setRelayState(true); }
}

void triggerTelegram(const char* message) {
  char msg_copy[256];
  strncpy(msg_copy, message, 255);
  msg_copy[255] = '\0';
  if (telegramQueue != NULL) {
    if (xQueueSend(telegramQueue, &msg_copy, (TickType_t)0) != pdPASS) {
      Serial.println("ERR: Telegram Queue Full!");
    }
  }
}

void playRunningLED(int loops) {
  for (int j = 0; j < loops; j++) {
    setTrafficLight(false, false, true); vTaskDelay(pdMS_TO_TICKS(120)); 
    setTrafficLight(false, true, false); vTaskDelay(pdMS_TO_TICKS(120)); 
    setTrafficLight(true, false, false); vTaskDelay(pdMS_TO_TICKS(120)); 
    setTrafficLight(false, true, false); vTaskDelay(pdMS_TO_TICKS(120)); 
  }
  setTrafficLight(false, false, true); vTaskDelay(pdMS_TO_TICKS(200));   
  setTrafficLight(false, false, false);              
}

void printLCDPad(int col, int row, const char* text) {
  char padded[21];
  snprintf(padded, sizeof(padded), "%-20s", text); 
  lcd.setCursor(col, row); lcd.print(padded);        
}

void showModeTransition(const char* msg) {
  lcd.clear();
  printLCDPad(0, 1, msg);
  vTaskDelay(pdMS_TO_TICKS(1500)); 
  lcd.clear();
}

// ==========================================
// 5. BLYNK CONTROL (CORE 0)
// ==========================================
BLYNK_CONNECTED() {
  bool temp_ai = false, temp_relay = false;  
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) {
    temp_ai = is_ai_mode; temp_relay = global_relay_state;
    xSemaphoreGive(dataMutex);
  }
  Blynk.virtualWrite(V5, temp_ai ? 1 : 0);
  Blynk.virtualWrite(V6, temp_relay ? 1 : 0);
}

BLYNK_WRITE(V5) {
  bool enable = param.asInt() != 0;
  setAIMode(enable);
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) {
    req_ui_transition = enable ? 1 : 2; xSemaphoreGive(dataMutex);
  }
}

BLYNK_WRITE(V6) {
  int relay_req = param.asInt();
  bool trip_state = false, ai_state = false;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) { 
    trip_state = system_tripped; ai_state = is_ai_mode; xSemaphoreGive(dataMutex); 
  }
  if (trip_state || ai_state) { 
    bool current_relay = false;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) { current_relay = global_relay_state; xSemaphoreGive(dataMutex); }
    Blynk.virtualWrite(V6, current_relay ? 1 : 0); return; 
  }
  setRelayState(relay_req == 1);
}

BLYNK_WRITE(V7) { 
  if (param.asInt() == 1) {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) { 
      if (system_tripped) req_system_reset = true; 
      sync_blynk_v7_reset = true; xSemaphoreGive(dataMutex); 
    }
  }
}

BLYNK_WRITE(V8) { 
  if (param.asInt() == 1) {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) { 
      simulate_short_circuit = true; sync_blynk_v8_reset = true; xSemaphoreGive(dataMutex); 
    }
  }
}

// ==========================================
// 6. MAIN SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  
  pinMode(RELAY_PIN, OUTPUT); pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_R, OUTPUT); pinMode(LED_Y, OUTPUT); pinMode(LED_G, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP); 

  digitalWrite(RELAY_PIN, HIGH); digitalWrite(BUZZER_PIN, HIGH);   
   
  Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN); 
  lcd.init(); lcd.backlight();
  
  printLCDPad(0, 0, " AI PUMP PROTECT    "); 
  
  printLCDPad(0, 1, "PZEM Sensor  : Init");
  Serial1.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
  vTaskDelay(pdMS_TO_TICKS(1200)); 
  if (isnan(pzem.voltage())) {
    printLCDPad(0, 1, "PZEM Sensor  :[FAIL]");
  } else {
    printLCDPad(0, 1, "PZEM Sensor  :  [OK]");
  }

  printLCDPad(0, 2, "System RTOS  : Init");
  dataMutex = xSemaphoreCreateMutex();
  telegramQueue = xQueueCreate(10, 256);
  if (dataMutex == NULL || telegramQueue == NULL) { 
    printLCDPad(0, 2, "System RTOS  :[FAIL]");
    while(1); 
  } else {
    printLCDPad(0, 2, "System RTOS  :  [OK]");
  }

  printLCDPad(0, 3, "Network Service:Init");
  client.setInsecure(); 
  client.setTimeout(5000); 
  xTaskCreatePinnedToCore(networkTaskCode, "NetworkTask", 16384, NULL, 1, &NetworkTask, 0);
  vTaskDelay(pdMS_TO_TICKS(200)); 
  if (NetworkTask == NULL) {
    printLCDPad(0, 3, "Network Serv :[FAIL]");
  } else {
    printLCDPad(0, 3, "Network Serv :  [OK]");
  }
  
  vTaskDelay(pdMS_TO_TICKS(1500)); lcd.clear();
  printLCDPad(0, 1, "INITIALIZATION DONE ");
  printLCDPad(0, 2, "    SYSTEM READY    ");
  vTaskDelay(pdMS_TO_TICKS(1500));

  setRelayState(true); 
  lcd.clear();
}

// ==========================================
// 7. CORE 0 TASK (WIFI & BLYNK)
// ==========================================
void configModeCallback(WiFiManager *myWiFiManager) {
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) {
    req_ui_transition = 3; 
    strncpy(global_wifi_ssid, "OFFLINE", 31); 
    xSemaphoreGive(dataMutex);
  }
}

void saveConfigCallback() {
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) {
    req_ui_transition = 5; xSemaphoreGive(dataMutex);
  }
}

void networkTaskCode(void * pvParameters) {
  WiFi.mode(WIFI_STA); 
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  
  WiFiManager wm;
  wm.setAPCallback(configModeCallback); 
  wm.setSaveConfigCallback(saveConfigCallback);
  wm.setConfigPortalBlocking(false);

  unsigned long last_blynk_sync = 0;
  unsigned long last_tele_check = 0; 
  bool start_msg_sent = false, blynk_configured = false;
  char last_sent_status[32] = ""; 

  bool is_connected = wm.autoConnect("SETUP POMPA AI");
  bool was_disconnected = !is_connected; 

  // 🌟 Pop-up ganda dibuang dari sini. Callback (configModeCallback) sudah menangani UI.

  for(;;) { 
    wm.process();

    if (WiFi.status() != WL_CONNECTED) {
      if (!was_disconnected) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) {
          req_ui_transition = 6; 
          strncpy(global_wifi_ssid, "OFFLINE", 31);
          xSemaphoreGive(dataMutex);
        }
        was_disconnected = true;
        start_msg_sent = false; 
      }
    }
    else {
      if (was_disconnected) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) {
          req_ui_transition = 4; 
          strncpy(global_wifi_ssid, WiFi.SSID().c_str(), 31);
          xSemaphoreGive(dataMutex);
        }
        was_disconnected = false;
      }

      if (!blynk_configured) { Blynk.config(BLYNK_AUTH_TOKEN); blynk_configured = true; }
      
      if (!start_msg_sent) { 
        if (bot.sendMessage(CHAT_ID, "🟢 System Online | Protection services are active.", "")) {
          start_msg_sent = true; 
        }
      }
      
      Blynk.run();
      
      bool do_v5 = false, do_v6 = false, do_v7 = false, do_v8 = false;
      int val_v5 = 0, val_v6 = 0;

      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) {
        if (sync_blynk_v5) { do_v5 = true; val_v5 = blynk_v5_val; sync_blynk_v5 = false; }
        if (sync_blynk_v6) { do_v6 = true; val_v6 = blynk_v6_val; sync_blynk_v6 = false; }
        if (sync_blynk_v7_reset) { do_v7 = true; sync_blynk_v7_reset = false; }
        if (sync_blynk_v8_reset) { do_v8 = true; sync_blynk_v8_reset = false; }
        xSemaphoreGive(dataMutex);
      }

      if (do_v5) Blynk.virtualWrite(V5, val_v5);
      if (do_v6) Blynk.virtualWrite(V6, val_v6);
      if (do_v7) Blynk.virtualWrite(V7, 0); 
      if (do_v8) Blynk.virtualWrite(V8, 0); 

      if (millis() - last_blynk_sync > 3000) {
        float temp_v = 0.0, temp_i = 0.0, temp_p = 0.0, temp_pf = 0.0; 
        char temp_status[32] = "NORMAL";
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) {
          temp_v = global_v; temp_i = global_i; temp_p = global_p; temp_pf = global_pf;
          strncpy(temp_status, global_status_msg, 31); temp_status[31] = '\0';
          xSemaphoreGive(dataMutex);
        }
        Blynk.virtualWrite(V0, temp_v); Blynk.virtualWrite(V1, temp_i);
        Blynk.virtualWrite(V2, temp_p); Blynk.virtualWrite(V3, temp_pf);
        if (strcmp(temp_status, last_sent_status) != 0) {
          Blynk.virtualWrite(V4, temp_status); 
          strncpy(last_sent_status, temp_status, 31); last_sent_status[31] = '\0';
        }
        last_blynk_sync = millis();
      }

      if (critical_alert_pending) {
        for (int retry = 0; retry < 3; retry++) {
          if (bot.sendMessage(CHAT_ID, critical_alert_msg, "")) break;
          vTaskDelay(pdMS_TO_TICKS(2000));
        }
        critical_alert_pending = false;
      }

      char rcv_msg[256];
      if (telegramQueue != NULL) {
        if (xQueueReceive(telegramQueue, &rcv_msg, 0) == pdPASS) {
          bot.sendMessage(CHAT_ID, rcv_msg, "");
        }
      }

      if (millis() - last_tele_check > 5000) {
        int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
        for (int i = 0; i < numNewMessages; i++) {
          String chat_id = bot.messages[i].chat_id;
          String text = bot.messages[i].text;
          text.toLowerCase();
          
          if (text == "/ping") {
            bot.sendMessage(chat_id, "🏓 Pong! System Online.", "");
          }
        }
        last_tele_check = millis();
      }

    }
    vTaskDelay(pdMS_TO_TICKS(20)); 
  }
}

// ==========================================
// 8. CORE 1 TASK (HARDWARE & AI LOOP)
// ==========================================
void loop() {
  static unsigned long last_lcd_check = 0;
  if (millis() - last_lcd_check >= 5000) {
    last_lcd_check = millis();
    Wire.beginTransmission(0x27);
    if (Wire.endTransmission() != 0) {
      Serial.println("WARN: LCD I2C Error! Auto-recovery berjalan...");
      Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
      lcd.init();
      lcd.backlight();
    }
  }

  bool current_trip_state = false; 
  bool do_reset_from_blynk = false;
  int ui_transition_pending = 0; 
  char current_ssid[32] = "";

  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) { 
    current_trip_state = system_tripped; 
    if (req_system_reset) { do_reset_from_blynk = true; req_system_reset = false; }
    if (req_ui_transition != 0) {
      ui_transition_pending = req_ui_transition;
      strncpy(current_ssid, global_wifi_ssid, 31);
      req_ui_transition = 0;
    }
    xSemaphoreGive(dataMutex); 
  }

  if (ui_transition_pending == 1) { showModeTransition(" > AI MODE ACTIVE < "); last_read_time = 0; } 
  else if (ui_transition_pending == 2) { showModeTransition("> MONITORING MODE <"); last_read_time = 0; } 
  else if (ui_transition_pending == 3 || ui_transition_pending == 6) {
    lcd.clear();
    printLCDPad(0, 0, "COMMUNICATION LOST  ");
    printLCDPad(0, 1, "Mode: Standalone    "); 
    printLCDPad(0, 2, "Access AP to Setup  "); 
    printLCDPad(0, 3, "> SETUP POMPA AI <  "); 
    vTaskDelay(pdMS_TO_TICKS(3000));
    lcd.clear(); last_read_time = 0;
  } else if (ui_transition_pending == 5) {
    lcd.clear();
    printLCDPad(0, 1, "SAVING CONFIG       ");
    printLCDPad(0, 2, "PLEASE WAIT...      ");
    vTaskDelay(pdMS_TO_TICKS(2000));
    lcd.clear(); last_read_time = 0;
  } else if (ui_transition_pending == 4) {
    lcd.clear();
    printLCDPad(0, 0, "WIFI CONNECTED!     ");
    char ssid_msg[21]; snprintf(ssid_msg, sizeof(ssid_msg), "SSID: %-14s", current_ssid); 
    printLCDPad(0, 1, ssid_msg);
    printLCDPad(0, 2, "Cloud Services      ");
    printLCDPad(0, 3, "Connected           ");
    vTaskDelay(pdMS_TO_TICKS(3000)); 
    lcd.clear(); last_read_time = 0;
  }

  if (do_reset_from_blynk && current_trip_state) { resetSystem(); current_trip_state = false; }

  if (current_trip_state) {
    bool reading = digitalRead(BUTTON_PIN);
    if (reading == LOW) {
      if (!is_button_held) { 
        button_pressed_time = millis(); is_button_held = true; 
        long_press_triggered = false; hard_reset_triggered = false;
      } else {
        unsigned long hold_time = millis() - button_pressed_time;
        if (hold_time >= 5000 && !hard_reset_triggered) {
          hard_reset_triggered = true;
          lcd.clear(); printLCDPad(0, 1, "  HARD RESET ESP  ");
          vTaskDelay(pdMS_TO_TICKS(1000)); ESP.restart();
        } 
        else if (hold_time >= 3000 && hold_time < 5000 && !long_press_triggered) {
          long_press_triggered = true; 
          digitalWrite(BUZZER_PIN, LOW); vTaskDelay(pdMS_TO_TICKS(100)); digitalWrite(BUZZER_PIN, HIGH);
        }
      }
    } else { 
      if (is_button_held && long_press_triggered && !hard_reset_triggered) {
        resetSystem(); 
      }
      is_button_held = false; 
    }
    last_button_state = reading; return; 
  }

  bool reading = digitalRead(BUTTON_PIN);
  if (reading != last_button_state) last_debounce_time = millis();

  if ((millis() - last_debounce_time) > 50) {
    if (reading == LOW && !is_button_held) {
      button_pressed_time = millis(); is_button_held = true; 
      long_press_triggered = false; hard_reset_triggered = false;
      digitalWrite(BUZZER_PIN, LOW); vTaskDelay(pdMS_TO_TICKS(50)); digitalWrite(BUZZER_PIN, HIGH);
    }
    
    if (reading == LOW && is_button_held) {
      unsigned long hold_time = millis() - button_pressed_time;
      if (hold_time >= 5000 && !hard_reset_triggered) {
        hard_reset_triggered = true;
        lcd.clear(); printLCDPad(0, 1, "  HARD RESET ESP  ");
        vTaskDelay(pdMS_TO_TICKS(1000)); ESP.restart();
      }
      else if (hold_time >= 3000 && hold_time < 5000 && !long_press_triggered) {
        long_press_triggered = true; 
        digitalWrite(BUZZER_PIN, LOW); vTaskDelay(pdMS_TO_TICKS(100)); digitalWrite(BUZZER_PIN, HIGH);
      }
    }
    
    if (reading == HIGH && is_button_held) {
      is_button_held = false; 
      unsigned long hold_time = millis() - button_pressed_time;
      
      if (!hard_reset_triggered) {
        if (hold_time >= 3000) {
          if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) { simulate_short_circuit = true; xSemaphoreGive(dataMutex); }
          showModeTransition("INJECT 50A CURRENT! "); last_read_time = 0; 
        }
        else {
          bool temp_ai = false; 
          if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) { temp_ai = !is_ai_mode; xSemaphoreGive(dataMutex); }
          setAIMode(temp_ai);
          if (temp_ai) showModeTransition(" > AI MODE ACTIVE < "); else showModeTransition("> MONITORING MODE <");
          last_read_time = 0; 
        }
      }
    }
  }
  last_button_state = reading;

  bool active_ai_mode = false; 
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) { active_ai_mode = is_ai_mode; xSemaphoreGive(dataMutex); }

  if (is_dry_run_warning && active_ai_mode) {
    if (millis() - dry_run_start_time >= DRY_RUN_TIMEOUT) {
      is_dry_run_warning = false; executeTrip("DRY RUN TIMEOUT"); return; 
    } else if (millis() - last_beep_time >= 500) {
      last_beep_time = millis(); buzzer_state = !buzzer_state;
      digitalWrite(BUZZER_PIN, buzzer_state ? LOW : HIGH); setTrafficLight(buzzer_state, !buzzer_state, false);
    }
  }

  if (millis() - last_read_time >= 500) {
    last_read_time = millis();
    float temp_v = pzem.voltage(), temp_i = pzem.current(), temp_p = pzem.power(), temp_pf = pzem.pf(); 

    static int pzem_fail_count = 0;
    if (isnan(temp_v) || isnan(temp_i) || isnan(temp_p) || isnan(temp_pf)) { 
      pzem_fail_count++;
      if (pzem_fail_count >= 5) {
        Serial1.end(); vTaskDelay(pdMS_TO_TICKS(200));
        Serial1.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN); pzem_fail_count = 0;
      }
      printLCDPad(0, 0, "ERR: PZEM DISCONNECT"); setTrafficLight(true, true, true); return; 
    }
    pzem_fail_count = 0;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) {
      if (simulate_short_circuit) { temp_i = 50.0; simulate_short_circuit = false; }
      global_v = temp_v; global_i = temp_i; global_p = temp_p; global_pf = temp_pf; 
      xSemaphoreGive(dataMutex);
    }

    if (temp_i >= SHORT_CIRCUIT_LIMIT) { executeTrip("SHORT CIRCUIT (OVERCURRENT)"); return; }

    char lcd_buf[21];
    snprintf(lcd_buf, sizeof(lcd_buf), "V:%.0f I:%.2f PF:%.2f", temp_v, temp_i, temp_pf);
    printLCDPad(0, 0, lcd_buf);
    snprintf(lcd_buf, sizeof(lcd_buf), "P:%.1f W", temp_p);
    printLCDPad(0, 1, lcd_buf);

    if (active_ai_mode) {
      if (temp_i < MIN_ACTIVE_CURRENT) { 
        printLCDPad(0, 2, "STATUS: PUMP IDLE   "); 
        printLCDPad(0, 3, "AI: Standby         ");
        if (is_dry_run_warning) { is_dry_run_warning = false; digitalWrite(BUZZER_PIN, HIGH); }
        setTrafficLight(false, true, false); 
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) { 
          pump_is_running = false; sample_count = 0; 
          strncpy(global_status_msg, "PUMP OFF", 31); global_status_msg[31] = '\0'; xSemaphoreGive(dataMutex); 
        }
        return; 
      } else {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) { 
          if (!pump_is_running) { pump_is_running = true; pump_start_time = millis(); }
          xSemaphoreGive(dataMutex);
        }
      }

      float z = (temp_i > 0) ? (temp_v / temp_i) : 0.0;

      const float PF_MIN = 0.1f, PF_MAX = 1.0f; const float Z_MIN = 0.5f, Z_MAX = 5000.0f;
      if (z < Z_MIN || z > Z_MAX || temp_pf < PF_MIN || temp_pf > PF_MAX) {
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) { sample_count = 0; xSemaphoreGive(dataMutex); }
        printLCDPad(0, 2, "AI: BUFFER RESET!   "); printLCDPad(0, 3, "Dirty Data Detected ");
        return;
      }

      for (int idx = 0; idx < (NUM_SAMPLES - 1) * NUM_FEATURES; idx++) { feature_buffer[idx] = feature_buffer[idx + NUM_FEATURES]; }
      int last_idx = (NUM_SAMPLES - 1) * NUM_FEATURES;
      feature_buffer[last_idx + 0] = temp_pf; feature_buffer[last_idx + 1] = z;  

      int current_samples = 0; unsigned long current_pump_time = 0;
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) { 
        if (sample_count < NUM_SAMPLES) sample_count++; 
        current_samples = sample_count; current_pump_time = pump_start_time; xSemaphoreGive(dataMutex); 
      }

      // 🌟 SOLUSI BUG BUFFER 5/6 NYANGKUT: LCD mencetak status saat Wait
      if (current_samples < NUM_SAMPLES) {
        if (millis() - current_pump_time < INRUSH_BYPASS_TIME) { 
          printLCDPad(0, 2, "AI: INRUSH BYPASS!  "); 
          if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) { strncpy(global_status_msg, "INRUSH BYPASS", 31); global_status_msg[31] = '\0'; xSemaphoreGive(dataMutex); }
        } else { printLCDPad(0, 2, "AI: SAMPLING...     "); }
        snprintf(lcd_buf, sizeof(lcd_buf), "Buffer: %d/6         ", current_samples); printLCDPad(0, 3, lcd_buf);
      } else { 
        if (millis() - current_pump_time >= INRUSH_BYPASS_TIME) { run_inference(); } 
        else { 
          printLCDPad(0, 2, "AI: INRUSH BYPASS!  "); 
          printLCDPad(0, 3, "Buffer: 6/6 (Wait)  "); 
        }
      }

    } else {
      printLCDPad(0, 2, "MODE: MONITORING    "); 
      printLCDPad(0, 3, "AI  : DISABLED      ");
      if (temp_i < MIN_ACTIVE_CURRENT) setTrafficLight(false, true, false); else setTrafficLight(false, false, true); 
    }
  }
}

// ==========================================
// 9. TINYML INFERENCE FUNCTION
// ==========================================
void run_inference() {
  signal_t features_signal;
  if (numpy::signal_from_buffer(feature_buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &features_signal) != 0) return;

  ei_impulse_result_t result = { 0 };
  if (run_classifier(&features_signal, &result, false) != EI_IMPULSE_OK) return;

  float max_confidence = 0.0; const char* best_label_ptr = "";
  for (uint16_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    if (result.classification[ix].value > max_confidence) { 
      max_confidence = result.classification[ix].value; best_label_ptr = result.classification[ix].label; 
    }
  }

  char best_label[16];
  strncpy(best_label, best_label_ptr, 15); best_label[15] = '\0';
  for (int i = 0; best_label[i]; i++) best_label[i] = toupper(best_label[i]);
  if (strcmp(best_label, "DRY_RUN") == 0) strcpy(best_label, "DRY RUN"); 

  static int mampet_confirm = 0, dry_run_confirm = 0;

  if (strcmp(best_label, "NORMAL") == 0) {
    mampet_confirm = 0; dry_run_confirm = 0;
    if (is_dry_run_warning) { is_dry_run_warning = false; digitalWrite(BUZZER_PIN, HIGH); }
    setTrafficLight(false, false, true); 
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) { strncpy(global_status_msg, "NORMAL", 31); global_status_msg[31] = '\0'; xSemaphoreGive(dataMutex); }
  } 
  else if (strcmp(best_label, "MAMPET") == 0 && max_confidence >= 0.85) {
     mampet_confirm++; dry_run_confirm = 0;
     if (mampet_confirm >= 3) { executeTrip("PIPE BLOCKAGE"); return; }
  } 
  else if (strcmp(best_label, "DRY RUN") == 0 && max_confidence >= 0.85) {
    dry_run_confirm++; mampet_confirm = 0;
    if (dry_run_confirm >= 3) {
      if (!is_dry_run_warning) { 
        is_dry_run_warning = true; dry_run_start_time = millis(); last_beep_time = millis(); 
        triggerTelegram("⚠️ WARNING: DRY RUN\nPump is running without water. Circuit will cut in 120s!");
      }
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) { strncpy(global_status_msg, "WARNING: DRY RUN", 31); global_status_msg[31] = '\0'; xSemaphoreGive(dataMutex); }
    }
  } else { mampet_confirm = 0; dry_run_confirm = 0; }

  char lcd_buf[21];
  if (is_dry_run_warning) {
    snprintf(lcd_buf, sizeof(lcd_buf), "WARN: DRY %ds       ", (int)(120 - ((millis() - dry_run_start_time) / 1000))); printLCDPad(0, 2, lcd_buf);
  } else { 
    snprintf(lcd_buf, sizeof(lcd_buf), "AI: %-16s", best_label); printLCDPad(0, 2, lcd_buf); 
  }
  snprintf(lcd_buf, sizeof(lcd_buf), "Conf: %.1f%%        ", max_confidence * 100.0); printLCDPad(0, 3, lcd_buf);
}

// ==========================================
// 10. SYSTEM TRIP FUNCTION
// ==========================================
void executeTrip(const char* reason) {
  float t_v = 0.0, t_i = 0.0, t_p = 0.0, t_pf = 0.0;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) { 
    system_tripped = true; t_v = global_v; t_i = global_i; t_p = global_p; t_pf = global_pf; xSemaphoreGive(dataMutex); 
  }
  
  digitalWrite(BUZZER_PIN, LOW); setTrafficLight(true, false, false); setRelayState(false); 
  
  char rsn[32]; snprintf(rsn, sizeof(rsn), "TRIP: %s", reason);
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) { 
    strncpy(global_status_msg, rsn, 31); global_status_msg[31] = '\0'; xSemaphoreGive(dataMutex); 
  }
  
  snprintf(critical_alert_msg, sizeof(critical_alert_msg), 
           "🚨 PROTECTION EVENT: TRIP 🚨\n\nFault Class : %s\n\n📊 Pre-Trip Measurement:\nVoltage     : %.0f V\nCurrent     : %.2f A\nActive Power: %.1f W\nPower Factor: %.2f\n\n⚠️ Action Required:\nIsolate the load and inspect the electrical circuit before initiating manual reset.", 
           reason, t_v, t_i, t_p, t_pf);
  critical_alert_pending = true; 
  
  vTaskDelay(pdMS_TO_TICKS(1000)); lcd.init(); lcd.backlight(); lcd.clear(); 
  printLCDPad(0, 0, "!! SYSTEM HALTED !! ");
  char lcd_rsn[21]; snprintf(lcd_rsn, sizeof(lcd_rsn), "RSN: %-15s", reason);
  printLCDPad(0, 1, lcd_rsn); printLCDPad(0, 3, "Hold 3s to RESET... ");
}

// ==========================================
// 11. SYSTEM RESET FUNCTION
// ==========================================
void resetSystem() {
  digitalWrite(BUZZER_PIN, HIGH); showModeTransition(" REBOOTING SYSTEM..");
  triggerTelegram("Reset Success! System restored.\nMonitoring resumed | Protection is reactivated.");

  unsigned long wait_start = millis();
  while(digitalRead(BUTTON_PIN) == LOW && (millis() - wait_start < 5000)) { 
    setTrafficLight(false, true, false); vTaskDelay(pdMS_TO_TICKS(50)); 
  }
  playRunningLED(3);

  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200))) {
    system_tripped = false; is_dry_run_warning = false; sample_count = 0; 
    pump_is_running = false; simulate_short_circuit = false; 

    if (is_ai_mode) {
      strncpy(global_status_msg, "AI STANDBY", 31);
    } else {
      strncpy(global_status_msg, "MODE MONITORING", 31);
    }
    global_status_msg[31] = '\0';

    xSemaphoreGive(dataMutex);
  }
  
  is_button_held = false; last_button_state = HIGH; last_debounce_time = millis();
  setRelayState(true); vTaskDelay(pdMS_TO_TICKS(150)); lcd.init(); lcd.backlight(); lcd.clear(); last_read_time = 0; 
}