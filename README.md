# ESP32 TinyML Pump Protection

Repositori ini berisi *source code* dan *dataset* kelistrikan untuk penelitian Tugas Akhir: **"Implementasi TinyML pada ESP32 untuk Deteksi Anomali Arus Listrik dan Proteksi Beban Induktif (Studi Kasus: Pompa Air Rumah Tangga)"**.

## Arsitektur Sistem
Sistem proteksi cerdas ini dibangun menggunakan arsitektur *Dual-Core* dengan sistem operasi waktu nyata (*FreeRTOS*).
* **Core 0:** Menangani fungsi IoT (WiFiManager, Captive Portal, Blynk, dan Telegram Bot).
* **Core 1:** Menangani fungsi deterministik (Sensor PZEM-004T, Inferensi TinyML secara luring, dan aktuasi Relai Ganda *Fail-safe*).

## Struktur Repositori
* `AI_Pump_Protect_Firmware.ino`: Kode sumber utama mikrokontroler ESP32-S3.
* `dataset pompa air v2`: Sampel data Tegangan (voltage), Arus (current), Daya (power), *Power Factor* (PF) dan Impedansi (Z) untuk kondisi beban Normal, Pipa Mampet (overload), dan *Dry Running*. dengan beberapa rentang tegangan

## Spesifikasi Perangkat Keras
* **Mikrokontroler:** ESP32-S3
* **Sensor Utama:** PZEM-004T V3.0 (Non-Invasive)
* **Aktuator:** Dual-Channel Relay Module (Active-LOW)

## Penulis
**Muhammad Raslan**
Teknologi Rekayasa Komputer Jaringan, Politeknik Negeri Tanah Laut (2026).
