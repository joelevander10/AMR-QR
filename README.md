# 🤖 AMR QR Navigation System

> **Autonomous Mobile Robot** berbasis navigasi QR Code dengan sensor fusion IMU + Encoder, safety lidar SICK, dan web dashboard Flask.

---

## 📋 Daftar Isi

- [Arsitektur Sistem](#-arsitektur-sistem)
- [Deskripsi File](#-deskripsi-file)
  - [amr_controller.py](#1-amr_controllerpy--otak-utama-amr)
  - [amr_qr_nav.py](#2-amr_qr_navpy--sistem-visi-qr)
  - [trackless_module.py](#3-trackless_modulepy--sensor-fusion-imu--encoder)
  - [app.py](#4-apppy--web-dashboard-flask)
  - [encoder_idset.py](#5-encoder_idsetpy--utilitas-konfigurasi-encoder)
  - [index.html](#6-indexhtml--tampilan-hmi-dashboard)
  - [Encoder_.eds](#7-encoder_eds--definisi-objek-canopen)
- [Alur Navigasi Otomatis](#-alur-navigasi-otomatis)
- [Topologi Jaringan & Hardware](#-topologi-jaringan--hardware)
- [Dependensi](#-dependensi)
- [Cara Menjalankan](#-cara-menjalankan)

---

## 🏗 Arsitektur Sistem

```
┌─────────────────────────────────────────────────────────┐
│                      app.py (Flask HMI)                 │
│          Web Dashboard  ←→  AMRController               │
└────────────────────────┬────────────────────────────────┘
                         │
          ┌──────────────▼──────────────┐
          │       amr_controller.py      │
          │  ┌──────────┐ ┌───────────┐ │
          │  │ IntegratedQRSystem      │ │
          │  │ (amr_qr_nav.py bridge)  │ │
          │  └────┬─────┘ └─────┬─────┘ │
          │       │ Vision       │ MQTT  │
          │  ┌────▼─────┐ ┌─────▼─────┐ │
          │  │ Kamera   │ │ Lidar ROS │ │
          │  │ USB/CV2  │ │ SICK Nano │ │
          │  └──────────┘ └───────────┘ │
          │  ┌──────────────────────────┐│
          │  │   trackless_module.py    ││
          │  │  IMU Serial + 2x Encoder ││
          │  │  (CAN via ttyACM0)       ││
          │  └──────────────────────────┘│
          └──────────────────────────────┘
```

---

## 📂 Deskripsi File

---

### 1. `amr_controller.py` — Otak Utama AMR

> File terbesar dan terpenting. Mengintegrasikan semua subsistem: visi, motor, lidar, encoder, IMU, dan komunikasi MQTT.

#### 🔧 Kelas-kelas di dalamnya

| Kelas | Peran |
|---|---|
| `IntegratedQRSystem` | Bridge antara visi QR dan kontrol motor |
| `LidarSafetyMonitor` | Safety layer berbasis ROS + SICK lidar |
| `AMRController` | Kontroler utama: state machine, motor, navigasi |

---

#### 🔹 `IntegratedQRSystem`

Kelas ini mewarisi `QRNavigationSystem` (dari `amr_qr_nav.py`) dan mengimplementasikan `control_logic()` — fungsi yang dipanggil setiap frame kamera.

**Prioritas Alignment (berurutan):**

1. **Y-axis** — AMR mundur/maju hingga QR berada di tengah vertikal frame (`deadzone_y = 25px`)
2. **Sudut besar > 30°** → `ALIGN_LARGE_ROTATION`: rotasi penuh dengan target dinamis
3. **Sudut kecil < 30°** → `ALIGN_ROTATION`: pulse halus (gerak-berhenti)
4. **Semua aligned** → tunggu stabil 1 detik → eksekusi perintah QR

**Anti-Osilasi:** Jika AMR bolak-balik alignment > 10 kali, langsung eksekusi tanpa menunggu.

---

#### 🔹 `LidarSafetyMonitor`

Subscribe ke topik ROS `/sick_safetyscanners/scan` dan memantau zona depan AMR.

| Parameter | Nilai |
|---|---|
| Zona pantau | -30° s/d +30° (depan AMR) |
| Jarak STOP | < 0.6 m → motor berhenti penuh |
| Jarak SLOW | < 1.0 m → kecepatan 50% |
| Filter noise | Hanya range valid 0.15 – 10.0 m |

---

#### 🔹 `AMRController`

State machine utama dengan `nav_mode` sebagai penentu perilaku:

```
IDLE
 ├─► QR_SEARCH_ROTATION   (timeout QR > 3 detik → rotasi cari QR)
 ├─► ALIGN_Y              (koreksi posisi maju-mundur)
 ├─► ALIGN_ROTATION       (koreksi sudut kecil, pulse)
 ├─► ALIGN_LARGE_ROTATION (rotasi penuh > 30°)
 ├─► POST_ROT_CREEP       (creep setelah rotasi besar)
 ├─► WAITING_STABLE       (tunggu 1 detik sebelum eksekusi)
 ├─► EXECUTE_MOVE         (gerakan MAJU/MUNDUR dengan S-Curve PID)
 ├─► EXECUTE_ROTATION_90  (rotasi 90° berbasis IMU)
 └─► GOAL_REACHED         (berhenti di titik tujuan)
```

**Fitur Motor Control:**
- Kontrol arah via Digital I/O (CK5162E) dan kecepatan via Analog Output (CKDA08ETH)
- Trim otomatis kiri/kanan: `TRIM_L_AUTO = 0.96`, `TRIM_R_AUTO = 1.00`
- S-Curve acceleration pada fase `EXECUTE_MOVE`
- Dual PID: IMU (`KP=0.22, KI=0.01, KD=0.80`) + Encoder (`KP_ENC=0.06`)
- Stall guard: kecepatan < `V_STALL` dibulatkan ke `V_STALL` atau 0

**Input Fisik:**
| Pin | Fungsi |
|---|---|
| DI0 | Emergency Stop (NC — aktif saat LOW) |
| DI1 | Tombol START AUTO (rising edge) |
| DI2 | Tombol STOP AUTO / kembali MANUAL (rising edge) |

**MQTT Commands** (broker `192.168.3.100`):

| Pesan | Aksi |
|---|---|
| `START_AUTO` | Aktifkan mode AUTO |
| `RETURN_HOME` | Cari QR HOMEPOST dan berhenti |
| `STOP_AUTO` / `STOP` | Kembali ke MANUAL |
| `W/A/S/D` | Manual control via MQTT |

**Logging:** Setiap gerakan EXECUTE_MOVE menghasilkan file `amr_diagnostic_log.csv` dengan kolom: `Time, Status, Dist_X, Offset_Awal_Y, Target_Yaw_SCurve, IMU_Yaw, Enc_L, Enc_R, Enc_Yaw, PID_Corr, Int_Err`.

---

### 2. `amr_qr_nav.py` — Sistem Visi QR

> Kelas dasar untuk pembacaan dan interpretasi QR Code dari kamera USB. Berdiri sendiri dan dapat diuji secara independen.

#### Fitur Utama

- **Inisialisasi kamera:** OpenCV `V4L2` pada resolusi 640×480, buffer size 1 (mengurangi lag)
- **Deteksi multi-QR:** `cv2.QRCodeDetector().detectAndDecodeMulti()` — semua QR dalam frame diproses bersamaan
- **Kalkulasi orientasi yang akurat:**
  ```
  Vektor dari titik tengah sisi bawah → sisi atas QR
  angle = atan2(dx, dy) dalam derajat
  ```
  Metode ini tahan terhadap distorsi perspektif kamera miring dibanding metode corner langsung.

- **Parsing perintah QR:** Format `CMD:VALUE` (contoh: `MAJU:300`, `GOAL:5`) atau hanya `CMD`

- **Blind zone saat meninggalkan QR:** Setelah eksekusi perintah MAJU/MUNDUR, sistem mengabaikan frame selama `blind_duration = 3.0 detik` agar tidak membaca QR yang sedang ditinggalkan.

- **Visualisasi overlay:** Outline QR, garis arah atas, crosshair, label error/angle ditampilkan live di window OpenCV.

#### Format Data QR yang Didukung

| Format QR | Contoh | Aksi |
|---|---|---|
| `MAJU` / `MUNDUR` | `MAJU` | Gerak linear |
| `GOAL:N` | `GOAL:3` | Berhenti di goal N |
| `PUTAR_KANAN` / `PUTAR_KIRI` | `PUTAR_KANAN` | Rotasi 90° |
| `HOMEPOST` | `HOMEPOST` | Penanda titik asal |

---

### 3. `trackless_module.py` — Sensor Fusion IMU + Encoder

> Modul odometri tanpa jalur fisik (trackless). Menggabungkan IMU serial dan 2 encoder CANopen untuk estimasi posisi dan koreksi lintasan.

#### Konfigurasi Hardware

| Parameter | Nilai |
|---|---|
| Diameter roda utama | 18.0 cm |
| Jarak antar roda (track width) | 37.7 cm |
| Resolusi encoder | 8192 PPR |
| Rasio gearbox (mekanis) | 1:1 (direct axis) |
| IMU port | `/dev/ttyUSB0`, 9600 baud |
| CAN adapter | `/dev/ttyACM0`, 125 kbps (SLCAN) |
| Node encoder kiri | CAN ID 1 |
| Node encoder kanan | CAN ID 2 |

#### Cara Kerja Sensor Fusion

```
IMU Yaw (absolut, 40% bobot)
        +
Encoder Differential (relatif, 60% bobot)
        ↓
    Fused Error
        ↓
   PID Controller
        ↓
  Correction Signal → set_motor()
```

**Kinematika diferensial:**
```
delta_L/R = (raw - offset) / PPR * keliling_roda
yaw_enc   = atan((dist_L - dist_R) / track_width)
```

#### Thread

| Thread | Fungsi |
|---|---|
| `_imu_thread` | Membaca byte serial dari IMU (protokol WIT), parse paket 11 byte, update `yaw_val` |
| `_encoder_thread` | Poll SDO CANopen index `0x6004` (posisi absolut) dari 2 node encoder @ 100Hz |

---

### 4. `app.py` — Web Dashboard Flask

> Server web yang menyajikan HMI (Human Machine Interface) berbasis browser. Menjalankan `AMRController` di background dan meneruskan live video + status ke browser.

#### Fitur

- **Live video stream** via MJPEG (`/video_feed`) — frame diambil dari `IntegratedQRSystem` via global buffer thread-safe
- **Monkey-patch** `process_stream`: menggantikan fungsi bawaan `QRNavigationSystem` dengan versi yang menulis ke `global_last_frame` agar bisa diakses Flask
- **REST API endpoints:**

| Endpoint | Method | Fungsi |
|---|---|---|
| `/` | GET | Halaman utama HMI |
| `/video_feed` | GET | MJPEG stream kamera |
| `/api/status` | GET | Status lengkap AMR (JSON) |
| `/api/connect` | POST | Hubungkan hardware |
| `/api/estop` | POST | Toggle E-Stop |
| `/api/mode` | POST | Ganti mode MANUAL/AUTO |
| `/api/manual/move` | POST | Gerak manual (arah) |
| `/api/manual/stop` | POST | Hentikan motor |

#### Payload `/api/status` (ringkasan)

```json
{
  "connected": true,
  "mode": "AUTO",
  "nav_mode": "EXECUTE_MOVE",
  "nav_status_detail": "S-CRV | Ty:0.5° | Y:0.3° | C:1.2",
  "estop": false,
  "lidar": { "dist_cm": 85.0, "is_stop": false, "is_slow": false, "active_rays": 12 },
  "vision_error": { "x": -5, "y": 3, "angle": 0.8, "x_cm": -0.5, "y_cm": 0.3 },
  "trackless": { "enc_l": 12.5, "enc_r": 12.4, "yaw": 0.2, "dist_traveled": 12.45 }
}
```

#### Startup Sequence

1. Buat instance `AMRController`
2. Pasang signal handler `SIGINT`/`SIGTERM` untuk graceful shutdown
3. Jalankan `rospy.spin()` di background thread (untuk callback lidar)
4. Jalankan Flask di `host=0.0.0.0, port=5050`

---

### 5. `encoder_idset.py` — Utilitas Konfigurasi Encoder

> Script satu-kali-jalan untuk mengubah Node ID encoder baru dari ID default pabrik (1) ke ID yang diinginkan (2) via CANopen SDO.

#### Prosedur

```
Connect SLCAN (/dev/ttyACM0, 125kbps)
    ↓
Add node ID 1 + load EDS
    ↓
Set state PRE-OPERATIONAL
    ↓
SDO write 0x3000:0 = Node ID baru (coba 1 byte, fallback 4 byte)
    ↓
SDO write 0x1010:1 = 0x65766173 ("save" → EEPROM)
    ↓
NMT Reset All (0x81)
    ↓
Power cycle encoder
```

> ⚠️ **Jalankan hanya sekali** per encoder. Setelah ID berubah, gunakan `trackless_module.py` untuk komunikasi rutin.

> 💡 Jika muncul `Permission Denied`: jalankan `sudo chmod 666 /dev/ttyACM0`

---

### 6. `index.html` — Tampilan HMI Dashboard

> Single-page dashboard yang berkomunikasi dengan `app.py` via fetch API. Auto-refresh status setiap 200ms.

#### Panel yang Tersedia

| Panel | Konten |
|---|---|
| **Video Feed** | Live stream kamera dengan overlay QR |
| **Mode Control** | Toggle MANUAL / AUTO, tombol E-Stop |
| **Manual Control** | Tombol arah (W/A/S/D) untuk gerak manual |
| **Navigation Status** | `nav_mode` dan `nav_status_detail` real-time |
| **Vision Error** | Error X/Y (pixel & cm), sudut QR |
| **Lidar Safety** | Jarak minimum, status STOP/SLOW, jumlah ray aktif |
| **Encoder/Odometry** | Jarak tempuh L/R, yaw IMU, jarak total |

---

### 7. `Encoder_.eds` — Definisi Objek CANopen

> File EDS (Electronic Data Sheet) standar CANopen untuk encoder rotary. Digunakan oleh library `canopen` Python untuk mengetahui struktur Object Dictionary encoder.

**Index kunci yang digunakan sistem:**

| Index | Sub | Fungsi |
|---|---|---|
| `0x6004` | 0 | Posisi absolut encoder (dibaca setiap 10ms) |
| `0x3000` | 0 | Node ID (ditulis saat setup via `encoder_idset.py`) |
| `0x1010` | 1 | Save ke EEPROM (`"save"` = `0x65766173`) |

---

## 🔄 Alur Navigasi Otomatis

```
[START AUTO]
     │
     ▼
  IDLE ──────────────────── QR tidak terdeteksi > 3 detik
     │                              │
     │ QR terdeteksi                ▼
     ▼                    QR_SEARCH_ROTATION
  Hitung error                     │
  (X, Y, angle)            QR ketemu? ──► IDLE
     │
     ├─► Y tidak aligned? ──► ALIGN_Y (maju/mundur)
     │
     ├─► |angle| > 30°? ──► ALIGN_LARGE_ROTATION ──► POST_ROT_CREEP
     │
     ├─► |angle| > 2°? ──► ALIGN_ROTATION (pulse)
     │
     └─► Semua aligned ──► WAITING_STABLE (1 detik)
                               │
                               ▼
                         EXECUTE_MOVE / EXECUTE_ROTATION_90
                               │
                         [Encoder: jarak tercapai atau timeout]
                               │
                               ▼
                         Cari QR berikutnya → IDLE
                               │
                          [QR = GOAL:N]
                               │
                               ▼
                         GOAL_REACHED → Tunggu START AUTO
```

---

## 🌐 Topologi Jaringan & Hardware

```
[PC/Tablet Browser]
        │  HTTP :5050
        ▼
[Raspberry Pi / Jetson]
   ├── Flask app.py
   ├── amr_controller.py
   ├── rospy (ROS1)
   │
   ├── /dev/video0  ──── USB Camera
   ├── /dev/ttyUSB0 ──── IMU Serial (WIT protocol)
   ├── /dev/ttyACM0 ──── CAN Adapter (SLCAN)
   │                      ├── Encoder Kiri  (CAN ID 1)
   │                      └── Encoder Kanan (CAN ID 2)
   │
   ├── 192.168.1.30 ──── CKDA08ETH (Analog Output 0-5V)
   ├── 192.168.2.30 ──── CK5162E   (Digital I/O)
   └── 192.168.3.100 ─── MQTT Broker

[ROS] /sick_safetyscanners/scan ← SICK nanoScan3 Safety Lidar
```

---

## 📦 Dependensi

```
# Python packages
opencv-python        # Deteksi QR, capture kamera
numpy                # Kalkulasi array
flask                # Web server HMI
paho-mqtt            # Komunikasi MQTT
canopen              # Komunikasi encoder via CAN
pyserial             # Komunikasi IMU serial
rospy                # ROS1 (untuk lidar SICK)
sensor_msgs          # ROS message type LaserScan

# Custom modules (tidak termasuk di repo ini)
analog_digital       # Driver CK5162E & CKDA08ETH (Ethernet I/O)
```

---

## ▶️ Cara Menjalankan

### Mode Standalone (visi saja, tanpa hardware)
```bash
python3 amr_qr_nav.py
```

### Mode Controller penuh (terminal keyboard)
```bash
python3 amr_controller.py
# W/A/S/D → gerak manual | Space → stop | Q / Ctrl+C → keluar
```

### Mode Web Dashboard (HMI)
```bash
# Pastikan roscore dan driver lidar sudah berjalan
roscore &
roslaunch sick_safetyscanners sick_safetyscanners.launch &

# Jalankan Flask
python3 app.py
# Buka browser: http://<IP_ROBOT>:5050
```

### Setup encoder baru (satu kali)
```bash
python3 encoder_idset.py
# Pastikan hanya 1 encoder terhubung saat menjalankan ini
```

---

> 📌 **Catatan:** Seluruh sistem dirancang untuk berjalan di Linux (Ubuntu/Raspbian). Pastikan user memiliki akses ke `/dev/ttyUSB0` dan `/dev/ttyACM0`:
> ```bash
> sudo usermod -aG dialout $USER
> ```
