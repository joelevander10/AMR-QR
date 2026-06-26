# 🤖 Industrial AMR (Autonomous Mobile Robot) - Navigation & HMI System

Repositori ini berisi sistem navigasi otonom berbasis QR Code, integrasi Sensor Fusion (IMU + Encoder), serta sistem keamanan LiDAR untuk robot industri Autonomous Mobile Robot (AMR). Proyek ini dilengkapi dengan antarmuka HMI (Human-Machine Interface) berbasis web interaktif.

# 📌 Daftar Isi

- 🏗️ Arsitektur Sistem & Aliran Data

- 📁 Struktur Direktori Proyek

- 📂 Penjelasan Berkas & Komponen

- ⚙️ Mekanisme Kontrol Navigasi

    A. Logika Penyelarasan (Alignment)

    B. Kontrol Jalur S-Curve Lintasan

- 🛡️ Sistem Keselamatan LiDAR (SICK Nano)

- 💻 Spesifikasi HMI & Web Dashboard

- 🚀 Panduan Instalasi & Cara Menjalankan

- 🏗️ Arsitektur Sistem & Aliran Data

Sistem AMR ini mengintegrasikan pemrosesan citra komputer (Computer Vision), kendali PID pergerakan motor diferensial, pembacaan umpan balik odometri (Trackless/CANopen Encoder), serta sistem keselamatan LiDAR berbasis ROS.

       [ Kamera USB ] -> Deteksi QR (amr_qr_nav.py)
              |
              v (Kalkulasi error X, Y, Sudut)
      [ app.py (Flask) ] <======== API (Web HTTP/MJPEG) ========> [ HMI Dashboard (index.html) ]
              |                                                               ^
              v (State & Command)                                             |
     [ amr_controller.py ] <================== telemetry =====================+
       /      |      \
      /       |       \
     v        v        v
 [Motor]   [LiDAR]   [Trackless (IMU + Encoder)]
(Analog)   (ROS)     (CAN / Serial)


📁 Struktur Direktori Proyek

Agar aplikasi Flask dan pengontrol dapat berjalan dengan lancar, pastikan struktur berkas di dalam repositori Anda disusun seperti berikut:

amr-navigation-system/
├── templates/
│   └── index.html         # Tampilan HMI Dashboard (Frontend)
├── Encoder_.eds           # Electronic Data Sheet CANopen untuk Encoder
├── README.md              # Dokumentasi Proyek
├── amr_controller.py      # Otak Utama & Integrasi Kontrol AMR
├── amr_qr_nav.py          # Modul Pengolahan Citra & Deteksi QR
├── app.py                 # Flask Web Server & API Gateway
└── encoder_idset.py       # Utilitas Konfigurasi ID Encoder CANopen


📂 Penjelasan Berkas & Komponen

1. amr_controller.py (Otak Utama AMR)

Bertindak sebagai Main Controller yang mengatur logika navigasi otonom dan manual, state machine robot, serta antarmuka dengan modul perangkat keras fisik.

Fungsi Utama:

Menginisialisasi Node ROS (amr_controller) untuk berlangganan data sensor LiDAR SICK.

Menjalankan thread kamera latar belakang (start_camera_thread) yang mengaktifkan sistem visi.

Menghubungkan driver I/O Digital (CK5162E) dan Analog DAC (CKDA08ETH) untuk menggerakkan roda motor BLDC 400W secara diferensial.

Memproses tombol fisik industri (E-Stop pada DI0, Start Auto pada DI1, Stop Auto pada DI2) dengan algoritma debouncing.

Mendukung kendali jarak jauh via protokol broker MQTT untuk integrasi dengan Warehouse Management System (WMS).

2. amr_qr_nav.py (Sistem Visi Kamera & Deteksi QR)

Modul ini menangani pemrosesan citra dari kamera bawah yang mengarah ke lantai untuk mencari markah jalan berupa QR Code.

Fungsi Utama:

Menggunakan OpenCV (cv2.QRCodeDetector) untuk menangkap frame kamera dan mendekode isi QR Code.

Menghitung orientasi rotasi (kemiringan sudut) QR Code terhadap sumbu kamera dengan algoritma proyeksi vektor:


$$\theta_{\text{rotasi}} = \text{atan2}(dx, dy)$$


Vektor dihitung dari titik tengah bawah QR ke titik tengah atas QR, sehingga sangat toleran terhadap distorsi kemiringan fisik kamera (camera tilt perspective).

Menyediakan visualisasi overlay (menggambar kotak batas QR, sumbu koordinat, arah hadap QR) sebelum mengirim data penyimpangan ($e_x$, $e_y$, dan $\theta$) ke pengendali utama.

3. app.py (Web Server & API Gateway)

Aplikasi berbasis Flask yang menjembatani kontrol internal robot dengan antarmuka pengguna (HMI).

Fungsi Utama:

Monkey Patching: Melakukan modifikasi dinamis pada metode IntegratedQRSystem.process_stream agar frame kamera yang sedang diproses oleh detektor QR dapat disalurkan secara simultan ke web browser dalam format MJPEG Stream (/video_feed).

Menyediakan API RESTful untuk memantau status telemetri robot secara real-time (/api/status), memicu koneksi hardware (/api/connect), memicu E-stop soft (/api/estop), mengubah mode kerja (/api/mode), dan mengontrol pergerakan manual (/api/manual/move & /api/manual/stop).

4. templates/index.html (HMI Dashboard Web)

Halaman depan (Front-end) web dashboard modern berbasis Bootstrap 5 dan Font Awesome dengan gaya bertema gelap (dark mode) yang dioptimalkan untuk layar tablet industri / IPC (Industrial PC) pada bodi AMR.

Fungsi Utama:

Menampilkan video feed real-time dengan overlay hasil deteksi QR.

Menyajikan widget visualisasi status kritis: E-Stop State, Mode Navigasi, Jarak LiDAR SICK, Posisi Odometri (Encoder Kiri/Kanan), IMU Yaw, dan nilai deviasi sumbu QR ($X$ & $Y$ dalam satuan cm).

Menyediakan panel kontrol manual berupa digital virtual joystick yang mendukung event sentuh (touch events) untuk pengoperasian langsung di lapangan.

5. encoder_idset.py (Utilitas Konfigurasi ID Encoder)

Skrip bantu mandiri yang digunakan saat proses komisioning awal atau perawatan hardware encoder roda.

Fungsi Utama:

Berkomunikasi via protokol CANopen menggunakan library canopen melalui adapter USB-to-CAN SLCAN (/dev/ttyACM0).

Memindahkan Node ID Encoder dari ID default pabrik (Node 1) ke Node ID target (Node 2) agar tidak bentrok pada jaringan bus CAN robot.

Menyimpan perubahan konfigurasi ke dalam EEPROM internal encoder secara permanen dengan mengirimkan tanda khusus save (0x65766173) pada indeks Object Dictionary 0x1010:01.

6. Encoder_.eds (Electronic Data Sheet CANopen)

Berkas konfigurasi standar yang mendeskripsikan struktur Object Dictionary dari encoder roda CANopen (seri AM50).

Fungsi Utama:

Digunakan oleh encoder_idset.py dan kelas TracklessSystem untuk memetakan alamat memori internal encoder, seperti pembacaan posisi nilai encoder (0x6004), konfigurasi baudrate (0x2003), dan pengaturan Node ID (0x2004).

⚙️ Mekanisme Kontrol Navigasi & Sensor Fusion

A. Logika Penyelarasan (Alignment)

Saat robot mendekati QR Code baru, ia akan melakukan beberapa tahapan penyelarasan posisi sebelum mengeksekusi instruksi pergerakan selanjutnya:

Koreksi Geser Samping (Sumbu Y): Jika error $y$ di luar batas toleransi deadzone_y ($25\text{ px}$), AMR akan masuk ke mode ALIGN_Y menggunakan sistem pergerakan pulsa (pulse-move) demi menghindari slip pada roda.

Koreksi Rotasi Kasar (Sudut Ekstrem > 30°): Jika sudut kemiringan $\theta > 30^\circ$, robot akan mengeksekusi rotasi cepat searah jarum jam atau berlawanan arah lewat mode ALIGN_LARGE_ROTATION. Target sudut ditentukan secara dinamis menggunakan rumus:


$$\theta_{\text{target}} = \text{clamp}(|\theta| - 10^\circ, \, \text{min}=30^\circ, \, \text{max}=170^\circ)$$

Koreksi Rotasi Halus (Sudut Kecil < 30°): Robot bergerak presisi dalam mode ALIGN_ROTATION secara perlahan agar sumbu hadap lurus tegak terhadap marka lantai.

Proteksi Anti-Osilasi: Jika robot terdeteksi berosilasi bolak-balik melewati batas deadzone lebih dari 10 kali (OSCILLATION_MAX), sistem akan secara otomatis melompati fase alignment dan langsung mengeksekusi perintah gerak berikutnya untuk efisiensi waktu kerja.

B. Kontrol Jalur S-Curve Lintasan (EXECUTE_MOVE)

Saat bergerak maju antar-QR, robot menggunakan S-Curve lateral correction berbasis Sensor Fusion untuk memastikan gerakan berjalan lurus secara presisi:

Deviasi posisi awal sumbu $X$ dari QR diubah menjadi parameter offset awal lintasan ($scurve\_offset\_cm$).

Target sudut dinamis ($\theta_{\text{target}}$) dikalkulasi sepanjang jarak tempuh ($x$) menggunakan turunan pertama kurva polinomial:


$$y'(x) = \frac{6 \cdot \text{offset}}{\text{active\_dist}} \left( \frac{x}{\text{active\_dist}} - \left(\frac{x}{\text{active\_dist}}\right)^2 \right)$$

$$\theta_{\text{target}}(x) = \arctan(y'(x))$$

Deviasi aktual didapatkan dari gabungan sensor IMU Yaw dan selisih pembacaan Encoder roda kiri-kanan:


$$u(t) = K_{p} \cdot e_{\theta}(t) + K_{i} \cdot \int e_{\theta}(t)\,dt + K_{d} \cdot \frac{de_{\theta}(t)}{dt} + K_{p\_enc} \cdot e_{\text{encoder}}(t)$$

Sinyal koreksi kontrol $u(t)$ diumpankan balik untuk membedakan voltase kecepatan motor kiri dan motor kanan.

🛡️ Sistem Keselamatan LiDAR (SICK Nano)

AMR dilengkapi dengan sensor keselamatan laser scanner LiDAR SICK yang terintegrasi melalui ROS (Robot Operating System) pada topik /sick_safetyscanners/scan.

Zona Pantau Depan: Dibatasi ketat pada sudut depan $-30^\circ$ hingga $+30^\circ$ untuk memfokuskan deteksi halangan tepat di jalur lintasan robot.

Filter Noise Validasi: Mengabaikan objek di bawah $0.15\text{ m}$ untuk membuang anomali pantulan piringan logam robot sendiri (SICK laser artifact reflection).

Skema Respons Jarak:

Zona Aman ($d > 1.0\text{ m}$): Kecepatan motor berjalan normal 100%.

Zona Peringatan ($0.6\text{ m} \le d \le 1.0\text{ m}$): Robot memasuki status SLOW, kecepatan putaran motor diredam sebesar 50% dari kecepatan jelajah.

Zona Bahaya ($d < 0.6\text{ m}$): Robot memasuki status STOP, rem elektromagnetik aktif secara instan untuk menghentikan laju robot demi menghindari tabrakan fisik.

💻 Spesifikasi HMI & Web Dashboard

HMI web dirancang agar mudah digunakan oleh operator pabrik langsung di lapangan:

Komponen Visual

Deskripsi Fungsi

Indikator State

Badge Offline/Online

Status komunikasi antara HMI dan Web Server robot

Hijau (Online) / Merah (Offline)

Video Feed Overlay

Streaming visualisasi sensor kamera dengan proyeksi sudut QR

Overlay garis hijau jika terdeteksi

Lidar Safety Badge

Menampilkan status zonasi jarak hambatan secara cepat

✔ OK (Aman) / ⚠ SLOW (Melambat) / ⛔ STOP (Bahaya)

Telemetry Panel

Data numerik IMU, Target S-Curve, Odometri roda, dan Error QR

Diperbarui secara berkala setiap $200\text{ ms}$

Virtual Joystick

Tombol navigasi manual arah (Maju, Mundur, Rotasi)

Mendukung klik mouse & sentuhan jari (touch screen)

E-Stop Button

Tombol darurat lunak (soft emergency stop)

Animasi berkedip merah saat aktif

🚀 Panduan Instalasi & Cara Menjalankan

A. Prasyarat Sistem & Dependensi

Sistem operasi yang direkomendasikan adalah Linux Ubuntu 20.04 LTS dengan ROS Noetic.

Instal pustaka Python yang diperlukan:

# Update package list
sudo apt update

# Instal dependensi Python utama
pip3 install flask opencv-python numpy paho-mqtt python-canopen

# Berikan hak akses untuk port hardware serial & USB-to-CAN
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyUSB0


B. Langkah Menjalankan Aplikasi

Jalankan ROS Core dan Driver LiDAR SICK (jika menggunakan LiDAR fisik):

roscore
# (Gunakan terminal terpisah untuk menjalankan driver sick_safetyscanners)


Jalankan Aplikasi Web Server HMI & Kontroler Utama:
Jalankan file app.py yang secara otomatis akan menginisialisasi sistem kontrol utama AMRController:

sudo python3 app.py


Server web Flask akan mulai berjalan pada alamat http://0.0.0.0:5050.

Akses Dashboard Pengguna:
Buka peramban web (Google Chrome / Mozilla Firefox) di tablet HMI atau laptop yang terhubung dalam satu jaringan Wi-Fi AMR, lalu akses alamat IP robot:

http://<IP_ROBOT_AMR>:5050


Operasional:

Klik tombol RECONNECT HARDWARE di bagian bawah telemetry panel untuk menyinkronkan koneksi DAC dan I/O controller.

Gunakan tombol MANUAL untuk menguji pergerakan roda dengan tombol arah virtual.

Tekan tombol AUTO RUN untuk memindahkan robot ke mode otonom agar mengikuti markah jalan QR Code di lantai pabrik.

[!IMPORTANT]

Selalu pastikan area depan AMR bebas dari halangan fisik saat pertama kali mengaktifkan AUTO RUN. Gunakan tombol E-STOP fisik atau tombol darurat pada Web Dashboard apabila terjadi deviasi pergerakan yang membahayakan.
