Panduan Sistem Navigasi & HMI Industrial AMR (Autonomous Mobile Robot)

Dokumentasi ini menjelaskan arsitektur perangkat lunak, fungsionalitas berkas, mekanisme kontrol, dan panduan operasional untuk sistem kemudi AMR berbasis navigasi QR Code, sensor fusion (IMU + Encoder), serta sistem keamanan LiDAR.

📌 Daftar Isi

Arsitektur Sistem & Aliran Data

Penjelasan Masing-Masing File

Mekanisme Kontrol Navigasi & Sensor Fusion

Sistem Keselamatan LiDAR (SICK Nano)

Spesifikasi HMI & Web Dashboard

Panduan Instalasi & Cara Menjalankan

🏗️ Arsitektur Sistem & Aliran Data

Sistem AMR ini mengintegrasikan pemrosesan citra (visi komputer), kontrol PID pergerakan motor, pembacaan sensor odometri (Trackless/CANopen Encoder), serta perlindungan tabrakan berbasis ROS LiDAR.

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


📂 Penjelasan Masing-Masing File

1. amr_controller.py (Otak Utama AMR)

Berkas ini bertindak sebagai Main Controller yang mengatur logika navigasi otonom dan manual, state machine robot, serta antarmuka dengan modul perangkat keras fisik.

Fungsi Utama:

Menginisialisasi Node ROS (amr_controller) untuk berlangganan data sensor LiDAR SICK.

Menjalankan thread kamera latar belakang (start_camera_thread) yang mengaktifkan sistem visi.

Menghubungkan driver I/O Digital (CK5162E) dan Analog DAC (CKDA08ETH) untuk menggerakkan driver motor BLDC 400W secara diferensial.

Mengimplementasikan kontrol loop navigasi (control_loop dan handle_auto_navigation) dengan toleransi dinamis.

Memproses tombol fisik industri (E-Stop pada DI0, Start Auto pada DI1, Stop Auto pada DI2) dengan algoritma debouncing.

Mengatur kendali darurat via protokol MQTT broker untuk kemudahan integrasi dengan sistem WMS (Warehouse Management System).

2. amr_qr_nav.py (Sistem Visi Kamera & Deteksi QR)

Modul ini menangani pemrosesan citra dari kamera bawah yang mengarah ke lantai untuk mencari markah QR Code.

Fungsi Utama:

Menggunakan OpenCV (cv2.QRCodeDetector) untuk menangkap frame kamera dan mendekode isi QR Code.

Menghitung orientasi rotasi (kemiringan sudut) QR Code terhadap sumbu kamera dengan algoritma proyeksi vektor:


$$\theta_{rotasi} = \text{atan2}(dx, dy)$$


Vektor dihitung dari titik tengah bawah QR ke titik tengah atas QR, sehingga sangat toleran terhadap distorsi kemiringan fisik kamera (camera tilt perspective).

Menyediakan fungsi visualisasi overlay (menggambar kotak batas QR, garis koordinat, dan arah vektor hadap QR) sebelum mengirim data penyimpangan (error $e_x$, $e_y$, dan $\theta$) ke pengendali utama.

3. app.py (Web Server & API Gateway)

Aplikasi berbasis Flask yang menjembatani kontrol internal robot dengan antarmuka pengguna (HMI).

Fungsi Utama:

Monkey Patching: Melakukan modifikasi dinamis pada metode IntegratedQRSystem.process_stream agar frame kamera yang sedang diproses oleh detektor QR dapat disalurkan secara simultan ke web browser dalam format MJPEG Stream (/video_feed).

Menyediakan API RESTful untuk memantau status telemetri robot secara real-time (/api/status), memicu koneksi hardware (/api/connect), memicu E-stop soft (/api/estop), mengubah mode kerja (/api/mode), dan mengontrol pergerakan manual (/api/manual/move & /api/manual/stop).

4. index.html (HMI Dashboard Web)

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

A. Aliran Logika Penyelarasan (Alignment)

Saat robot mendeteksi QR Code baru, ia akan melakukan tahapan penyelarasan posisi sebelum mengeksekusi instruksi pergerakan:

Koreksi Geser Samping (Sumbu Y): Jika error $y$ di luar batas toleransi deadzone_y (25 px), AMR akan masuk ke mode ALIGN_Y dengan sistem pulsa dinamis (pulse-move) demi menghindari slip pada roda.

Koreksi Rotasi Kasar (Sudut Ekstrem > 30°): Jika sudut kemiringan $\theta > 30^\circ$, robot akan mengeksekusi rotasi cepat searah jarum jam atau berlawanan arah lewat mode ALIGN_LARGE_ROTATION. Target sudut ditentukan secara dinamis menggunakan rumus:


$$\theta_{target} = \text{clamp}(|\theta| - 10^\circ, \, \text{min}=30^\circ, \, \text{max}=170^\circ)$$

Koreksi Rotasi Halus (Sudut Kecil < 30°): Robot bergerak presisi dalam mode ALIGN_ROTATION secara perlahan agar sumbu hadap lurus tegak terhadap marka lantai.

Proteksi Anti-Osilasi: Jika robot terdeteksi berosilasi bolak-balik melewati batas deadzone lebih dari 10 kali (OSCILLATION_MAX), sistem akan secara otomatis melompati fase alignment dan langsung mengeksekusi perintah gerak berikutnya untuk efisiensi waktu kerja.

B. Kontrol Jalur S-Curve Lintasan (EXECUTE_MOVE)

Saat bergerak maju antar-QR, robot menggunakan S-Curve lateral correction berbasis Sensor Fusion untuk memastikan gerakan berjalan lurus secara presisi:

Deviasi posisi awal sumbu $X$ dari QR diubah menjadi parameter offset awal lintasan ($scurve\_offset\_cm$).

Target sudut dinamis ($\theta_{target}$) dikalkulasi sepanjang jarak tempuh ($x$) menggunakan turunan pertama kurva polinomial:


$$y'(x) = \frac{6 \cdot \text{offset}}{\text{active\_dist}} \left( \frac{x}{\text{active\_dist}} - \left(\frac{x}{\text{active\_dist}}\right)^2 \right)$$

$$\theta_{target}(x) = \arctan(y'(x))$$

Deviasi aktual didapatkan dari gabungan sensor IMU Yaw dan selisih pembacaan Encoder roda kiri-kanan:


$$u(t) = K_{p} \cdot e_{\theta}(t) + K_{i} \cdot \int e_{\theta}(t)\,dt + K_{d} \cdot \frac{de_{\theta}(t)}{dt} + K_{p\_enc} \cdot e_{encoder}(t)$$

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

Pastikan pustaka pendukung berikut telah terinstal pada sistem operasi (direkomendasikan Linux Ubuntu dengan ROS Noetic):

# Instalasi pustaka Python penting
pip3 install flask opencv-python numpy paho-mqtt python-canopen

# Pastikan Driver Serial & Port USB CAN memiliki izin akses
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyUSB0


B. Langkah Menjalankan Aplikasi

Jalankan ROS Node & Core (jika menggunakan Lidar Fisik):

roscore
# (Pastikan driver sick_safetyscanners sudah aktif dan mempublikasikan data ke /sick_safetyscanners/scan)


Jalankan Aplikasi Web HMI & Kontroler Utama:
Jalankan file app.py yang secara otomatis akan menginisialisasi sistem kontrol utama AMRController:

sudo python3 app.py


Server web Flask akan mulai berjalan pada alamat http://0.0.0.0:5050.

Akses Dashboard Pengguna:
Buka peramban web (Google Chrome/Firefox) di PC lokal atau tablet yang terhubung dalam satu jaringan Wi-Fi AMR, lalu akses alamat IP robot:

http://<IP_ROBOT_AMR>:5050


Operasional:

Klik tombol RECONNECT HARDWARE di bagian bawah telemetry panel untuk menyinkronkan koneksi DAC dan I/O controller.

Gunakan tombol MANUAL untuk menggerakkan robot dengan tombol arah virtual.

Tekan tombol AUTO RUN untuk memindahkan robot ke mode otonom agar mengikuti markah jalan QR Code di lantai pabrik.
