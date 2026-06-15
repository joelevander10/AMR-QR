import cv2
import numpy as np
import math
import time

class QRNavigationSystem:
    def __init__(self, camera_id=0):
        """
        Inisialisasi Sistem Navigasi berbasis QR Code.
        """
        self.cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            print(f"[ERROR] Port kamera {camera_id} tidak merespon atau tidak ditemukan.")
            return 

        self.width = 640
        self.height = 480
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.center_x = self.width // 2
        self.center_y = self.height // 2
        self.detector = cv2.QRCodeDetector()

        self.deadzone_y = 30      
        self.deadzone_angle = 2.0 
        
        self.is_executing = False         
        self.execution_start_time = 0     
        self.execution_duration = 3.0     
        self.last_command = "NONE"        
        self.blind_duration = 3.0 

        print(f"[INFO] Kamera berhasil diinisialisasi pada sumber: {camera_id}")

    def calculate_orientation(self, corners):
        """
        [DIPERBARUI] Menghitung sudut rotasi QR terhadap frame kamera.
        Mencari vektor dari tengah-bawah QR ke tengah-atas QR.
        Ini jauh lebih tahan terhadap distorsi perspektif kamera miring.
        """
        # corners format: [Top-Left, Top-Right, Bottom-Right, Bottom-Left]
        # Pastikan titik berupa float untuk perhitungan rata-rata
        pts = corners.astype(float)
        
        # Cari titik tengah sisi atas dan sisi bawah
        top_center_x = (pts[0][0] + pts[1][0]) / 2.0
        top_center_y = (pts[0][1] + pts[1][1]) / 2.0
        
        bottom_center_x = (pts[2][0] + pts[3][0]) / 2.0
        bottom_center_y = (pts[2][1] + pts[3][1]) / 2.0

        # Menghitung selisih X dan Y (Vektor menghadap ke depan/atas)
        dx = top_center_x - bottom_center_x
        # dy dibalik karena sumbu Y pada pixel kamera bertambah ke arah bawah
        dy = bottom_center_y - top_center_y 
        
        # math.atan2(dx, dy) menghasilkan 0 derajat jika QR tegak lurus
        # Positif jika QR miring ke kanan, negatif jika miring ke kiri
        angle_rad = math.atan2(dx, dy)
        return math.degrees(angle_rad)

    def parse_command(self, qr_data):
        try:
            if ":" in qr_data:
                cmd, val = qr_data.split(":")
                return cmd.upper(), val
            return qr_data.upper(), "0"
        except:
            return "ERROR", "0"

    def process_stream(self):
        print(f"[INFO] Thread pemrosesan visi dimulai...")
        while True:
            ret, frame = self.cap.read()
            if not ret: 
                time.sleep(0.1)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            current_time = time.time()
            
            if self.is_executing:
                elapsed = current_time - self.execution_start_time
                is_moving_cmd = "MAJU" in self.last_command or "MUNDUR" in self.last_command

                if is_moving_cmd:
                    if elapsed < self.blind_duration:
                        cv2.rectangle(frame, (0, 0), (self.width, 60), (0, 0, 150), -1)
                        cv2.putText(frame, f"STATUS: MENINGGALKAN QR ({self.blind_duration - elapsed:.1f}s)", 
                                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                        cv2.imshow("AMR Vision System", frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'): break
                        continue 
                    else:
                        cv2.rectangle(frame, (0, 0), (self.width, 60), (0, 100, 0), -1)
                        cv2.putText(frame, "STATUS: MENCARI QR SELANJUTNYA...", 
                                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            retval, decoded_info, points, _ = self.detector.detectAndDecodeMulti(gray)

            if retval:
                if self.is_executing and (time.time() - self.execution_start_time > self.blind_duration):
                    self.is_executing = False
                    print("\n[ALERT] QR BARU TERDETEKSI! Memulai proses Alignment...")

                for i, qr_data in enumerate(decoded_info):
                    if not qr_data: continue
                    
                    qr_corners = points[i] # Gunakan float murni dahulu
                    qr_center_x = int(np.mean(qr_corners[:, 0]))
                    qr_center_y = int(np.mean(qr_corners[:, 1]))

                    error_x = qr_center_x - self.center_x
                    error_y = self.center_y - qr_center_y 
                    
                    # Panggil fungsi orientasi yang sudah diperbaiki
                    angle = self.calculate_orientation(qr_corners)
                    cmd, val = self.parse_command(qr_data)

                    # Visualisasi
                    int_corners = qr_corners.astype(int)
                    for j in range(4):
                        cv2.line(frame, tuple(int_corners[j]), tuple(int_corners[(j+1)%4]), (0, 255, 0), 2)
                    
                    # Indikator orientasi: Garis dari pusat ke arah atas QR
                    top_cx = int((int_corners[0][0] + int_corners[1][0]) / 2)
                    top_cy = int((int_corners[0][1] + int_corners[1][1]) / 2)
                    cv2.line(frame, (qr_center_x, qr_center_y), (top_cx, top_cy), (0, 0, 255), 3)
                    
                    cv2.circle(frame, (qr_center_x, qr_center_y), 5, (0, 0, 255), -1)
                    cv2.line(frame, (self.center_x, 0), (self.center_x, self.height), (0, 255, 255), 1)
                    cv2.line(frame, (0, self.center_y), (self.width, self.center_y), (0, 255, 255), 1)

                    pt_tl = int_corners[0]
                    pt_tr = int_corners[1]
                    qr_w_px = math.sqrt((pt_tr[0] - pt_tl[0])**2 + (pt_tr[1] - pt_tl[1])**2)
                    
                    cv2.putText(frame, f"ID: {cmd} | Ang: {angle:.1f}", (qr_center_x + 10, qr_center_y - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    self.control_logic(frame, error_x, error_y, angle, cmd, val, qr_w_px)

            else:
                if not self.is_executing:
                    cv2.putText(frame, "STANDBY - NO QR IN SIGHT", (20, 40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            cv2.imshow("AMR Vision System", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

    def control_logic(self, frame, ex, ey, angle, cmd, val, qr_w_px):
        pass

if __name__ == "__main__":
    try:
        print("[TEST] Memulai pengujian visi mandiri...")
        amr_vision = QRNavigationSystem(camera_id=0)
        amr_vision.process_stream()
    except KeyboardInterrupt:
        print("\n[INFO] Pengujian dihentikan oleh pengguna.")
