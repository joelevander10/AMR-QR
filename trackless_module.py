import serial
import time
import math
import threading
import canopen

class TracklessSystem:
    def __init__(self, port="/dev/ttyUSB0", baudrate=9600, can_channel="/dev/ttyACM0"):
        """
        Modul Sensor Fusion: Menggabungkan IMU (Absolut) dan Encoder (Relatif Kecepatan Tinggi)
        menggunakan perhitungan Kinematika Diferensial.
        """
        # --- KONFIGURASI FISIK AGV DARI USER ---
        self.DIA_MAIN = 18.0      # Diameter Roda Utama (cm)
        self.RATIO = 1.0          # Rasio Mekanis = 1:1 (Direct Axis)
        self.TRACK_WIDTH = 37.7   # Jarak antar titik pusat ban utama (cm)
        
        # --- HASIL KALIBRASI DARI test_encoder.py ---
        self.PPR = 8192.0         # Resolusi Encoder (Telah dikalibrasi 2x lipat)
        
        # Ganti ke -1 jika arah putaran salah satu ban terbalik saat didorong maju
        self.ARAH_KIRI = 1        
        self.ARAH_KANAN = 1       
        
        # --- KONFIGURASI IMU ---
        self.imu_port = port
        self.imu_baud = baudrate
        self.yaw_val = 0.0
        self.offset_yaw = None
        self.imu_state = 0
        self.data_len = 0
        self.checksum = 0
        self.rx_buf = [0] * 11
        
        # --- KONFIGURASI 2 NODE ENCODER ---
        self.can_channel = can_channel
        self.encoder_left_id = 1   # Node ID 1 untuk Motor Kiri
        self.encoder_right_id = 2  # Node ID 2 untuk Motor Kanan
        
        self.dist_l = 0.0
        self.dist_r = 0.0
        self.offset_l = None
        self.offset_r = None
        
        # --- VARIABEL KONTROL & PID ---
        self.is_running = False
        self.target_yaw = 0.0
        self.target_enc_diff = 0.0
        
        self.kp = 1.0
        self.ki = 0.01
        self.kd = 0.5
        self.integral = 0.0
        self.prev_error = 0.0
        
        self.lock = threading.Lock()
        
        # --- JALANKAN THREAD HARDWARE ---
        threading.Thread(target=self._imu_thread, daemon=True).start()
        threading.Thread(target=self._encoder_thread, daemon=True).start()
        
        print(f"[TRACKLESS] Kinematik diset: As Langsung, Track {self.TRACK_WIDTH}cm")
        print(f"[TRACKLESS] Node KIRI: ID {self.encoder_left_id} | Node KANAN: ID {self.encoder_right_id}")

    def set_pid_tunings(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def start_locking(self):
        with self.lock:
            self.target_yaw = self.yaw_val
            self.target_enc_diff = self.dist_l - self.dist_r
            self.integral = 0.0
            self.prev_error = 0.0
            self.is_running = True

    def get_yaw(self):
        return self.yaw_val

    def get_correction(self):
        if not self.is_running:
            return 0.0, 0.0, 0.0
            
        with self.lock:
            error_imu = self.yaw_val - self.target_yaw
            error_imu = (error_imu + 180) % 360 - 180  
            
            current_enc_diff = self.dist_l - self.dist_r
            delta_diff = current_enc_diff - self.target_enc_diff
            error_enc_rad = delta_diff / self.TRACK_WIDTH
            error_enc_deg = math.degrees(error_enc_rad)
            
            fused_error_deg = (error_imu * 0.4) + (error_enc_deg * 0.6)
            
            self.integral += fused_error_deg
            self.integral = max(-50.0, min(50.0, self.integral)) 
            
            derivative = fused_error_deg - self.prev_error
            correction = (self.kp * fused_error_deg) + (self.ki * self.integral) + (self.kd * derivative)
            self.prev_error = fused_error_deg
            
        return correction, error_imu, error_enc_deg

    def _imu_thread(self):
        try:
            ser = serial.Serial(self.imu_port, self.imu_baud, timeout=0.01)
            ser.flushInput()
            while True:
                waiting = ser.in_waiting
                if waiting > 0:
                    for byte in ser.read(waiting):
                        self._parse_imu_byte(byte)
                time.sleep(0.005)
        except Exception as e: print(f"[IMU FATAL] {e}")

    def _parse_imu_byte(self, byte):
        if byte == 0x55 and self.imu_state == 0:
            self.imu_state = 1; self.data_len = 11; self.checksum = 0; self.rx_buf = [0]*11
        if self.imu_state == 1:
            self.checksum += byte; self.rx_buf[11 - self.data_len] = byte; self.data_len -= 1
            if self.data_len == 0:
                self.checksum = (self.checksum - byte) & 0xff 
                self.imu_state = 0 
                if self.rx_buf[10] == self.checksum and self.rx_buf[1] == 0x53:
                    rzl, rzh = self.rx_buf[6], self.rx_buf[7]
                    raw_yaw = (rzh << 8 | rzl) / 32768.0 * 180.0
                    if raw_yaw >= 180.0: raw_yaw -= 360.0
                    if self.offset_yaw is None: self.offset_yaw = raw_yaw
                    with self.lock: self.yaw_val = ((raw_yaw - self.offset_yaw) + 180.0) % 360.0 - 180.0

    def _encoder_thread(self):
        network = canopen.Network()
        try:
            network.connect(bustype='slcan', channel=self.can_channel, bitrate=125000)
            
            enc_l = network.add_node(self.encoder_left_id, 'Encoder_.eds')
            enc_r = network.add_node(self.encoder_right_id, 'Encoder_.eds')
            
            enc_l.nmt.state = 'OPERATIONAL'
            enc_r.nmt.state = 'OPERATIONAL'
            time.sleep(0.5)
            
            self.offset_l = enc_l.sdo[0x6004].raw
            self.offset_r = enc_r.sdo[0x6004].raw
            keliling_utama = math.pi * self.DIA_MAIN
            
            while True:
                try:
                    raw_l = enc_l.sdo[0x6004].raw
                    raw_r = enc_r.sdo[0x6004].raw
                    
                    delta_l = raw_l - self.offset_l 
                    delta_r = raw_r - self.offset_r
                    
                    # --- TERAPKAN KALIBRASI ARAH DARI test_encoder.py ---
                    delta_l_arah_benar = delta_l * self.ARAH_KIRI
                    delta_r_arah_benar = delta_r * self.ARAH_KANAN
                    
                    putaran_l = (delta_l_arah_benar / self.PPR) / self.RATIO
                    putaran_r = (delta_r_arah_benar / self.PPR) / self.RATIO
                    
                    with self.lock:
                        self.dist_l = putaran_l * keliling_utama
                        self.dist_r = putaran_r * keliling_utama
                        
                except canopen.SdoCommunicationError: pass 
                time.sleep(0.01) 
        except Exception as e: print(f"[ENC FATAL] {e}")
