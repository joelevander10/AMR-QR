import threading
import time
import math
import sys
import os
import paho.mqtt.client as mqtt

# --- IMPORT MODULES ---
try: 
    from analog_digital import CK5162E_Controller, CKDA08ETH_Controller
except ImportError: 
    print("[SYSTEM] Driver analog_digital.py tidak ditemukan.")

try:
    from amr_qr_nav import QRNavigationSystem
    from trackless_module import TracklessSystem
except ImportError as e: 
    print(f"[MODULE ERROR] {e}")

try:
    import rospy
    from sensor_msgs.msg import LaserScan
    ROS_AVAILABLE = True
except ImportError:
    print("[SYSTEM] ROS/rospy tidak ditemukan. Fitur Lidar Safety dinonaktifkan.")
    ROS_AVAILABLE = False

# ==========================================
# KELAS INTEGRASI VISI (BRIDGE)
# ==========================================
class IntegratedQRSystem(QRNavigationSystem):
    def __init__(self, controller_instance, camera_id=0):
        self.controller = controller_instance 
        super().__init__(camera_id)
        
        self.deadzone_angle = 0.5  
        self.deadzone_y = 25       
        
        self.alignment_start_time = 0
        self.STABILITY_DELAY = 1.0  
        self.has_reset_sensors = False

        # Oscillation counter: hitung berapa kali sudut keluar-masuk deadzone
        # Kalau sudah >= OSCILLATION_MAX → langsung eksekusi tanpa tunggu stability
        self.OSCILLATION_MAX   = 10    # toleransi 3x bolak-balik → langsung eksekusi
        self._osc_count        = 0    # jumlah transisi masuk deadzone
        self._osc_was_aligned  = False  # status aligned di iterasi sebelumnya

    def reset_vision_state(self):
        self.is_executing = False
        self.alignment_start_time = 0
        self.has_reset_sensors = False
        self._osc_count       = 0
        self._osc_was_aligned = False

    def control_logic(self, frame, ex, ey, angle, cmd, val, qr_w_px):
        self.controller.update_qr_detection_time()
        
        if qr_w_px > 0:
            ratio = self.controller.QR_REAL_SIZE_MM / qr_w_px
            dist_x_mm = round(ex * ratio, 1)
            dist_y_mm = round(ey * ratio, 1)
        else: 
            dist_x_mm, dist_y_mm = 0.0, 0.0

        self.controller.update_vision_state(ex, ey, angle, dist_x_mm, dist_y_mm)

        if self.is_executing:
            time_elapsed = time.time() - self.execution_start_time
            if self.controller.nav_mode == "EXECUTE_MOVE":
                if self.controller.dist_traveled > 50.0 or time_elapsed > 4.0:
                    self.reset_vision_state()
                return
            elif self.controller.nav_mode == "EXECUTE_ROTATION_90":
                if time_elapsed > 6.0:  
                    self.reset_vision_state()
                return
            else: 
                return 

        if not self.has_reset_sensors:
            self.controller.reset_sensors_for_alignment()
            self.has_reset_sensors = True

        aligned_y     = abs(ey)    <= self.deadzone_y
        aligned_angle = abs(angle) <= self.deadzone_angle

        # =========================================================================
        # PRIORITAS ALIGNMENT
        # Mode yang dikendalikan control_loop → jangan diganggu dari sini
        # =========================================================================
        CONTROL_LOOP_MODES = [
            "ALIGN_LARGE_ROTATION", "POST_ROT_CREEP", "ALIGN_ROTATION_IMU",
            "EXECUTE_MOVE", "EXECUTE_ROTATION_90"
        ]

        # 1. PRIORITAS UTAMA: Y-axis (maju/mundur) agar QR aman di tengah
        if not aligned_y and self.controller.nav_mode not in CONTROL_LOOP_MODES:
            self.alignment_start_time = 0
            self.controller.set_nav_mode("ALIGN_Y")
            self.controller.set_nav_status_detail(f"ALIGN M/M: {ey}px")
            return

        # 2. PRIORITAS KEDUA: sudut ekstrem (>30°) → rotasi besar IMU-based
        if abs(angle) > 30.0 and self.controller.nav_mode not in CONTROL_LOOP_MODES:
            self.alignment_start_time = 0
            self.controller.start_large_rotation(angle)
            return

        # Guard: mode ini dikendalikan control_loop — jangan diganggu vision
        if self.controller.nav_mode in ["ALIGN_LARGE_ROTATION", "POST_ROT_CREEP", "ALIGN_ROTATION_IMU"]:
            return

        # 3. PRIORITAS KETIGA: sudut kecil (<30°) → putar halus berbasis IMU
        if not aligned_angle and self.controller.nav_mode not in CONTROL_LOOP_MODES:
            self.alignment_start_time = 0
            if self._osc_was_aligned:
                self._osc_was_aligned = False
            # Setup target IMU & masuk mode ALIGN_ROTATION_IMU (continuous)
            self.controller.start_align_rotation_imu(angle)
            self.controller.set_nav_status_detail(
                f"ALIGN SUDUT IMU: {angle:.1f}° | osc:{self._osc_count}/{self.OSCILLATION_MAX}")
            return

        # 4. Sudut masuk deadzone
        # Deteksi transisi masuk deadzone (sebelumnya tidak aligned, sekarang aligned)
        if not self._osc_was_aligned:
            self._osc_count      += 1
            self._osc_was_aligned = True

        # Jika sudah osilasi >= OSCILLATION_MAX kali → langsung eksekusi tanpa tunggu
        if self._osc_count >= self.OSCILLATION_MAX:
            self.controller.set_nav_status_detail(
                f"OSILASI {self._osc_count}x → LANGSUNG EKSEKUSI!")
            self.is_executing = True
            self.execution_start_time = time.time()
            self.last_command = f"{cmd}:{val}"
            self._osc_count = 0
            self.controller.execute_qr_command(cmd, val)
            return

        # Belum osilasi berlebihan → tunggu jeda stabilitas normal
        if self.alignment_start_time == 0:
            self.alignment_start_time = time.time()

        if (time.time() - self.alignment_start_time) < self.STABILITY_DELAY:
            self.controller.set_nav_mode("WAITING_STABLE")
            sisa_waktu = self.STABILITY_DELAY - (time.time() - self.alignment_start_time)
            self.controller.set_nav_status_detail(
                f"PAS! TUNGGU {sisa_waktu:.1f}s... | osc:{self._osc_count}/{self.OSCILLATION_MAX}")
        else:
            self.is_executing = True
            self.execution_start_time = time.time()
            self.last_command = f"{cmd}:{val}"
            self._osc_count = 0
            self.controller.execute_qr_command(cmd, val)


# ==========================================
# KELAS MONITOR LIDAR (SAFETY)
# ==========================================
class LidarSafetyMonitor:
    def __init__(self):
        self.STOP_DISTANCE = 0.6  
        self.SLOW_DISTANCE = 1.0  
        self.obstacle_stop = False
        self.obstacle_slow = False
        self.min_dist = 99.9
        
        if ROS_AVAILABLE:
            try:
                self.sub = rospy.Subscriber("/sick_safetyscanners/scan", LaserScan, self.callback)
            except Exception as e: 
                print(f"[LIDAR] Gagal subscribe ROS: {e}")

    def callback(self, data):
        try:
            ranges = data.ranges
            mid = len(ranges) // 2
            window = int(len(ranges) * 0.25)
            front_ranges = ranges[max(0, mid-window) : min(len(ranges), mid+window)]
            valid = [r for r in front_ranges if 0.05 < r < 10.0 and not math.isinf(r)]
            if not valid: 
                self.min_dist = 99.9
                self.obstacle_stop = False
                self.obstacle_slow = False
                return
            self.min_dist = min(valid)
            self.obstacle_stop = self.min_dist < self.STOP_DISTANCE
            self.obstacle_slow = self.min_dist < self.SLOW_DISTANCE
        except: pass


# ==========================================
# KELAS KONTROLER UTAMA AMR
# ==========================================
class AMRController:
    def __init__(self):
        """
        SPESIFIKASI AGV:
        - Panjang: 1050mm | Lebar: 500mm | Jarak antar roda (Track): 377mm
        - Motor: BLDC 400W 3000 RPM, Gearbox 1:30 (Analog 0-5V)
        """
        self.QR_REAL_SIZE_MM = 30.0
        self.TRACK_WIDTH_MM  = 377.0 

        # --- SETUP HARDWARE ---
        self.digital = CK5162E_Controller("192.168.2.30")
        self.analog  = CKDA08ETH_Controller("192.168.1.30")
        self.PIN_R_FWD = 1; self.PIN_R_REV = 2; self.PIN_R_BRK = 3
        self.PIN_L_FWD = 4; self.PIN_L_REV = 5; self.PIN_L_BRK = 6
        self.ANA_CH_R  = 0; self.ANA_CH_L  = 1; self.PIN_ESTOP = 0 

        # =======================================================
        # KALIBRASI MOTOR (TRIM) & KOREKSI ARAH
        # =======================================================
        self.TRIM_L        = 0.97
        self.TRIM_R        = 1.00
        self.DIR_KOREKSI   = -1   # +1 atau -1 sesuai wiring motor
        self.IMU_DIR       = 1    # +1 atau -1 sesuai orientasi IMU
        self.REVERSE_CAMERA_X = False

        # --- KONFIGURASI VOLTASE & KECEPATAN ---
        self.V_SCALE           = 20.0
        self.LIMIT_SPD_LINEAR  = 1.5 * self.V_SCALE
        self.LIMIT_SPD_ROTATION= 1.0 * self.V_SCALE
        self.V_CRUISE          = 1.2 * self.V_SCALE
        self.V_APPROACH        = 0.8 * self.V_SCALE
        self.V_STALL           = 0.4 * self.V_SCALE
        self.V_ALIGN           = 0.3 * self.V_SCALE

        # --- KONFIGURASI S-CURVE & JARAK ---
        self.STOP_SEARCH_DIST   = 200.0
        self.TARGET_X_DIST_CM   = 200.0
        self.ACCEL_DIST_CM      = 15.0
        self.CORRECTION_DIST_CM = 190.0
        self.scurve_offset_cm   = 0.0

        # =======================================================
        # PARAMETER GAIN PID
        # =======================================================
        self.KP_IMU = 0.20
        self.KI_IMU = 0.02
        self.KD_IMU = 1.00
        self.KP_ENC = 0.08
        self.integral_err_imu = 0.0

        # --- Pulse align (untuk ALIGN_Y saja) ---
        self.align_pulse_timer  = 0.0
        self.align_pulse_state  = "MOVE"
        self.ALIGN_MOVE_DURATION= 0.3
        self.ALIGN_STOP_DURATION= 1.0

        # --- ALIGN_ROTATION_IMU (continuous, no pulse) ---
        # AMR putar pelan sekali, target sudut diukur dari IMU.
        # Saat masuk mode ini, IMU saat itu disimpan sebagai referensi (0°),
        # lalu AMR putar sampai delta_yaw ≈ -error_angle (negatif karena lawan arah error).
        self.V_ALIGN_ROT_SLOW    = 0.18 * self.V_SCALE  # super pelan, anti osilasi
        self.ALIGN_ROT_DEADZONE  = 0.3   # derajat — toleransi target tercapai
        self._align_rot_start_yaw = 0.0  # IMU yaw saat mulai mode ini
        self._align_rot_target    = 0.0  # target delta_yaw yang harus ditempuh

        # --- State mesin ---
        self.connected       = False
        self.estop_active    = False
        self.mode            = "MANUAL"
        self.nav_mode        = "IDLE"
        self.nav_status_detail = "STANDBY"
        self.vision_error    = {'x': 0, 'y': 0, 'angle': 0.0, 'x_mm': 0.0, 'y_mm': 0.0}
        self.last_qr_seen_time = 0

        self.current_cmd         = "NONE"
        self.current_val         = "0"
        self.current_direction   = None
        self.brake_released_manual = False

        self.dist_traveled    = 0.0
        self.start_dist_l     = 0.0
        self.start_dist_r     = 0.0
        self.start_yaw        = 0.0
        self.target_yaw_abs   = 0.0
        self.target_yaw       = 0.0
        self.target_large_angle = 0.0

        self.prev_err_imu   = 0.0
        self.jarak_asli_kiri  = 0.0
        self.jarak_asli_kanan = 0.0

        # =======================================================
        # PARAMETER ROTASI BESAR (ALIGN_LARGE_ROTATION)
        # =======================================================
        # Rotasi dilakukan ~85° (sedikit kurang dari 90°) berbasis IMU,
        # lalu berhenti. Setelah itu cek QR:
        #   - QR terbaca penuh → IDLE (lanjut alignment normal)
        #   - QR belum terbaca → langsung IDLE, vision akan handle
        self.LARGE_ROT_TARGET_DEG = 85.0  # Derajat IMU yang ditempuh sebelum berhenti
        self._pre_rot_x_offset    = 0.0   # x_mm QR sebelum rotasi besar (+ = kanan, - = kiri)
        self._rot_direction        = 0     # +1 = belok kanan, -1 = belok kiri
        self.V_POST_ROT            = 0.3 * self.V_SCALE  # 0.3V kecepatan creep setelah rotasi
        self._post_rot_creep_dir   = 1     # +1 maju, -1 mundur (ditentukan saat rotasi selesai)

        # --- GOAL state ---
        # Ketika QR goal terdeteksi, AMR akan lurus dulu ke QR,
        # lalu masuk GOAL_REACHED dan berhenti sampai START_AUTO ditekan lagi.
        self.goal_pending   = False  # True = QR goal terdeteksi, tunggu alignment selesai
        self.ignore_goal    = False  # True = baru start dari GOAL, abaikan cmd GOAL → jadikan MAJU
        self.return_home    = False  # True = mode kembali ke homepost

        # --- CSV log ---
        self.csv_file      = "amr_diagnostic_log.csv"
        self.last_log_time = 0
        try:
            with open(self.csv_file, 'w') as f:
                f.write("Time,Status,Dist_X,Offset_Awal_Y,Target_Yaw_SCurve,"
                        "IMU_Yaw,Enc_L,Enc_R,Enc_Yaw,PID_Corr,Int_Err\n")
        except: pass

        self.lidar = LidarSafetyMonitor()
        try:
            self.trackless = TracklessSystem(
                port="/dev/ttyUSB0", baudrate=9600, can_channel="/dev/ttyACM0")
        except:
            self.trackless = None

        self.init_mqtt("192.168.3.100")

        self.qr_sys = None
        threading.Thread(target=self.start_camera_thread, daemon=True).start()

        self.connect_hardware()
        threading.Thread(target=self.control_loop, daemon=True).start()

    # ----------------------------------------------------------
    # THREAD KAMERA
    # ----------------------------------------------------------
    def start_camera_thread(self):
        print("[VISION] Menghidupkan kamera di background...")
        time.sleep(2)
        while True:
            try:
                if self.qr_sys is None:
                    self.qr_sys = IntegratedQRSystem(self, camera_id=0)
                if hasattr(self.qr_sys, 'cap') and self.qr_sys.cap.isOpened():
                    self.qr_sys.process_stream()
                    if hasattr(self.qr_sys, 'cap'): self.qr_sys.cap.release()
                    self.qr_sys = None
                else:
                    self.qr_sys = None
            except:
                self.qr_sys = None
            time.sleep(3)

    # ----------------------------------------------------------
    # HELPERS
    # ----------------------------------------------------------
    def reset_sensors_for_alignment(self):
        if self.trackless:
            self.start_dist_l = self.trackless.dist_l
            self.start_dist_r = self.trackless.dist_r
            self.start_yaw    = self.trackless.get_yaw()
        self.dist_traveled = 0.0

    def reset_vision_system(self):
        if hasattr(self, 'qr_sys') and self.qr_sys is not None:
            self.qr_sys.reset_vision_state()

    def update_qr_detection_time(self):
        self.last_qr_seen_time = time.time()

    def update_vision_state(self, ex, ey, angle, x_mm, y_mm):
        self.vision_error = {
            'x': ex, 'y': ey, 'angle': angle,
            'x_mm': x_mm, 'y_mm': y_mm
        }

    def set_nav_mode(self, mode):
        if not self.estop_active: self.nav_mode = mode

    def set_nav_status_detail(self, msg):
        self.nav_status_detail = msg

    def connect_hardware(self):
        try:
            if self.digital.connect() and self.analog.connect():
                self.connected = True
                print("[INFO] Hardware Kontrol Terhubung")
        except: pass

    def release_brake(self):
        self.brake_released_manual = True
        try:
            self.digital.set_output(self.PIN_L_BRK, False)
            self.digital.set_output(self.PIN_R_BRK, False)
        except: pass

    # ----------------------------------------------------------
    # MOTOR CONTROL
    # ----------------------------------------------------------
    def set_motor(self, left_spd, right_spd):
        if not self.connected or self.estop_active:
            self.stop_motor(); return
        self.brake_released_manual = False
        l_s = left_spd  * self.TRIM_L
        r_s = right_spd * self.TRIM_R
        is_pure_rotation = (left_spd * right_spd < 0)
        max_spd = self.LIMIT_SPD_LINEAR if not is_pure_rotation else self.LIMIT_SPD_ROTATION

        if self.lidar.obstacle_stop and (left_spd > 0 or right_spd > 0):
            self.stop_motor(); return
        if self.lidar.obstacle_slow and (left_spd > 0 or right_spd > 0):
            l_s *= 0.5; r_s *= 0.5

        def stall_limit(spd):
            if abs(spd) < 0.1: return 0.0
            if abs(spd) < self.V_STALL: return math.copysign(self.V_STALL, spd)
            return spd

        l_s = stall_limit(l_s); r_s = stall_limit(r_s)
        l_s = max(-max_spd, min(max_spd, l_s))
        r_s = max(-max_spd, min(max_spd, r_s))

        self.digital.set_output(self.PIN_L_FWD, l_s > 0)
        self.digital.set_output(self.PIN_L_REV, l_s < 0)
        self.digital.set_output(self.PIN_R_FWD, r_s < 0)   # wiring terbalik
        self.digital.set_output(self.PIN_R_REV, r_s > 0)
        self.digital.set_output(self.PIN_L_BRK, False)
        self.digital.set_output(self.PIN_R_BRK, False)
        self.analog.set_analog_output(self.ANA_CH_L, abs(l_s) / 20.0)
        self.analog.set_analog_output(self.ANA_CH_R, abs(r_s) / 20.0)

    def stop_motor(self):
        try:
            self.analog.set_analog_output(self.ANA_CH_L, 0)
            self.analog.set_analog_output(self.ANA_CH_R, 0)
            if not self.brake_released_manual:
                self.digital.set_output(self.PIN_L_BRK, True)
                self.digital.set_output(self.PIN_R_BRK, True)
        except: pass

    def shutdown_system(self):
        try:
            self.stop_motor()
            self.digital.set_output(self.PIN_L_BRK, False)
            self.digital.set_output(self.PIN_R_BRK, False)
        except: pass

    def execute_manual_control(self):
        if not self.current_direction: self.stop_motor(); return
        base = 20.0; d = self.current_direction.upper()
        if   d == 'FORWARD' : self.set_motor( base,  base)
        elif d == 'BACKWARD': self.set_motor(-base, -base)
        elif d == 'LEFT'    : self.set_motor(-base,  base)
        elif d == 'RIGHT'   : self.set_motor( base, -base)
        else: self.stop_motor()

    # ----------------------------------------------------------
    # START LARGE ROTATION
    # ----------------------------------------------------------
    def start_large_rotation(self, target_angle):
        """
        Mulai rotasi besar berbasis IMU.
        Target aktual = LARGE_ROT_TARGET_DEG (85°), bukan 90°.
        Simpan x_offset QR saat ini dan arah belok — dipakai setelah rotasi
        untuk menentukan apakah AMR harus maju atau mundur cari QR.
        """
        self.nav_mode           = "ALIGN_LARGE_ROTATION"
        self.target_large_angle = target_angle   # + = kanan, - = kiri
        self._rot_direction     = 1 if target_angle > 0 else -1
        # Simpan x_offset QR sekarang (sebelum rotasi), dengan reverse jika perlu
        x_mm = self.vision_error.get('x_mm', 0.0)
        self._pre_rot_x_offset  = -x_mm if self.REVERSE_CAMERA_X else x_mm
        self.integral_err_imu   = 0.0
        self.prev_err_imu       = 0.0
        if self.trackless:
            self.start_yaw = self.trackless.get_yaw()
        self.set_nav_status_detail(f"LARGE ROT: {target_angle:.1f}° | x_off={self._pre_rot_x_offset:.1f}mm")

    # ----------------------------------------------------------
    # SETUP ALIGN_ROTATION_IMU
    # ----------------------------------------------------------
    def start_align_rotation_imu(self, error_angle):
        """
        Setup koreksi sudut continuous berbasis IMU.
        - error_angle: sudut error QR (derajat). Tanda menentukan arah putar.
        - Simpan IMU yaw saat ini sebagai referensi 0°.
        - Target = error_angle (AMR harus putar sebesar itu agar QR lurus).
          (Jika kalibrasi terbalik, ubah tanda di self._align_rot_target = ...)
        """
        self.nav_mode = "ALIGN_ROTATION_IMU"
        if self.trackless:
            self._align_rot_start_yaw = self.trackless.get_yaw()
        else:
            self._align_rot_start_yaw = 0.0
        # AMR perlu putar SEBESAR error_angle agar sudut QR jadi 0
        # Jika ternyata arah terbalik di lapangan, ganti tanda berikut:
        self._align_rot_target = error_angle

    # ----------------------------------------------------------
    # SETELAH ROTASI BESAR SELESAI → creep cari QR atau langsung IDLE
    # ----------------------------------------------------------
    def _enter_reacquire(self, qr_fully_visible):
        """
        Setelah rotasi besar selesai:
        - Jika QR langsung terbaca → IDLE (vision handle normal)
        - Jika QR belum terbaca → masuk POST_ROT_CREEP:
            Arah gerak ditentukan dari x_offset sebelum belok × arah belok:
              sign(x_offset) == sign(rot_direction) → MAJU
              sign(x_offset) != sign(rot_direction) → MUNDUR
            Ini karena:
              Belok kanan + QR di kanan → QR sekarang ada di depan → maju
              Belok kanan + QR di kiri  → QR sekarang ada di belakang → mundur
              Belok kiri  + QR di kanan → QR sekarang ada di belakang → mundur
              Belok kiri  + QR di kiri  → QR sekarang ada di depan → maju
        """
        self.stop_motor()

        if qr_fully_visible:
            self.nav_mode = "IDLE"
            self.reset_vision_system()
            self.set_nav_status_detail("ROTASI SELESAI, QR OK")
            print("[AMR] Rotasi selesai. QR langsung terbaca → IDLE")
        else:
            # Hitung arah creep
            x_sign   = 1 if self._pre_rot_x_offset >= 0 else -1
            rot_sign = self._rot_direction  # +1 kanan, -1 kiri
            # Sama tanda → maju (+1), beda tanda → mundur (-1)
            creep_dir = x_sign * rot_sign
            label = "MAJU" if creep_dir > 0 else "MUNDUR"

            self._post_rot_creep_dir = creep_dir
            self.nav_mode = "POST_ROT_CREEP"
            self.set_nav_status_detail(
                f"ROTASI SELESAI → {label} cari QR "
                f"(x_off={self._pre_rot_x_offset:.0f}mm, belok={'KANAN' if rot_sign>0 else 'KIRI'})"
            )
            print(f"[AMR] Rotasi selesai. QR belum terbaca → Creep {label}")

    # ----------------------------------------------------------
    # EXECUTE QR COMMAND
    # ----------------------------------------------------------
    def execute_qr_command(self, cmd, val):
        # -------------------------------------------------------
        # MODE RETURN HOME: semua QR → MAJU, kecuali HOMEPOST → berhenti
        # -------------------------------------------------------
        if self.return_home:
            if "HOMEPOST" in cmd.upper() or "HOME" in cmd.upper():
                # Sampai homepost → berhenti, reset mode
                self.stop_motor()
                self.return_home  = False
                self.mode         = "MANUAL"
                self.nav_mode     = "GOAL_REACHED"
                self.set_nav_status_detail("🏠 HOMEPOST TERCAPAI! Tekan START AUTO untuk lanjut.")
                print("[AMR] HOMEPOST reached — berhenti.")
                return
            else:
                # QR lain (termasuk GOAL) → paksa jadi MAJU
                print(f"[AMR] RETURN HOME: QR '{cmd}' diabaikan → MAJU")
                cmd = "MAJU"
                # jatuh ke handler MAJU di bawah

        # QR GOAL: lurus dulu (alignment sudah selesai saat ini dipanggil),
        # lalu langsung masuk GOAL_REACHED — berhenti tunggu START_AUTO.
        # KECUALI: ignore_goal aktif (baru start dari GOAL) → eksekusi sebagai MAJU
        elif "GOAL" in cmd.upper() or "STOP" in cmd.upper():
            if self.ignore_goal:
                # Abaikan GOAL, jadikan MAJU biasa, lalu clear flag
                self.ignore_goal = False
                print("[AMR] GOAL diabaikan (baru start) → eksekusi sebagai MAJU")
                cmd = "MAJU"
                # jatuh ke handler MAJU di bawah
            else:
                self.stop_motor()
                self.mode         = "MANUAL"
                self.nav_mode     = "GOAL_REACHED"
                self.goal_pending = False
                self.set_nav_status_detail("🏁 GOAL TERCAPAI! Tekan START AUTO untuk lanjut.")
                print("[AMR] GOAL REACHED — menunggu START AUTO dari operator.")
                return

        if "PUTAR" in cmd.upper() and self.trackless:
            initial_yaw   = self.trackless.get_yaw()
            move_angle    = 90.0 if "KANAN" in cmd.upper() else -90.0
            self.target_yaw_abs = (initial_yaw + move_angle + 180) % 360 - 180
            self.target_yaw     = self.target_yaw_abs
            self.nav_mode       = "EXECUTE_ROTATION_90"

        elif "MAJU" in cmd.upper() or "MUNDUR" in cmd.upper():
            if self.trackless:
                time.sleep(0.05)
                self.start_dist_l = self.trackless.dist_l
                self.start_dist_r = self.trackless.dist_r
                self.start_yaw    = self.trackless.get_yaw()

                offset_x_mm = self.vision_error.get('x_mm', 0.0)
                if self.REVERSE_CAMERA_X: offset_x_mm = -offset_x_mm
                self.scurve_offset_cm = (-offset_x_mm / 10.0) * 0.80

                self.target_yaw = 0.0
                try:
                    with open(self.csv_file, 'w') as f:
                        f.write("Time,Status,Dist_X,Offset_Awal_Y,Target_Yaw_SCurve,"
                                "IMU_Yaw,Enc_L,Enc_R,Enc_Yaw,PID_Corr,Int_Err\n")
                        f.write(f"{time.time():.3f},START_{cmd},0.00,"
                                f"{self.scurve_offset_cm:.2f},0.00,0.00,0.00,0.00,0.00,0.00,0.00\n")
                except: pass

            self.dist_traveled    = 0.0
            self.prev_err_imu     = 0.0
            self.integral_err_imu = 0.0
            self.nav_mode         = "EXECUTE_MOVE"

    # ----------------------------------------------------------
    # CONTROL LOOP (thread utama)
    # ----------------------------------------------------------
    def control_loop(self):
        while True:
            if self.connected:
                estop = not self.digital.read_input(self.PIN_ESTOP)
                if estop != self.estop_active:
                    self.estop_active = estop
                    if estop: self.mode = "MANUAL"
            if self.estop_active:
                self.stop_motor()
            elif self.mode == "AUTO":
                self.handle_auto_navigation()
            time.sleep(0.02)

    # ----------------------------------------------------------
    # HANDLE AUTO NAVIGATION
    # ----------------------------------------------------------
    def handle_auto_navigation(self):
        err_a    = self.vision_error['angle']
        err_y    = self.vision_error['y']
        qr_sight = (time.time() - self.last_qr_seen_time < 0.5)

        abs_yaw = rel_yaw = 0.0
        if self.trackless:
            abs_yaw       = self.trackless.get_yaw()
            rel_yaw       = (abs_yaw - self.start_yaw + 180) % 360 - 180
            rel_dist_l    = self.trackless.dist_l - self.start_dist_l
            rel_dist_r    = self.trackless.dist_r - self.start_dist_r
            self.jarak_asli_kiri  = abs(rel_dist_l)
            self.jarak_asli_kanan = abs(rel_dist_r)
            self.dist_traveled    = (self.jarak_asli_kiri + self.jarak_asli_kanan) / 2.0

        # ── WAITING_STABLE ────────────────────────────────────────
        if self.nav_mode == "WAITING_STABLE":
            self.stop_motor()
            return

        # ── GOAL_REACHED ──────────────────────────────────────────
        # AMR sudah sampai tujuan. Berhenti total sampai operator
        # menekan START AUTO lagi (via MQTT atau tombol UI).
        if self.nav_mode == "GOAL_REACHED":
            self.stop_motor()
            self.set_nav_status_detail("🏁 GOAL TERCAPAI! Tekan START AUTO untuk lanjut.")
            return

        # ── ALIGN_LARGE_ROTATION ──────────────────────────────────
        # AMR berputar ~85° (LARGE_ROT_TARGET_DEG) berbasis IMU,
        # dengan kecepatan proporsional agar tidak overshoot.
        # Selesai → _enter_reacquire() menentukan langkah berikutnya.
        if self.nav_mode == "ALIGN_LARGE_ROTATION":
            degrees_rotated = abs(rel_yaw)
            remaining_deg   = self.LARGE_ROT_TARGET_DEG - degrees_rotated

            if remaining_deg <= 0.0:
                # Sudah mencapai (atau melewati) target → berhenti
                self._enter_reacquire(qr_sight)
            else:
                # Kecepatan proporsional: makin dekat target makin pelan
                if   remaining_deg > 30.0: rot_speed = 0.50 * self.V_SCALE
                elif remaining_deg > 15.0: rot_speed = 0.35 * self.V_SCALE
                else:                      rot_speed = 0.22 * self.V_SCALE

                out = math.copysign(rot_speed, self.target_large_angle)
                self.set_motor(out, -out)
                self.set_nav_status_detail(
                    f"LARGE ROT: sisa {remaining_deg:.1f}° | {rot_speed:.1f}V"
                )
            return

        # ── POST_ROT_CREEP ────────────────────────────────────────
        # AMR bergerak pelan (0.3V) setelah rotasi besar sampai QR terbaca.
        if self.nav_mode == "POST_ROT_CREEP":
            if qr_sight:
                # QR terbaca → berhenti, kembali ke vision normal
                self.stop_motor()
                self.nav_mode = "IDLE"
                self.reset_vision_system()
                self.set_nav_status_detail("QR TERBACA SETELAH ROTASI → IDLE")
                print("[AMR] Post-rot creep: QR terbaca → IDLE")
                return

            if self.lidar.obstacle_stop:
                self.stop_motor(); return

            spd = self.V_POST_ROT * self._post_rot_creep_dir
            self.set_motor(spd, spd)
            label = "MAJU" if self._post_rot_creep_dir > 0 else "MUNDUR"
            self.set_nav_status_detail(f"POST ROT CREEP: {label} 0.3V | cari QR...")
            return

        # ── ALIGN_ROTATION_IMU (continuous IMU-based, anti osilasi) ──
        # AMR putar pelan terus tanpa berhenti, sampai delta_yaw ≈ target.
        # Tidak berhenti-jalan agar lebih cepat dan halus.
        if self.nav_mode == "ALIGN_ROTATION_IMU":
            if not self.trackless:
                # Tanpa IMU, fallback ke pulse mode
                self.nav_mode = "ALIGN_ROTATION"
                return

            # Hitung delta_yaw sejak masuk mode ini
            current_yaw = self.trackless.get_yaw()
            delta_yaw   = (current_yaw - self._align_rot_start_yaw + 180) % 360 - 180

            # Sisa sudut = target - delta sekarang
            remaining = self._align_rot_target - delta_yaw

            if abs(remaining) <= self.ALIGN_ROT_DEADZONE:
                # Target tercapai → berhenti, biarkan vision evaluasi ulang
                self.stop_motor()
                self.nav_mode = "IDLE"
                self.reset_vision_system()
                self.set_nav_status_detail(
                    f"ALIGN IMU SELESAI: Δ={delta_yaw:.2f}° / target={self._align_rot_target:.2f}°"
                )
                return

            # Kecepatan adaptif: makin dekat target makin pelan (anti overshoot)
            if   abs(remaining) > 5.0: rot_speed = self.V_ALIGN_ROT_SLOW * 1.4  # ~0.25V
            elif abs(remaining) > 2.0: rot_speed = self.V_ALIGN_ROT_SLOW        # ~0.18V
            else:                      rot_speed = self.V_ALIGN_ROT_SLOW * 0.7  # ~0.13V (super pelan)

            out = math.copysign(rot_speed, remaining)
            self.set_motor(out, -out)
            self.set_nav_status_detail(
                f"ALIGN IMU: Δ={delta_yaw:.2f}° / tgt={self._align_rot_target:.2f}° | sisa={remaining:.2f}°"
            )
            return

        # ── ALIGN_Y (pulse halus, tetap seperti sebelumnya) ───────
        if self.nav_mode in ["ALIGN_ROTATION", "ALIGN_Y"]:
            current_t = time.time()
            if self.align_pulse_timer == 0.0:
                self.align_pulse_timer = current_t
                self.align_pulse_state = "MOVE"
            elapsed = current_t - self.align_pulse_timer

            if self.align_pulse_state == "MOVE":
                if elapsed > self.ALIGN_MOVE_DURATION:
                    self.align_pulse_state = "STOP"
                    self.align_pulse_timer = current_t
                    self.set_motor(0, 0)
                else:
                    if self.nav_mode == "ALIGN_ROTATION":
                        out = math.copysign(self.V_ALIGN, err_a)
                        self.set_motor(out, -out)
                    elif self.nav_mode == "ALIGN_Y":
                        out = math.copysign(self.V_ALIGN, err_y)
                        self.set_motor(out, out)
                return

            elif self.align_pulse_state == "STOP":
                self.set_motor(0, 0)
                if elapsed > self.ALIGN_STOP_DURATION:
                    self.align_pulse_state = "MOVE"
                    self.align_pulse_timer = current_t
                return
        else:
            self.align_pulse_timer = 0.0

        # ── EXECUTE_ROTATION_90 ───────────────────────────────────
        if self.nav_mode == "EXECUTE_ROTATION_90":
            diff_imu = (self.target_yaw_abs - abs_yaw + 180) % 360 - 180
            if abs(diff_imu) < 1.0 or (
                    qr_sight and abs(diff_imu) < 5.0 and abs(err_a) < 2.0):
                self.stop_motor()
                time.sleep(1.0)
                self.reset_vision_system()
                self.nav_mode = "EXECUTE_MOVE"
            else:
                out = max(-25.0, min(25.0, diff_imu * 0.8))
                if abs(diff_imu) < 10: out = max(-10.0, min(10.0, out))
                self.set_motor(-out, out)

        # ── EXECUTE_MOVE (S-Curve PID) ────────────────────────────
        elif self.nav_mode == "EXECUTE_MOVE":
            # ---------------------------------------------------
            # ZONA CREEP (170cm - 190cm):
            #   AMR jalan sangat pelan (0.5V) sambil nunggu QR terbaca.
            #   Begitu QR terbaca → berhenti & reset vision.
            #   Kalau sampai 190cm QR masih belum kebaca → berhenti paksa.
            # ---------------------------------------------------
            CREEP_START_CM = 190.0
            CREEP_SPEED    = 0.5 * self.V_SCALE   # 0.5V
            HARD_STOP_CM   = 200.0

            if self.dist_traveled >= HARD_STOP_CM:
                # Berhenti paksa — QR tidak ketemu sampai batas maksimal
                self.stop_motor()
                self.set_nav_status_detail(
                    f"BERHENTI PAKSA (QR tidak terbaca) | Dist: {self.dist_traveled:.1f}cm")
                self.reset_vision_system()
                return

            if self.dist_traveled >= CREEP_START_CM:
                # Zona creep: cek QR dulu
                if qr_sight:
                    # QR terbaca → berhenti
                    self.stop_motor()
                    self.set_nav_status_detail(
                        f"QR TERBACA, BERHENTI | Dist: {self.dist_traveled:.1f}cm")
                    self.reset_vision_system()
                    return
                # QR belum terbaca → lanjut creep pelan
                target_speed = CREEP_SPEED
                if self.lidar.obstacle_stop:
                    self.stop_motor(); return
                self.set_motor(
                    target_speed - (0 * self.DIR_KOREKSI),
                    target_speed + (0 * self.DIR_KOREKSI)
                )
                self.set_nav_status_detail(
                    f"CREEP SCAN QR | Dist: {self.dist_traveled:.1f}cm")
                return

            # Profil kecepatan normal (0 - 170cm)
            target_speed = self.V_CRUISE
            if self.dist_traveled < self.ACCEL_DIST_CM:
                ramp = self.dist_traveled / self.ACCEL_DIST_CM
                target_speed = self.V_STALL + (self.V_CRUISE - self.V_STALL) * ramp
            elif self.dist_traveled >= 150.0:
                target_speed = 0.3 * self.V_SCALE  # 0.3V mulai 150cm

            if self.lidar.obstacle_stop:
                self.stop_motor(); return

            correction = 0.0
            if self.trackless:
                active_dist = max(10.0, min(self.TARGET_X_DIST_CM, self.CORRECTION_DIST_CM))
                x = max(0.0, min(self.dist_traveled, self.TARGET_X_DIST_CM))

                if x < active_dist:
                    ratio       = x / active_dist
                    y_prime     = (6.0 * self.scurve_offset_cm / active_dist) * (ratio - ratio**2)
                    dyn_yaw     = math.degrees(math.atan(y_prime))
                    self.target_yaw = max(-15.0, min(15.0, dyn_yaw))
                else:
                    self.target_yaw = 0.0

                err_imu = ((rel_yaw - self.target_yaw + 180) % 360 - 180) * self.IMU_DIR

                self.integral_err_imu += err_imu
                self.integral_err_imu  = max(-100.0, min(100.0, self.integral_err_imu))

                yaw_enc_rad = ((self.jarak_asli_kanan - self.jarak_asli_kiri)
                               / (self.TRACK_WIDTH_MM / 10.0))
                yaw_enc_deg = math.degrees(yaw_enc_rad)
                err_enc     = yaw_enc_deg - self.target_yaw

                d_err_imu       = err_imu - self.prev_err_imu
                self.prev_err_imu = err_imu

                correction = (err_imu  * self.KP_IMU
                            + self.integral_err_imu * self.KI_IMU
                            + d_err_imu * self.KD_IMU
                            + err_enc   * self.KP_ENC)

                max_corr   = target_speed * 0.25
                correction = max(-max_corr, min(max_corr, correction))

                self.set_nav_status_detail(
                    f"S-CRV | Ty:{self.target_yaw:.1f}° | Y:{rel_yaw:.1f}° | C:{correction:.1f}")

                if time.time() - self.last_log_time > 0.1:
                    try:
                        with open(self.csv_file, 'a') as f:
                            f.write(f"{time.time():.3f},MOVING,"
                                    f"{self.dist_traveled:.2f},{self.scurve_offset_cm:.2f},"
                                    f"{self.target_yaw:.2f},{rel_yaw:.2f},"
                                    f"{self.jarak_asli_kiri:.2f},{self.jarak_asli_kanan:.2f},"
                                    f"{yaw_enc_deg:.2f},{correction:.2f},"
                                    f"{self.integral_err_imu:.2f}\n")
                        self.last_log_time = time.time()
                    except: pass

            self.set_motor(
                target_speed - (correction * self.DIR_KOREKSI),
                target_speed + (correction * self.DIR_KOREKSI)
            )

    # ----------------------------------------------------------
    # MQTT
    # ----------------------------------------------------------
    def init_mqtt(self, broker):
        try:
            self.client = mqtt.Client(client_id="amr_brain")
            self.client.on_message = self.on_mqtt_message
            self.client.connect_async(broker, 1883, 60)
            self.client.loop_start()
            self.client.subscribe("amr/command")
        except: pass

    def on_mqtt_message(self, client, userdata, msg):
        p = msg.payload.decode().upper()
        if p == "START_AUTO":
            self.mode = "AUTO"
            # Kalau sebelumnya GOAL_REACHED, reset agar bisa jalan lagi
            if self.nav_mode == "GOAL_REACHED":
                self.nav_mode    = "IDLE"
                self.ignore_goal = True   # QR goal berikutnya → jadikan MAJU
                self.reset_vision_system()
                self.set_nav_status_detail("LANJUT DARI GOAL...")
                print("[AMR] START AUTO diterima. Lanjut dari GOAL — GOAL berikutnya diabaikan.")
        elif p == "RETURN_HOME":
            self.return_home = True
            self.mode        = "AUTO"
            self.nav_mode    = "IDLE"
            self.reset_vision_system()
            self.set_nav_status_detail("🏠 RETURN HOME aktif — cari HOMEPOST...")
            print("[AMR] RETURN HOME diaktifkan.")
        elif p in ["STOP_AUTO", "STOP"]:
            self.return_home = False
            self.mode        = "MANUAL"
            self.stop_motor()


if __name__ == "__main__":
    c = AMRController()
    c.mode = "AUTO"
    try:
        while True: time.sleep(1)
    except KeyboardInterrupt:
        c.shutdown_system()
        sys.exit(0)
