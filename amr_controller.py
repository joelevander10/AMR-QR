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
        self.deadzone_y     = 25       
        
        self.alignment_start_time = 0
        self.STABILITY_DELAY      = 1.0  
        self.has_reset_sensors    = False

        # Oscillation counter
        self.OSCILLATION_MAX  = 10
        self._osc_count       = 0
        self._osc_was_aligned = False

    def reset_vision_state(self):
        self.is_executing         = False
        self.alignment_start_time = 0
        self.has_reset_sensors    = False
        self._osc_count           = 0
        self._osc_was_aligned     = False

    def control_logic(self, frame, ex, ey, angle, cmd, val, qr_w_px):
        self.controller.update_qr_detection_time()
        
        if qr_w_px > 0:
            ratio     = self.controller.QR_REAL_SIZE_MM / qr_w_px
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

        # Cek apakah sudut sudah mendekati ±90° (dalam deadzone)
        aligned_to_90 = abs(abs(angle) - 90.0) <= self.deadzone_angle

        CONTROL_LOOP_MODES = [
            "ALIGN_LARGE_ROTATION", "POST_ROT_CREEP",
            "EXECUTE_MOVE", "EXECUTE_ROTATION_90",
            "ALIGN_TO_90"
        ]

        # 1. PRIORITAS UTAMA: Y-axis
        if not aligned_y and self.controller.nav_mode not in CONTROL_LOOP_MODES:
            self.alignment_start_time = 0
            self.controller.set_nav_mode("ALIGN_Y")
            self.controller.set_nav_status_detail(f"ALIGN M/M: {ey}px")
            return

        # 2. PRIORITAS KEDUA: sudut ekstrem (>30°)
        #    Harus align ke ±90° dulu sebelum start_large_rotation
        if abs(angle) > 30.0 and self.controller.nav_mode not in CONTROL_LOOP_MODES:
            self.alignment_start_time = 0

            if not aligned_to_90:
                # Belum mendekati ±90°, align dulu
                target_90 = 90.0 if angle > 0 else -90.0
                err_to_90 = angle - target_90
                self.controller.set_nav_mode("ALIGN_TO_90")
                self.controller.set_nav_status_detail(
                    f"ALIGN KE {'90°' if angle > 0 else '-90°'}: "
                    f"sudut={angle:.1f}° | err={err_to_90:.1f}°")
            else:
                # Sudah di ±90°, eksekusi rotasi besar
                self.controller.start_large_rotation(angle)
            return

        # Guard
        if self.controller.nav_mode in ["ALIGN_LARGE_ROTATION", "POST_ROT_CREEP", "ALIGN_TO_90"]:
            return

        # 3. PRIORITAS KETIGA: sudut kecil (<30°) → pulse halus
        if not aligned_angle and self.controller.nav_mode not in CONTROL_LOOP_MODES:
            self.alignment_start_time = 0
            if self._osc_was_aligned:
                self._osc_was_aligned = False
            self.controller.set_nav_mode("ALIGN_ROTATION")
            self.controller.set_nav_status_detail(
                f"ALIGN SUDUT: {angle:.1f}° | osc:{self._osc_count}/{self.OSCILLATION_MAX}")
            return

        # 4. Sudut masuk deadzone
        if not self._osc_was_aligned:
            self._osc_count      += 1
            self._osc_was_aligned = True

        # Osilasi berlebihan → langsung eksekusi
        if self._osc_count >= self.OSCILLATION_MAX:
            self.controller.set_nav_status_detail(
                f"OSILASI {self._osc_count}x → LANGSUNG EKSEKUSI!")
            self.is_executing         = True
            self.execution_start_time = time.time()
            self.last_command         = f"{cmd}:{val}"
            self._osc_count           = 0
            self.controller.execute_qr_command(cmd, val)
            return

        # Tunggu stabilitas
        if self.alignment_start_time == 0:
            self.alignment_start_time = time.time()

        if (time.time() - self.alignment_start_time) < self.STABILITY_DELAY:
            self.controller.set_nav_mode("WAITING_STABLE")
            sisa_waktu = self.STABILITY_DELAY - (time.time() - self.alignment_start_time)
            self.controller.set_nav_status_detail(
                f"PAS! TUNGGU {sisa_waktu:.1f}s... | osc:{self._osc_count}/{self.OSCILLATION_MAX}")
        else:
            self.is_executing         = True
            self.execution_start_time = time.time()
            self.last_command         = f"{cmd}:{val}"
            self._osc_count           = 0
            self.controller.execute_qr_command(cmd, val)


# ==========================================
# KELAS MONITOR LIDAR (SAFETY)
# Zone pantau: -45° s/d +45° (depan AMR)
# STOP_DISTANCE : 0.6 m → motor berhenti penuh
# SLOW_DISTANCE : 1.0 m → motor melambat 50%
# ==========================================
class LidarSafetyMonitor:
    def __init__(self):
        self.STOP_DISTANCE = 0.6
        self.SLOW_DISTANCE = 1.0

        self.ANGLE_MIN_DEG = 100.0
        self.ANGLE_MAX_DEG = 140.0

        self.RANGE_MIN_VALID = 0.15
        self.RANGE_MAX_VALID = 10.0

        self.obstacle_stop = False
        self.obstacle_slow = False
        self.min_dist      = 99.9
        self.active_ranges = 0

        if ROS_AVAILABLE:
            try:
                self.sub = rospy.Subscriber(
                    "/sick_safetyscanners/scan", LaserScan, self.callback)
                print(f"[LIDAR] Subscribe /sick_safetyscanners/scan "
                      f"— zona {self.ANGLE_MIN_DEG}°~{self.ANGLE_MAX_DEG}°"
                      f" | filter {self.RANGE_MIN_VALID}–{self.RANGE_MAX_VALID}m")
            except Exception as e:
                print(f"[LIDAR] Gagal subscribe ROS: {e}")
        else:
            print("[LIDAR] ROS tidak tersedia, safety lidar dinonaktifkan.")

    def callback(self, data):
        try:
            angle_min = data.angle_min
            angle_inc = data.angle_increment
            n         = len(data.ranges)

            idx_min = int((math.radians(self.ANGLE_MIN_DEG) - angle_min) / angle_inc)
            idx_max = int((math.radians(self.ANGLE_MAX_DEG) - angle_min) / angle_inc)

            idx_min = max(0, min(n - 1, idx_min))
            idx_max = max(0, min(n - 1, idx_max))
            if idx_min > idx_max:
                idx_min, idx_max = idx_max, idx_min

            zone_ranges = data.ranges[idx_min : idx_max + 1]

            valid = [r for r in zone_ranges
                     if not math.isinf(r) and not math.isnan(r)
                     and self.RANGE_MIN_VALID < r < self.RANGE_MAX_VALID]

            self.active_ranges = len(valid)

            if not valid:
                self.min_dist      = 99.9
                self.obstacle_stop = False
                self.obstacle_slow = False
                return

            self.min_dist      = min(valid)
            self.obstacle_stop = self.min_dist < self.STOP_DISTANCE
            self.obstacle_slow = self.min_dist < self.SLOW_DISTANCE

        except Exception as e:
            print(f"[LIDAR] Callback error: {e}")


# ==========================================
# KELAS KONTROLER UTAMA AMR
# ==========================================
class AMRController:
    def __init__(self):
        if ROS_AVAILABLE:
            try:
                rospy.init_node('amr_controller', anonymous=True)
                print("[ROS] Node 'amr_controller' berhasil diinisialisasi.")
            except rospy.exceptions.ROSException as e:
                print(f"[ROS] init_node: {e}")

        self.QR_REAL_SIZE_MM = 30.0
        self.TRACK_WIDTH_MM  = 377.0 

        # --- SETUP HARDWARE ---
        self.digital   = CK5162E_Controller("192.168.2.30")
        self.analog    = CKDA08ETH_Controller("192.168.1.30")
        self.PIN_R_FWD = 1; self.PIN_R_REV = 2; self.PIN_R_BRK = 3
        self.PIN_L_FWD = 4; self.PIN_L_REV = 5; self.PIN_L_BRK = 6
        self.ANA_CH_R      = 0; self.ANA_CH_L  = 1
        self.PIN_ESTOP     = 0
        self.PIN_START_AUTO = 1
        self.PIN_STOP_AUTO  = 2

        self._btn_start_prev = False
        self._btn_stop_prev  = False

        # =======================================================
        # KALIBRASI MOTOR (TRIM) & KOREKSI ARAH
        # =======================================================
        self.TRIM_L_AUTO      = 0.96
        self.TRIM_R_AUTO      = 1.00
        
        self.DIR_KOREKSI      = -1
        self.IMU_DIR          = 1
        self.REVERSE_CAMERA_X = False

        # --- KONFIGURASI VOLTASE & KECEPATAN ---
        self.V_SCALE            = 20.0
        self.LIMIT_SPD_LINEAR   = 1.5 * self.V_SCALE
        self.LIMIT_SPD_ROTATION = 1.0 * self.V_SCALE
        self.V_CRUISE           = 1.5 * self.V_SCALE
        self.V_APPROACH         = 0.8 * self.V_SCALE
        self.V_STALL            = 0.4 * self.V_SCALE
        self.V_ALIGN            = 0.3 * self.V_SCALE

        # --- ALIGN_Y: kecepatan proporsional ---
        # Jauh (>150px) → V_ALIGN_Y_FAST, dekat (<=deadzone_y) → V_ALIGN (pelan)
        self.V_ALIGN_Y_FAST     = 0.6 * self.V_SCALE   # kecepatan saat jauh
        self.ALIGN_Y_FAST_ZONE  = 150                   # px threshold "jauh"

        # --- KONFIGURASI S-CURVE & JARAK (1.5 METER = 150 cm) ---
        self.STOP_SEARCH_DIST   = 150.0
        self.TARGET_X_DIST_CM   = 150.0
        self.ACCEL_DIST_CM      = 35.0
        self.CORRECTION_DIST_CM = 110.0
        self.scurve_offset_cm   = 0.0

        # =======================================================
        # DESELARASI SMOOTH EXECUTE_MOVE
        # Mulai deselerasi dari DECEL_START_CM, kecepatan turun linear
        # hingga V_CREEP_FINAL saat CREEP_START_CM, lalu creep scan QR
        # =======================================================
        self.DECEL_START_CM  = 100.0   # mulai deselerasi di 100cm
        self.CREEP_START_CM  = 140.0   # masuk creep scan QR di 140cm
        self.CREEP_SPEED     = 0.4 * self.V_SCALE   # kecepatan creep
        self.HARD_STOP_CM    = 148.0   # berhenti paksa jika QR tidak terbaca

        # =======================================================
        # PARAMETER GAIN PID
        # =======================================================
        self.KP_IMU = 0.22
        self.KI_IMU = 0.01
        self.KD_IMU = 0.80
        self.KP_ENC = 0.06
        self.integral_err_imu = 0.0

        # --- Pulse align (ALIGN_ROTATION & ALIGN_Y) ---
        self.align_pulse_timer   = 0.0
        self.align_pulse_state   = "MOVE"
        self.ALIGN_MOVE_DURATION = 0.3
        self.ALIGN_STOP_DURATION = 1.0

        # --- State mesin ---
        self.connected         = False
        self.estop_active      = False
        self.mode              = "MANUAL"
        self.nav_mode          = "IDLE"
        self.nav_status_detail = "STANDBY"
        self.vision_error      = {'x': 0, 'y': 0, 'angle': 0.0, 'x_mm': 0.0, 'y_mm': 0.0}
        self.last_qr_seen_time = 0

        self.current_cmd           = "NONE"
        self.current_val           = "0"
        self.current_direction     = None
        self.brake_released_manual = False

        self.dist_traveled  = 0.0
        self.start_dist_l   = 0.0
        self.start_dist_r   = 0.0
        self.start_yaw      = 0.0
        self.target_yaw_abs = 0.0
        self.target_yaw     = 0.0
        self.target_large_angle = 0.0

        self.prev_err_imu     = 0.0
        self.jarak_asli_kiri  = 0.0
        self.jarak_asli_kanan = 0.0

        # =======================================================
        # PARAMETER ROTASI BESAR (ALIGN_LARGE_ROTATION)
        # =======================================================
        self.LARGE_ROT_TARGET_DEG = 90.0
        self.BRAKE_BUFFER_DEG     = 4.0    # buffer kompensasi overshoot
        self._pre_rot_x_offset    = 0.0
        self._rot_direction       = 0
        self.V_POST_ROT           = 0.3 * self.V_SCALE
        self._post_rot_creep_dir  = 1

        # POST_ROT_CREEP pulse (pelan-pelan agar kamera sempat baca QR)
        self.V_POST_ROT_CREEP      = 0.2 * self.V_SCALE  # lebih pelan dari V_POST_ROT
        self.POST_ROT_MOVE_DUR     = 0.2   # detik gerak per pulse
        self.POST_ROT_STOP_DUR     = 0.8   # detik berhenti per pulse (kamera scan)
        self._post_rot_pulse_timer = 0.0
        self._post_rot_pulse_state = "MOVE"

        # =======================================================
        # PARAMETER QR SEARCH ROTATION
        # =======================================================
        self.QR_TIMEOUT_SEARCH = 3.0
        self.QR_SEARCH_ROT_DIR = 1
        self.QR_SEARCH_MAX_DEG = 90.0

        # --- GOAL & HOME state ---
        self.goal_pending = False
        self.ignore_goal  = False
        self.return_home  = False

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

        if self.mode == "MANUAL":
            eff_trim_l = 1.00
            eff_trim_r = 1.00
        else:
            eff_trim_l = self.TRIM_L_AUTO
            eff_trim_r = self.TRIM_R_AUTO

        l_s = left_spd  * eff_trim_l
        r_s = right_spd * eff_trim_r

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
        base = 1.5 * self.V_SCALE  
        d = self.current_direction.upper()
        if   d == 'FORWARD' : self.set_motor( base,  base)
        elif d == 'BACKWARD': self.set_motor(-base, -base)
        elif d == 'LEFT'    : self.set_motor(-base,  base)
        elif d == 'RIGHT'   : self.set_motor( base, -base)
        else: self.stop_motor()

    # ----------------------------------------------------------
    # START LARGE ROTATION
    # Target dinamis dengan buffer kompensasi overshoot
    # ----------------------------------------------------------
    def start_large_rotation(self, target_angle):
        self.nav_mode           = "ALIGN_LARGE_ROTATION"
        self._rot_direction     = 1 if target_angle > 0 else -1

        # Set target dengan braking buffer untuk kompensasi overshoot
        raw_target = abs(target_angle)
        self.LARGE_ROT_TARGET_DEG = max(0.0, raw_target - self.BRAKE_BUFFER_DEG)

        self.target_large_angle = target_angle
        x_mm = self.vision_error.get('x_mm', 0.0)
        self._pre_rot_x_offset  = -x_mm if self.REVERSE_CAMERA_X else x_mm
        self.integral_err_imu   = 0.0
        self.prev_err_imu       = 0.0
        if self.trackless:
            self.start_yaw = self.trackless.get_yaw()
        self.set_nav_status_detail(
            f"LARGE ROT: {target_angle:.1f}° | target_eff={self.LARGE_ROT_TARGET_DEG:.1f}° "
            f"| x_off={self._pre_rot_x_offset:.1f}mm")

    # ----------------------------------------------------------
    # ENTER REACQUIRE
    # ----------------------------------------------------------
    def _enter_reacquire(self, qr_fully_visible):
        self.stop_motor()
        self._post_rot_pulse_timer = 0.0
        self._post_rot_pulse_state = "MOVE"
        if qr_fully_visible:
            self.nav_mode = "IDLE"
            self.reset_vision_system()
            self.set_nav_status_detail("ROTASI SELESAI, QR OK")
        else:
            x_sign    = 1 if self._pre_rot_x_offset >= 0 else -1
            rot_sign  = self._rot_direction
            creep_dir = x_sign * rot_sign
            label     = "MAJU" if creep_dir > 0 else "MUNDUR"
            self._post_rot_creep_dir = creep_dir
            self.nav_mode = "POST_ROT_CREEP"
            self.set_nav_status_detail(
                f"ROTASI SELESAI → {label} cari QR "
                f"(x_off={self._pre_rot_x_offset:.0f}mm, "
                f"belok={'KANAN' if rot_sign>0 else 'KIRI'})")

    # ----------------------------------------------------------
    # EXECUTE QR COMMAND
    # ----------------------------------------------------------
    def execute_qr_command(self, cmd, val):
        # MODE RETURN HOME
        if self.return_home:
            if "HOMEPOST" in cmd.upper() or "HOME" in cmd.upper():
                self.stop_motor()
                self.return_home  = False
                self.mode         = "MANUAL"
                self.nav_mode     = "GOAL_REACHED"
                self.set_nav_status_detail("✓ HOMEPOST TERCAPAI! Tekan START AUTO untuk lanjut.")
                return
            else:
                cmd = "MAJU"

        # QR GOAL
        elif "GOAL" in cmd.upper() or "STOP" in cmd.upper():
            if self.ignore_goal:
                self.ignore_goal = False
                cmd = "MAJU"
            else:
                self.stop_motor()
                self.mode         = "MANUAL"
                self.nav_mode     = "GOAL_REACHED"
                self.goal_pending = False
                self.set_nav_status_detail("✓ GOAL TERCAPAI! Tekan START AUTO untuk lanjut.")
                return

        if "PUTAR" in cmd.upper() and self.trackless:
            initial_yaw         = self.trackless.get_yaw()
            move_angle          = 90.0 if "KANAN" in cmd.upper() else -90.0
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
                                f"{self.scurve_offset_cm:.2f},0.00,0.00,"
                                f"0.00,0.00,0.00,0.00,0.00\n")
                except: pass

            self.dist_traveled    = 0.0
            self.prev_err_imu     = 0.0
            self.integral_err_imu = 0.0
            self.nav_mode         = "EXECUTE_MOVE"

    # ----------------------------------------------------------
    # CONTROL LOOP
    # ----------------------------------------------------------
    def control_loop(self):
        while True:
            if self.connected:
                # --- DI0: Emergency Stop (NC — aktif saat LOW) ---
                estop = not self.digital.read_input(self.PIN_ESTOP)
                if estop != self.estop_active:
                    self.estop_active = estop
                    if estop:
                        self.mode = "MANUAL"
                        print("[ESTOP] Emergency Stop aktif!")
                    else:
                        print("[ESTOP] Emergency Stop dilepas.")

                # --- DI1: Tombol START AUTO (rising-edge) ---
                btn_start = self.digital.read_input(self.PIN_START_AUTO)
                if btn_start and not self._btn_start_prev:
                    if not self.estop_active:
                        self.mode = "AUTO"
                        if self.nav_mode == "GOAL_REACHED":
                            self.nav_mode    = "IDLE"
                            self.ignore_goal = True
                            self.reset_vision_system()
                            self.set_nav_status_detail("LANJUT DARI GOAL... (Tombol DI1)")
                        else:
                            self.set_nav_status_detail("MODE AUTO aktif (Tombol DI1)")
                        print("[DI1] START AUTO ditekan → MODE AUTO")
                self._btn_start_prev = btn_start

                # --- DI2: Tombol STOP AUTO / kembali MANUAL (rising-edge) ---
                btn_stop = self.digital.read_input(self.PIN_STOP_AUTO)
                if btn_stop and not self._btn_stop_prev:
                    self.return_home = False
                    self.mode        = "MANUAL"
                    self.stop_motor()
                    self.set_nav_status_detail("MODE MANUAL aktif (Tombol DI2)")
                    print("[DI2] STOP AUTO ditekan → MODE MANUAL")
                self._btn_stop_prev = btn_stop

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
            abs_yaw               = self.trackless.get_yaw()
            rel_yaw               = (abs_yaw - self.start_yaw + 180) % 360 - 180
            rel_dist_l            = self.trackless.dist_l - self.start_dist_l
            rel_dist_r            = self.trackless.dist_r - self.start_dist_r
            self.jarak_asli_kiri  = abs(rel_dist_l)
            self.jarak_asli_kanan = abs(rel_dist_r)
            self.dist_traveled    = (self.jarak_asli_kiri + self.jarak_asli_kanan) / 2.0

        # -- WAITING_STABLE ----------------------------------------
        if self.nav_mode == "WAITING_STABLE":
            self.stop_motor()
            return

        # -- GOAL_REACHED ------------------------------------------
        if self.nav_mode == "GOAL_REACHED":
            self.stop_motor()
            self.set_nav_status_detail("✓ GOAL TERCAPAI! Tekan START AUTO untuk lanjut.")
            return

        # -- ALIGN_LARGE_ROTATION ----------------------------------
        if self.nav_mode == "ALIGN_LARGE_ROTATION":
            degrees_rotated = abs(rel_yaw)
            remaining_deg   = self.LARGE_ROT_TARGET_DEG - degrees_rotated

            if remaining_deg <= 0.0:
                self._enter_reacquire(qr_sight)
            else:
                if   remaining_deg > 30.0: rot_speed = 0.50 * self.V_SCALE
                elif remaining_deg > 15.0: rot_speed = 0.35 * self.V_SCALE
                else:                      rot_speed = 0.22 * self.V_SCALE
                out = math.copysign(rot_speed, self.target_large_angle)
                self.set_motor(out, -out)
                self.set_nav_status_detail(
                    f"LARGE ROT: sisa {remaining_deg:.1f}° / "
                    f"target {self.LARGE_ROT_TARGET_DEG:.0f}° | {rot_speed:.1f}V")
            return

        # -- POST_ROT_CREEP ----------------------------------------
        # -- POST_ROT_CREEP (pulse pelan agar kamera sempat scan QR) --
        if self.nav_mode == "POST_ROT_CREEP":
            if qr_sight:
                self.stop_motor()
                self._post_rot_pulse_timer = 0.0
                self._post_rot_pulse_state = "MOVE"
                self.nav_mode = "IDLE"
                self.reset_vision_system()
                self.set_nav_status_detail("QR TERBACA SETELAH ROTASI → IDLE")
                return
            if self.lidar.obstacle_stop:
                self.stop_motor(); return

            current_t = time.time()
            if self._post_rot_pulse_timer == 0.0:
                self._post_rot_pulse_timer = current_t
                self._post_rot_pulse_state = "MOVE"

            elapsed = current_t - self._post_rot_pulse_timer
            label   = "MAJU" if self._post_rot_creep_dir > 0 else "MUNDUR"

            if self._post_rot_pulse_state == "MOVE":
                if elapsed > self.POST_ROT_MOVE_DUR:
                    self._post_rot_pulse_state = "STOP"
                    self._post_rot_pulse_timer = current_t
                    self.stop_motor()
                else:
                    spd = self.V_POST_ROT_CREEP * self._post_rot_creep_dir
                    self.set_motor(spd, spd)
                self.set_nav_status_detail(
                    f"POST ROT CREEP: {label} GERAK {self.V_POST_ROT_CREEP/self.V_SCALE:.1f}V | cari QR...")
                return

            elif self._post_rot_pulse_state == "STOP":
                self.stop_motor()
                if elapsed > self.POST_ROT_STOP_DUR:
                    self._post_rot_pulse_state = "MOVE"
                    self._post_rot_pulse_timer = current_t
                self.set_nav_status_detail(
                    f"POST ROT CREEP: {label} JEDA {self.POST_ROT_STOP_DUR:.1f}s | scan QR...")
                return

        # -- ALIGN_TO_90 (pulse menuju ±90°) -----------------------
        if self.nav_mode == "ALIGN_TO_90":
            err_a     = self.vision_error['angle']
            target_90 = 90.0 if err_a > 0 else -90.0
            err_to_90 = err_a - target_90

            # Sudah dalam deadzone ±90° → lanjut rotasi besar
            if abs(err_to_90) <= self.qr_sys.deadzone_angle if self.qr_sys else 0.5:
                self.nav_mode = "IDLE"
                self.reset_vision_system()
                self.set_nav_status_detail("ALIGN TO 90° SELESAI → siap LARGE ROT")
                return

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
                    out = math.copysign(self.V_ALIGN, err_to_90)
                    self.set_motor(out, -out)
                return
            elif self.align_pulse_state == "STOP":
                self.set_motor(0, 0)
                if elapsed > self.ALIGN_STOP_DURATION:
                    self.align_pulse_state = "MOVE"
                    self.align_pulse_timer = current_t
                return

        # =======================================================
        # QR SEARCH ROTATION
        # =======================================================
        if self.nav_mode == "IDLE":
            qr_timeout = (
                self.last_qr_seen_time > 0 and
                time.time() - self.last_qr_seen_time > self.QR_TIMEOUT_SEARCH
            )
            if qr_timeout:
                self.set_nav_status_detail(
                    f"QR HILANG >{self.QR_TIMEOUT_SEARCH:.0f}s "
                    f"→ SEARCH ROTATION {'KANAN' if self.QR_SEARCH_ROT_DIR > 0 else 'KIRI'}")
                search_angle = self.QR_SEARCH_MAX_DEG * self.QR_SEARCH_ROT_DIR
                self.start_large_rotation(search_angle)
                self.nav_mode = "QR_SEARCH_ROTATION"
                if self.trackless:
                    self.start_yaw = self.trackless.get_yaw()
            return

        if self.nav_mode == "QR_SEARCH_ROTATION":
            degrees_rotated = abs(rel_yaw)
            remaining_deg   = self.LARGE_ROT_TARGET_DEG - degrees_rotated

            if qr_sight:
                self.stop_motor()
                self.nav_mode = "IDLE"
                self.reset_vision_system()
                self.set_nav_status_detail("✓ QR DITEMUKAN saat SEARCH ROTATION → IDLE")
                return

            if remaining_deg <= 0.0:
                self.stop_motor()
                self.QR_SEARCH_ROT_DIR *= -1
                self.set_nav_status_detail(
                    f"SEARCH ROTATION habis → balik arah "
                    f"{'KANAN' if self.QR_SEARCH_ROT_DIR > 0 else 'KIRI'}")
                time.sleep(0.3)
                search_angle = self.QR_SEARCH_MAX_DEG * self.QR_SEARCH_ROT_DIR
                self.start_large_rotation(search_angle)
                self.nav_mode = "QR_SEARCH_ROTATION"
                if self.trackless:
                    self.start_yaw = self.trackless.get_yaw()
                return

            if self.lidar.obstacle_stop:
                self.stop_motor()
                self.set_nav_status_detail("SEARCH ROTATION: OBSTACLE STOP!")
                return

            rot_speed = 0.30 * self.V_SCALE
            out = math.copysign(rot_speed, self.target_large_angle)
            self.set_motor(out, -out)
            self.set_nav_status_detail(
                f"QR SEARCH ROT: sisa {remaining_deg:.1f}° "
                f"| {'KANAN' if self.QR_SEARCH_ROT_DIR > 0 else 'KIRI'} | 0.3V")
            return

        # -- ALIGN_ROTATION & ALIGN_Y (pulse proporsional) --------
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
                        # ==================================================
                        # ALIGN_Y PROPORSIONAL:
                        # Jauh (|ey| > ALIGN_Y_FAST_ZONE) → V_ALIGN_Y_FAST
                        # Sedang                           → interpolasi linear
                        # Dekat (|ey| <= deadzone_y)       → V_ALIGN (pelan)
                        # ==================================================
                        deadzone_y = self.qr_sys.deadzone_y if self.qr_sys else 25
                        abs_ey     = abs(err_y)

                        if abs_ey >= self.ALIGN_Y_FAST_ZONE:
                            spd_y = self.V_ALIGN_Y_FAST
                        elif abs_ey <= deadzone_y:
                            spd_y = self.V_ALIGN
                        else:
                            # Interpolasi linear dari V_ALIGN ke V_ALIGN_Y_FAST
                            t     = (abs_ey - deadzone_y) / (self.ALIGN_Y_FAST_ZONE - deadzone_y)
                            spd_y = self.V_ALIGN + t * (self.V_ALIGN_Y_FAST - self.V_ALIGN)

                        out = math.copysign(spd_y, err_y)
                        self.set_motor(out, out)
                        self.set_nav_status_detail(
                            f"ALIGN Y: {err_y}px | spd={spd_y/self.V_SCALE:.2f}V")
                return

            elif self.align_pulse_state == "STOP":
                self.set_motor(0, 0)
                if elapsed > self.ALIGN_STOP_DURATION:
                    self.align_pulse_state = "MOVE"
                    self.align_pulse_timer = current_t
                return
        else:
            self.align_pulse_timer = 0.0

        # -- EXECUTE_ROTATION_90 -----------------------------------
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

        # -- EXECUTE_MOVE (S-Curve PID + smooth deceleration) ------
        elif self.nav_mode == "EXECUTE_MOVE":

            # Berhenti paksa jika QR tidak terbaca sampai HARD_STOP_CM
            if self.dist_traveled >= self.HARD_STOP_CM:
                self.stop_motor()
                self.set_nav_status_detail(
                    f"BERHENTI PAKSA (QR tidak terbaca) | Dist: {self.dist_traveled:.1f}cm")
                self.reset_vision_system()
                return

            # -------------------------------------------------------
            # CREEP ZONE: dist >= CREEP_START_CM
            # Kecepatan sudah sangat rendah, scan QR untuk berhenti
            # -------------------------------------------------------
            if self.dist_traveled >= self.CREEP_START_CM:
                if qr_sight:
                    self.stop_motor()
                    self.set_nav_status_detail(
                        f"QR TERBACA, BERHENTI | Dist: {self.dist_traveled:.1f}cm")
                    self.reset_vision_system()
                    return
                if self.lidar.obstacle_stop:
                    self.stop_motor(); return
                self.set_motor(self.CREEP_SPEED, self.CREEP_SPEED)
                self.set_nav_status_detail(
                    f"CREEP SCAN QR | Dist: {self.dist_traveled:.1f}cm")
                return

            # -------------------------------------------------------
            # PROFIL KECEPATAN dengan smooth deceleration
            #
            # Fase 1 — AKSELERASI  : 0 ~ ACCEL_DIST_CM
            #           ramp dari V_STALL ke V_CRUISE
            # Fase 2 — CRUISE      : ACCEL_DIST_CM ~ DECEL_START_CM
            #           tetap V_CRUISE
            # Fase 3 — DESELERASI  : DECEL_START_CM ~ CREEP_START_CM
            #           turun linear dari V_CRUISE ke CREEP_SPEED
            # Fase 4 — CREEP       : CREEP_START_CM ~ HARD_STOP_CM
            #           (ditangani blok di atas)
            # -------------------------------------------------------
            if self.dist_traveled < self.ACCEL_DIST_CM:
                # Fase 1: akselerasi
                ramp         = self.dist_traveled / self.ACCEL_DIST_CM
                target_speed = self.V_STALL + (self.V_CRUISE - self.V_STALL) * ramp

            elif self.dist_traveled < self.DECEL_START_CM:
                # Fase 2: cruise
                target_speed = self.V_CRUISE

            else:
                # Fase 3: deselerasi smooth linear
                decel_range  = self.CREEP_START_CM - self.DECEL_START_CM
                decel_done   = self.dist_traveled - self.DECEL_START_CM
                t_decel      = max(0.0, min(1.0, decel_done / decel_range))
                # Gunakan ease-out (kuadratik) agar lebih smooth
                t_smooth     = 1.0 - (1.0 - t_decel) ** 2
                target_speed = self.V_CRUISE + t_smooth * (self.CREEP_SPEED - self.V_CRUISE)
                target_speed = max(self.CREEP_SPEED, target_speed)

            if self.lidar.obstacle_stop:
                self.stop_motor(); return

            # --- PID S-Curve correction ---
            correction = 0.0
            if self.trackless:
                active_dist = max(10.0, min(self.TARGET_X_DIST_CM, self.CORRECTION_DIST_CM))
                x           = max(0.0, min(self.dist_traveled, self.TARGET_X_DIST_CM))

                if x < active_dist:
                    ratio           = x / active_dist
                    y_prime         = (6.0 * self.scurve_offset_cm / active_dist) * (ratio - ratio**2)
                    dyn_yaw         = math.degrees(math.atan(y_prime))
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

                d_err_imu         = err_imu - self.prev_err_imu
                self.prev_err_imu = err_imu

                correction = (err_imu               * self.KP_IMU
                            + self.integral_err_imu * self.KI_IMU
                            + d_err_imu             * self.KD_IMU
                            + err_enc               * self.KP_ENC)

                max_corr   = target_speed * 0.25
                correction = max(-max_corr, min(max_corr, correction))

                self.set_nav_status_detail(
                    f"S-CRV | Ty:{self.target_yaw:.1f}° | Y:{rel_yaw:.1f}° "
                    f"| spd:{target_speed/self.V_SCALE:.2f}V | C:{correction:.1f}")

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
            if self.nav_mode == "GOAL_REACHED":
                self.nav_mode    = "IDLE"
                self.ignore_goal = True
                self.reset_vision_system()
                self.set_nav_status_detail("LANJUT DARI GOAL...")
        elif p == "RETURN_HOME":
            self.return_home = True
            self.mode        = "AUTO"
            self.nav_mode    = "IDLE"
            self.reset_vision_system()
            self.set_nav_status_detail("🏠 RETURN HOME aktif — cari HOMEPOST...")
        elif p in ["STOP_AUTO", "STOP", "SPACE"]:
            self.return_home = False
            self.mode        = "MANUAL"
            self.stop_motor()
        elif p in ["W", "A", "S", "D"]:
            self.return_home = False
            self.mode        = "MANUAL"
            if p == "W": self.current_direction = "FORWARD"
            elif p == "S": self.current_direction = "BACKWARD"
            elif p == "A": self.current_direction = "LEFT"
            elif p == "D": self.current_direction = "RIGHT"
            self.execute_manual_control()


if __name__ == "__main__":
    import signal

    c = AMRController()
    c.mode = "MANUAL"

    def handle_shutdown(signum, frame):
        print("\n[SYSTEM] Signal diterima, mematikan AMR...")
        c.shutdown_system()
        try:
            c.client.loop_stop()
            c.client.disconnect()
        except: pass
        if ROS_AVAILABLE:
            try: rospy.signal_shutdown("User interrupt")
            except: pass
        sys.exit(0)

    signal.signal(signal.SIGINT,  handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    print("\n" + "="*50)
    print("AMR CONTROLLER AKTIF (MODE MANUAL)")
    print("Kontrol Keyboard (Terminal):")
    print(" [W] : Maju")
    print(" [S] : Mundur")
    print(" [A] : Kiri")
    print(" [D] : Kanan")
    print(" [SPACE] : Berhenti Motor")
    print(" [Q] atau Ctrl+C : Keluar/Shutdown")
    print("="*50 + "\n")

    try:
        import tty, termios
        def getch():
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setcbreak(fd)
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

        while True:
            key = getch().lower()
            if key in ("\x03", "q"):
                handle_shutdown(None, None)
            if c.mode == "MANUAL":
                if key == "w":
                    c.current_direction = "FORWARD"
                    c.execute_manual_control()
                    print("\r[MANUAL] MAJU (1.5V)          ", end="", flush=True)
                elif key == "s":
                    c.current_direction = "BACKWARD"
                    c.execute_manual_control()
                    print("\r[MANUAL] MUNDUR (-1.5V)       ", end="", flush=True)
                elif key == "a":
                    c.current_direction = "LEFT"
                    c.execute_manual_control()
                    print("\r[MANUAL] BELOK KIRI           ", end="", flush=True)
                elif key == "d":
                    c.current_direction = "RIGHT"
                    c.execute_manual_control()
                    print("\r[MANUAL] BELOK KANAN          ", end="", flush=True)
                elif key == " ":
                    c.current_direction = "NONE"
                    c.stop_motor()
                    print("\r[MANUAL] BERHENTI (STOP)      ", end="", flush=True)

    except Exception as e:
        print(f"\n[INFO] Mode Keyboard Cepat gagal ({e}). Gunakan input() biasa.")
        while True:
            try:
                cmd = input("WASD / Space / Q: ").lower()
                if cmd == "q":
                    handle_shutdown(None, None)
                if c.mode == "MANUAL":
                    if cmd == "w":
                        c.current_direction = "FORWARD"; c.execute_manual_control()
                    elif cmd == "s":
                        c.current_direction = "BACKWARD"; c.execute_manual_control()
                    elif cmd == "a":
                        c.current_direction = "LEFT"; c.execute_manual_control()
                    elif cmd == "d":
                        c.current_direction = "RIGHT"; c.execute_manual_control()
                    elif cmd in (" ", ""):
                        c.current_direction = "NONE"; c.stop_motor()
            except KeyboardInterrupt:
                handle_shutdown(None, None)
