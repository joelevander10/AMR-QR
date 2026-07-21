import sys
import cv2
import time
import threading
import json
import numpy as np
import math
from flask import Flask, render_template, Response, request, jsonify

try:
    from amr_controller import AMRController, IntegratedQRSystem
except ImportError:
    print("[FATAL] amr_controller.py tidak ditemukan!")
    exit()

# ==========================================
# 1. MODIFIKASI DINAMIS (MONKEY PATCH)
# ==========================================
global_last_frame = None
global_frame_lock = threading.Lock()

def patched_process_stream(self):
    global global_last_frame
    print("[VISION] Web Stream & QR Detector Started.")
    
    while True:
        if not hasattr(self, 'cap') or not self.cap.isOpened():
            time.sleep(1)
            continue
            
        ret, frame = self.cap.read()
        if not ret: break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        retval, decoded_info, points, _ = self.detector.detectAndDecodeMulti(gray)

        if retval:
            for i, qr_data in enumerate(decoded_info):
                if not qr_data: continue
                
                qr_corners = points[i].astype(float)
                qr_center_x = int(np.mean(qr_corners[:, 0]))
                qr_center_y = int(np.mean(qr_corners[:, 1]))
                
                error_x = qr_center_x - self.center_x
                error_y = self.center_y - qr_center_y
                
                top_center_x    = (qr_corners[0][0] + qr_corners[1][0]) / 2.0
                top_center_y    = (qr_corners[0][1] + qr_corners[1][1]) / 2.0
                bottom_center_x = (qr_corners[2][0] + qr_corners[3][0]) / 2.0
                bottom_center_y = (qr_corners[2][1] + qr_corners[3][1]) / 2.0

                dx        = top_center_x - bottom_center_x
                dy        = bottom_center_y - top_center_y
                angle_deg = math.degrees(math.atan2(dx, dy))

                pt_tl      = qr_corners[0]; pt_tr = qr_corners[1]
                qr_width_px = math.sqrt(
                    (pt_tr[0] - pt_tl[0])**2 + (pt_tr[1] - pt_tl[1])**2)
                
                cmd, val = "NONE", "0"
                if ":" in qr_data:
                    parts = qr_data.split(":")
                    cmd = parts[0].upper(); val = parts[1]
                else:
                    cmd = qr_data.upper()

                int_corners = qr_corners.astype(int)
                for j in range(4):
                    cv2.line(frame, tuple(int_corners[j]),
                             tuple(int_corners[(j+1)%4]), (0, 255, 0), 2)
                cv2.circle(frame, (qr_center_x, qr_center_y), 5, (0, 0, 255), -1)
                cv2.line(frame, (qr_center_x, qr_center_y),
                         (int(top_center_x), int(top_center_y)), (255, 0, 0), 3)
                cv2.putText(frame, f"W:{qr_width_px:.1f} A:{angle_deg:.1f}deg",
                            (qr_center_x + 10, qr_center_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                self.control_logic(frame, error_x, error_y, angle_deg,
                                   cmd, val, qr_width_px)
        else:
            cv2.putText(frame, "SCANNING FLOOR...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        with global_frame_lock:
            ret, buffer = cv2.imencode('.jpg', frame)
            if ret: global_last_frame = buffer.tobytes()

IntegratedQRSystem.process_stream = patched_process_stream

# ==========================================
# 2. FLASK WEB SERVER SETUP
# ==========================================
app   = Flask(__name__)
robot = None

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/settings')
def settings():
    return render_template('settings.html')

def generate_video():
    placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(placeholder, "INITIALIZING CAMERA...", (110, 240),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    _, buf = cv2.imencode('.jpg', placeholder)
    placeholder_bytes = buf.tobytes()

    while True:
        frame_to_yield = placeholder_bytes
        with global_frame_lock:
            if global_last_frame is not None:
                frame_to_yield = global_last_frame
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_to_yield + b'\r\n')
        time.sleep(0.05)

@app.route('/video_feed')
def video_feed():
    return Response(generate_video(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/status')
def get_status():
    if not robot: return jsonify({"status": "error"})

    trackless_data = None
    if hasattr(robot, 'trackless') and robot.trackless:
        rel_l = round(robot.trackless.dist_l - robot.start_dist_l, 2)
        rel_r = round(robot.trackless.dist_r - robot.start_dist_r, 2)
        trackless_data = {
            "node_l"       : getattr(robot.trackless, 'encoder_left_id', 1),
            "node_r"       : getattr(robot.trackless, 'encoder_right_id', 2),
            "yaw"          : round(robot.trackless.yaw_val, 2),
            "target_yaw"   : round(robot.trackless.target_yaw, 2),
            "enc_l"        : rel_l,
            "enc_r"        : rel_r,
            "dist_traveled": round(getattr(robot, 'dist_traveled', 0.0), 2)
        }

    lidar_data = {
        "dist_cm"    : 999.0,
        "is_stop"    : False,
        "is_slow"    : False,
        "active_rays": 0,
        "zone_deg"   : "±45°"
    }
    if hasattr(robot, 'lidar'):
        ld = robot.lidar
        lidar_data = {
            "dist_cm"    : round(ld.min_dist * 100.0, 1),
            "is_stop"    : ld.obstacle_stop,
            "is_slow"    : ld.obstacle_slow,
            "active_rays": getattr(ld, 'active_ranges', 0),
            "zone_deg"   : f"{ld.ANGLE_MIN_DEG:.0f}°~{ld.ANGLE_MAX_DEG:.0f}°"
        }

    vision_cm = {
        'x'    : robot.vision_error.get('x', 0),
        'y'    : robot.vision_error.get('y', 0),
        'angle': robot.vision_error.get('angle', 0.0),
        'x_cm' : round(robot.vision_error.get('x_mm', 0) / 10.0, 1),
        'y_cm' : round(robot.vision_error.get('y_mm', 0) / 10.0, 1)
    }

    return jsonify({
        "connected"        : robot.connected,
        "mode"             : robot.mode,
        "nav_mode"         : robot.nav_mode,
        "nav_status_detail": robot.nav_status_detail,
        "estop"            : robot.estop_active,
        "lidar"            : lidar_data,
        "lidar_dist_cm"    : lidar_data["dist_cm"],
        "lidar_stop"       : lidar_data["is_stop"],
        "last_cmd"         : f"{robot.current_cmd} ({robot.current_val})",
        "vision_error"     : vision_cm,
        "trackless"        : trackless_data,
        "selected_loop"     : robot.selected_loop,
        "target_align_angle": robot.target_align_angle,
        "cabang_rotated"    : robot._cabang_rotated,
        "current_goal"      : robot.current_goal
    })

# ==========================================
# 3. PARAMETER API
# GET  /api/params  → baca semua parameter saat ini
# POST /api/params  → update satu atau lebih parameter sekaligus
# ==========================================
@app.route('/api/params', methods=['GET'])
def get_params():
    if not robot:
        return jsonify({"status": "error", "message": "Robot belum siap"})

    r = robot
    ld = robot.lidar

    params = {
        # --- VISION ---
        "QR_REAL_SIZE_MM"   : r.QR_REAL_SIZE_MM,

        # --- KECEPATAN (dalam satuan V_SCALE unit, tampil sebagai volt) ---
        "V_CRUISE_V"        : round(r.V_CRUISE   / r.V_SCALE, 2),
        "V_APPROACH_V"      : round(r.V_APPROACH / r.V_SCALE, 2),
        "V_STALL_V"         : round(r.V_STALL    / r.V_SCALE, 2),
        "V_ALIGN_V"         : round(r.V_ALIGN    / r.V_SCALE, 2),
        "V_ALIGN_Y_FAST_V"  : round(r.V_ALIGN_Y_FAST / r.V_SCALE, 2),
        "V_POST_ROT_V"      : round(r.V_POST_ROT / r.V_SCALE, 2),
        "CREEP_SPEED_V"     : round(r.CREEP_SPEED / r.V_SCALE, 2),

        # --- LIMIT KECEPATAN ---
        "LIMIT_SPD_LINEAR_V"  : round(r.LIMIT_SPD_LINEAR   / r.V_SCALE, 2),
        "LIMIT_SPD_ROTATION_V": round(r.LIMIT_SPD_ROTATION / r.V_SCALE, 2),

        # --- JARAK NAVIGASI (cm) ---
        "TARGET_X_DIST_CM"          : r.TARGET_X_DIST_CM,
        "USE_Y_OFFSET_CORRECTION"   : r.USE_Y_OFFSET_CORRECTION,
        "Y_OFFSET_CORRECTION_MAX"   : r.Y_OFFSET_CORRECTION_MAX,
        "ACCEL_DIST_CM"     : r.ACCEL_DIST_CM,
        "DECEL_START_CM"    : r.DECEL_START_CM,
        "CREEP_START_CM"    : r.CREEP_START_CM,
        "HARD_STOP_CM"      : r.HARD_STOP_CM,
        "CORRECTION_DIST_CM": r.CORRECTION_DIST_CM,

        # --- ROTASI BESAR ---
        "BRAKE_BUFFER_DEG"  : r.BRAKE_BUFFER_DEG,

        # --- QR SEARCH ---
        "QR_TIMEOUT_SEARCH" : r.QR_TIMEOUT_SEARCH,
        "QR_SEARCH_MAX_DEG" : r.QR_SEARCH_MAX_DEG,

        # --- ALIGN PULSE ---
        "ALIGN_MOVE_DURATION" : r.ALIGN_MOVE_DURATION,
        "ALIGN_STOP_DURATION" : r.ALIGN_STOP_DURATION,
        "ALIGN_Y_FAST_ZONE"   : r.ALIGN_Y_FAST_ZONE,

        # --- KICK-START ---
        "V_ALIGN_KICK_V"        : round(r.V_ALIGN_KICK / r.V_SCALE, 2),
        "ALIGN_KICK_DUR"        : r.ALIGN_KICK_DUR,
        "ALIGN_KICK_THRESHOLD"  : r.ALIGN_KICK_THRESHOLD,

        # --- PID ---
        "KP_IMU"            : r.KP_IMU,
        "KI_IMU"            : r.KI_IMU,
        "KD_IMU"            : r.KD_IMU,
        "KP_ENC"            : r.KP_ENC,

        # --- TRIM & ARAH ---
        "TRIM_L_AUTO"       : r.TRIM_L_AUTO,
        "TRIM_R_AUTO"       : r.TRIM_R_AUTO,
        "DIR_KOREKSI"       : r.DIR_KOREKSI,
        "IMU_DIR"           : r.IMU_DIR,
        "REVERSE_CAMERA_X"  : r.REVERSE_CAMERA_X,

        # --- LIDAR SAFETY ---
        "LIDAR_STOP_M"      : ld.STOP_DISTANCE,
        "LIDAR_SLOW_M"      : ld.SLOW_DISTANCE,
        "LIDAR_ANGLE_MIN"   : ld.ANGLE_MIN_DEG,
        "LIDAR_ANGLE_MAX"   : ld.ANGLE_MAX_DEG,
        "LIDAR_RANGE_MIN_M" : ld.RANGE_MIN_VALID,
        "LIDAR_RANGE_MAX_M" : ld.RANGE_MAX_VALID,

        # --- TRACK ---
        "TRACK_WIDTH_MM"    : r.TRACK_WIDTH_MM,
    }
    return jsonify({"status": "ok", "params": params})


@app.route('/api/params', methods=['POST'])
def set_params():
    if not robot:
        return jsonify({"status": "error", "message": "Robot belum siap"})

    data    = request.json or {}
    updated = []
    errors  = []
    r       = robot
    ld      = robot.lidar

    def _set(key, converter, setter):
        if key in data:
            try:
                val = converter(data[key])
                setter(val)
                updated.append(f"{key}={val}")
            except Exception as e:
                errors.append(f"{key}: {e}")

    # --- VISION ---
    _set("QR_REAL_SIZE_MM",    float, lambda v: setattr(r, 'QR_REAL_SIZE_MM', v))

    # --- KECEPATAN (input volt, simpan *V_SCALE) ---
    _set("V_CRUISE_V",         float, lambda v: setattr(r, 'V_CRUISE',    v * r.V_SCALE))
    _set("V_APPROACH_V",       float, lambda v: setattr(r, 'V_APPROACH',  v * r.V_SCALE))
    _set("V_STALL_V",          float, lambda v: setattr(r, 'V_STALL',     v * r.V_SCALE))
    _set("V_ALIGN_V",          float, lambda v: setattr(r, 'V_ALIGN',     v * r.V_SCALE))
    _set("V_ALIGN_Y_FAST_V",   float, lambda v: setattr(r, 'V_ALIGN_Y_FAST', v * r.V_SCALE))
    _set("V_POST_ROT_V",       float, lambda v: setattr(r, 'V_POST_ROT',  v * r.V_SCALE))
    _set("CREEP_SPEED_V",      float, lambda v: setattr(r, 'CREEP_SPEED', v * r.V_SCALE))

    # --- LIMIT ---
    _set("LIMIT_SPD_LINEAR_V",   float, lambda v: setattr(r, 'LIMIT_SPD_LINEAR',   v * r.V_SCALE))
    _set("LIMIT_SPD_ROTATION_V", float, lambda v: setattr(r, 'LIMIT_SPD_ROTATION', v * r.V_SCALE))

    # --- JARAK (cm) ---
    _set("TARGET_X_DIST_CM",         float, lambda v: setattr(r, 'TARGET_X_DIST_CM',          v))
    _set("USE_Y_OFFSET_CORRECTION",  bool,  lambda v: setattr(r, 'USE_Y_OFFSET_CORRECTION',   v))
    _set("Y_OFFSET_CORRECTION_MAX",  float, lambda v: setattr(r, 'Y_OFFSET_CORRECTION_MAX',   v))
    _set("ACCEL_DIST_CM",      float, lambda v: setattr(r, 'ACCEL_DIST_CM',      v))
    _set("DECEL_START_CM",     float, lambda v: setattr(r, 'DECEL_START_CM',     v))
    _set("CREEP_START_CM",     float, lambda v: setattr(r, 'CREEP_START_CM',     v))
    _set("HARD_STOP_CM",       float, lambda v: setattr(r, 'HARD_STOP_CM',       v))
    _set("CORRECTION_DIST_CM", float, lambda v: setattr(r, 'CORRECTION_DIST_CM', v))

    # --- ROTASI ---
    _set("BRAKE_BUFFER_DEG",   float, lambda v: setattr(r, 'BRAKE_BUFFER_DEG',   v))

    # --- QR SEARCH ---
    _set("QR_TIMEOUT_SEARCH",  float, lambda v: setattr(r, 'QR_TIMEOUT_SEARCH',  v))
    _set("QR_SEARCH_MAX_DEG",  float, lambda v: setattr(r, 'QR_SEARCH_MAX_DEG',  v))

    # --- ALIGN PULSE ---
    _set("ALIGN_MOVE_DURATION", float, lambda v: setattr(r, 'ALIGN_MOVE_DURATION', v))
    _set("ALIGN_STOP_DURATION", float, lambda v: setattr(r, 'ALIGN_STOP_DURATION', v))
    _set("ALIGN_Y_FAST_ZONE",   float, lambda v: setattr(r, 'ALIGN_Y_FAST_ZONE',   v))

    # --- KICK-START ---
    _set("V_ALIGN_KICK_V",       float, lambda v: setattr(r, 'V_ALIGN_KICK',        v * r.V_SCALE))
    _set("ALIGN_KICK_DUR",       float, lambda v: setattr(r, 'ALIGN_KICK_DUR',       v))
    _set("ALIGN_KICK_THRESHOLD", float, lambda v: setattr(r, 'ALIGN_KICK_THRESHOLD', v))

    # --- PID ---
    _set("KP_IMU", float, lambda v: setattr(r, 'KP_IMU', v))
    _set("KI_IMU", float, lambda v: setattr(r, 'KI_IMU', v))
    _set("KD_IMU", float, lambda v: setattr(r, 'KD_IMU', v))
    _set("KP_ENC", float, lambda v: setattr(r, 'KP_ENC', v))

    # --- TRIM & ARAH ---
    _set("TRIM_L_AUTO",      float, lambda v: setattr(r, 'TRIM_L_AUTO', v))
    _set("TRIM_R_AUTO",      float, lambda v: setattr(r, 'TRIM_R_AUTO', v))
    _set("DIR_KOREKSI",      float, lambda v: setattr(r, 'DIR_KOREKSI', v))
    _set("IMU_DIR",          float, lambda v: setattr(r, 'IMU_DIR',     v))
    _set("REVERSE_CAMERA_X", bool, lambda v: setattr(r, 'REVERSE_CAMERA_X', v))

    # --- LIDAR ---
    _set("LIDAR_STOP_M",     float, lambda v: setattr(ld, 'STOP_DISTANCE',   v))
    _set("LIDAR_SLOW_M",     float, lambda v: setattr(ld, 'SLOW_DISTANCE',   v))
    _set("LIDAR_ANGLE_MIN",  float, lambda v: setattr(ld, 'ANGLE_MIN_DEG',   v))
    _set("LIDAR_ANGLE_MAX",  float, lambda v: setattr(ld, 'ANGLE_MAX_DEG',   v))
    _set("LIDAR_RANGE_MIN_M",float, lambda v: setattr(ld, 'RANGE_MIN_VALID', v))
    _set("LIDAR_RANGE_MAX_M",float, lambda v: setattr(ld, 'RANGE_MAX_VALID', v))

    # --- TRACK ---
    _set("TRACK_WIDTH_MM",   float, lambda v: setattr(r, 'TRACK_WIDTH_MM', v))

    return jsonify({
        "status" : "ok" if not errors else "partial",
        "updated": updated,
        "errors" : errors
    })


# ==========================================
# 4. LOOP SELECTOR API
# ==========================================
@app.route('/api/loop', methods=['GET'])
def get_loop():
    if not robot:
        return jsonify({"status": "error"})
    return jsonify({
        "status"       : "ok",
        "selected_loop": robot.selected_loop
    })

@app.route('/api/loop', methods=['POST'])
def set_loop():
    if not robot:
        return jsonify({"status": "error"})
    data = request.json or {}
    loop = int(data.get('loop', 1))
    if loop not in [1, 2]:
        return jsonify({"status": "error", "message": "loop harus 1 atau 2"})
    robot.selected_loop        = loop
    robot.target_align_angle   = 0.0   # reset alignment
    robot._cabang_rotated      = False
    print(f"[LOOP] Selected loop: {loop} "
          f"({'KIRI -90°' if loop == 1 else 'KANAN +90°'})")
    return jsonify({
        "status"       : "ok",
        "selected_loop": robot.selected_loop
    })

@app.route('/api/connect', methods=['POST'])
def api_connect():
    if robot: robot.connect_hardware()
    return jsonify({"status": "ok"})

@app.route('/api/estop', methods=['POST'])
def api_estop():
    if robot:
        robot.estop_active = not robot.estop_active
        if robot.estop_active:
            robot.stop_motor(); robot.mode = "MANUAL"
    return jsonify({"status": "ok", "estop": robot.estop_active})

@app.route('/api/mode', methods=['POST'])
def api_mode():
    data = request.json
    mode = data.get('mode', 'MANUAL')
    if robot and not robot.estop_active:
        robot.mode = mode
        if mode == 'MANUAL':
            robot.stop_motor()
        elif mode == 'AUTO':
            # Kalau sebelumnya berhenti di GOAL, lanjut dari sana
            if robot.nav_mode == 'GOAL_REACHED':
                robot.nav_mode    = 'IDLE'
                robot.ignore_goal = True
                robot.reset_vision_system()
                robot.set_nav_status_detail("LANJUT DARI GOAL... (Web HMI)")
    return jsonify({"status": "ok", "mode": robot.mode})

@app.route('/api/manual/move', methods=['POST'])
def api_manual_move():
    data = request.json
    direction = data.get('direction', 'FORWARD')
    if robot and robot.mode == 'MANUAL':
        robot.current_direction = direction
        robot.execute_manual_control()
    return jsonify({"status": "ok"})

@app.route('/api/manual/stop', methods=['POST'])
def api_manual_stop():
    if robot:
        robot.current_direction = None
        robot.stop_motor()
    return jsonify({"status": "ok"})

if __name__ == "__main__":
    import signal

    print("[FLASK] Memulai Server AMR Dashboard...")
    robot = AMRController()

    def handle_shutdown(signum, frame):
        print("\n[SYSTEM] Shutdown signal diterima...")
        if robot:
            robot.shutdown_system()
            try:
                robot.client.loop_stop()
                robot.client.disconnect()
            except: pass
        try:
            import rospy
            rospy.signal_shutdown("Flask shutdown")
        except: pass
        sys.exit(0)

    signal.signal(signal.SIGINT,  handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    try:
        import rospy
        ros_thread = threading.Thread(target=rospy.spin, daemon=True)
        ros_thread.start()
        print("[ROS] rospy.spin() berjalan di background thread.")
    except Exception as e:
        print(f"[ROS] rospy.spin thread gagal: {e}")

    app.run(host='0.0.0.0', port=5050, debug=False, use_reloader=False)
