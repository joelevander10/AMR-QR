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

                # Visualisasi
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

    # --- Trackless ---
    trackless_data = None
    if hasattr(robot, 'trackless') and robot.trackless:
        # Tampilkan encoder RELATIF dari titik start terakhir (reset saat MAJU/MUNDUR)
        # agar dashboard menunjukkan jarak tempuh sesi ini, bukan nilai absolut kumulatif
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

    # --- Lidar — sekarang kirim data lengkap ke dashboard ---
    lidar_data = {
        "dist_cm"      : 999.0,
        "is_stop"      : False,
        "is_slow"      : False,
        "active_rays"  : 0,         # jumlah ray valid di zona ±45°
        "zone_deg"     : "±45°"
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

    # --- Vision ---
    vision_cm = {
        'x'    : robot.vision_error.get('x', 0),
        'y'    : robot.vision_error.get('y', 0),
        'angle': robot.vision_error.get('angle', 0.0),
        'x_cm' : round(robot.vision_error.get('x_mm', 0) / 10.0, 1),
        'y_cm' : round(robot.vision_error.get('y_mm', 0) / 10.0, 1)
    }

    return jsonify({
        "connected"       : robot.connected,
        "mode"            : robot.mode,
        "nav_mode"        : robot.nav_mode,
        "nav_status_detail": robot.nav_status_detail,
        "estop"           : robot.estop_active,
        "lidar"           : lidar_data,
        # field lama tetap ada agar index.html lama tidak error
        "lidar_dist_cm"   : lidar_data["dist_cm"],
        "lidar_stop"      : lidar_data["is_stop"],
        "last_cmd"        : f"{robot.current_cmd} ({robot.current_val})",
        "vision_error"    : vision_cm,
        "trackless"       : trackless_data
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
        if mode == 'MANUAL': robot.stop_motor()
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

    print("[FLASK] Memulai Server AGV Dashboard...")
    robot = AMRController()

    # Signal handler — Ctrl+C / kill / systemctl stop
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

    # rospy.spin() di thread terpisah agar callback lidar diproses
    try:
        import rospy
        ros_thread = threading.Thread(target=rospy.spin, daemon=True)
        ros_thread.start()
        print("[ROS] rospy.spin() berjalan di background thread.")
    except Exception as e:
        print(f"[ROS] rospy.spin thread gagal: {e}")

    app.run(host='0.0.0.0', port=5050, debug=False, use_reloader=False)
