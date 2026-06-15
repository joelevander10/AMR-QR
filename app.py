import cv2
import time
import threading
import json
import numpy as np
import math
from flask import Flask, render_template, Response, request, jsonify

# Import Controller Utama
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

        # 1. Deteksi QR (OpenCV Bawaan Sesuai Request User)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        retval, decoded_info, points, _ = self.detector.detectAndDecodeMulti(gray)

        if retval:
            for i, qr_data in enumerate(decoded_info):
                if not qr_data: continue
                
                # Ambil koordinat titik sudut
                qr_corners = points[i].astype(int)
                qr_center_x = int(np.mean(qr_corners[:, 0]))
                qr_center_y = int(np.mean(qr_corners[:, 1]))
                
                error_x = qr_center_x - self.center_x
                error_y = self.center_y - qr_center_y
                
                pt_tl = qr_corners[0]; pt_tr = qr_corners[1]
                angle_deg = math.degrees(math.atan2(pt_tr[1] - pt_tl[1], pt_tr[0] - pt_tl[0]))

                # --- HITUNG LEBAR QR UNTUK KALIBRASI DINAMIS ---
                qr_width_px = math.sqrt((pt_tr[0] - pt_tl[0])**2 + (pt_tr[1] - pt_tl[1])**2)
                
                cmd, val = "NONE", "0"
                if ":" in qr_data:
                    parts = qr_data.split(":")
                    cmd = parts[0].upper(); val = parts[1]
                else:
                    cmd = qr_data.upper()

                # --- VISUALISASI ---
                for j in range(4):
                    cv2.line(frame, tuple(qr_corners[j]), tuple(qr_corners[(j+1)%4]), (0, 255, 0), 2)
                cv2.circle(frame, (qr_center_x, qr_center_y), 5, (0, 0, 255), -1)
                
                # Tampilkan info lebar piksel untuk verifikasi di website
                cv2.putText(frame, f"W: {qr_width_px:.1f}px", (qr_center_x + 10, qr_center_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # KIRIM KE CONTROLLER UTAMA (Ditambah parameter qr_width_px)
                self.control_logic(frame, error_x, error_y, angle_deg, cmd, val, qr_width_px)
        else:
            cv2.putText(frame, "SCANNING FLOOR...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        with global_frame_lock:
            ret, buffer = cv2.imencode('.jpg', frame)
            if ret: global_last_frame = buffer.tobytes()

# Terapkan patch sebelum inisiasi
IntegratedQRSystem.process_stream = patched_process_stream


# ==========================================
# 2. FLASK WEB SERVER SETUP
# ==========================================
app = Flask(__name__)
robot = None 

@app.route('/')
def index():
    return render_template('index.html')

def generate_video():
    while True:
        with global_frame_lock:
            if global_last_frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + global_last_frame + b'\r\n')
        time.sleep(0.05)

@app.route('/video_feed')
def video_feed():
    return Response(generate_video(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/status')
def get_status():
    if not robot: return jsonify({"status": "error"})
    
    # Ambil data Odometri & Sensor Fusion
    trackless_data = None
    if hasattr(robot, 'trackless') and robot.trackless:
        trackless_data = {
            "node_l": getattr(robot.trackless, 'encoder_left_id', 1),
            "node_r": getattr(robot.trackless, 'encoder_right_id', 2),
            "yaw": round(robot.trackless.yaw_val, 2),
            "target_yaw": round(robot.trackless.target_yaw, 2),
            "enc_l": round(robot.trackless.dist_l, 2),
            "enc_r": round(robot.trackless.dist_r, 2),
            "dist_traveled": round(getattr(robot, 'dist_traveled', 0.0), 2)
        }

    # KONVERSI SATUAN KE CM
    # Lidar (Aslinya Meter)
    lidar_cm = 999.0
    lidar_is_stop = False
    if hasattr(robot, 'lidar'):
        lidar_cm = round(robot.lidar.min_dist * 100.0, 1) # Meter ke cm
        lidar_is_stop = robot.lidar.obstacle_stop

    # Vision Offset (Aslinya Milimeter)
    vision_cm = {
        'x': robot.vision_error.get('x', 0),
        'y': robot.vision_error.get('y', 0),
        'angle': robot.vision_error.get('angle', 0.0),
        'x_cm': round(robot.vision_error.get('x_mm', 0) / 10.0, 1), # mm ke cm
        'y_cm': round(robot.vision_error.get('y_mm', 0) / 10.0, 1)  # mm ke cm
    }

    return jsonify({
        "connected": robot.connected,
        "mode": robot.mode,
        "nav_mode": robot.nav_mode,
        "nav_status_detail": robot.nav_status_detail, 
        "estop": robot.estop_active,
        "lidar_dist_cm": lidar_cm,
        "lidar_stop": lidar_is_stop,
        "last_cmd": f"{robot.current_cmd} ({robot.current_val})",
        "vision_error": vision_cm,
        "trackless": trackless_data
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
        robot.current_direction = direction; robot.execute_manual_control()
    return jsonify({"status": "ok"})

@app.route('/api/manual/stop', methods=['POST'])
def api_manual_stop():
    if robot: robot.current_direction = None; robot.stop_motor()
    return jsonify({"status": "ok"})

if __name__ == "__main__":
    print("[FLASK] Memulai Server AGV Dashboard...")
    robot = AMRController()
    app.run(host='0.0.0.0', port=5050, debug=False, use_reloader=False)
