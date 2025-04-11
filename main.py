#!/usr/bin/env python3
import subprocess
import threading
import time
from flask import Flask, Response, jsonify
import cv2

current_mode = "idle"  # Modes: "idle", "navigation", "trash"

app = Flask(__name__)

@app.route('/start', methods=['GET'])
def start_nav():
    global current_mode
    current_mode = "navigation"
    print("Navigation start trigger received.")
    return "Navigation started", 200

@app.route('/trash', methods=['GET'])
def trigger_trash():
    global current_mode
    current_mode = "trash"
    print("Trash mode trigger received.")
    return "Trash mode triggered", 200

@app.route('/status', methods=['GET'])
def status():
    return jsonify({"mode": current_mode}), 200

# Video streaming route
def gen_frames():
    cap = cv2.VideoCapture(0)  # Use the Pi's camera device (adjust if needed)
    while True:
        success, frame = cap.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    cap.release()

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def run_server():
    app.run(host='0.0.0.0', port=5000)

def main():
    global current_mode
    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()
    
    print("Waiting for navigation start trigger from the Mac...")
    while current_mode != "navigation":
        time.sleep(1)
    
    print("Launching normal navigation (robot_main.py)...")
    nav_proc = subprocess.Popen(["python3", "/home/gama/lidar/robot_main.py"])
    
    while True:
        time.sleep(1)
        if current_mode == "trash":
            print("Trash mode triggered! Stopping normal navigation...")
            nav_proc.terminate()
            nav_proc.wait()
            print("Launching trash mode (trash_mode.py)...")
            subprocess.run(["python3", "/home/gama/lidar/trash_mode.py"])
            # After trash mode completes, reset mode back to navigation.
            current_mode = "navigation"
            print("Trash mode complete. Resuming normal navigation...")
            nav_proc = subprocess.Popen(["python3", "/home/gama/lidar/robot_main.py"])

if __name__ == '__main__':
    main()
