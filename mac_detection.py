import cv2
from ultralytics import YOLO
import time
import torch
import requests

# --- Configuration ---
pi_ip = "10.5.6.6"  # Pi's IP address
start_url = f"http://{pi_ip}:5000/start"
trash_url = f"http://{pi_ip}:5000/trash"
status_url = f"http://{pi_ip}:5000/status"
stream_url = f"http://{pi_ip}:5000/video_feed"  # Note: now served by Flask

# Wait for user input to start.
start_input = input("Press 'y' to send start request to the Pi and begin detection: ")
if start_input.lower() == 'y':
    try:
        response = requests.get(start_url)
        print("Response from Pi:", response.text)
    except Exception as e:
        print("Error sending start request:", e)
        exit()
else:
    print("Start command not received. Exiting.")
    exit()

# Device selection for YOLO.
device = torch.device("mps" if torch.backends.mps.is_available() else "cpu")
print("Using device:", device)

# Load YOLOv8 model.
model = YOLO('/Users/ninasgama/runs/detect/train10/weights/best.pt')
model.to(device)

cap = cv2.VideoCapture(stream_url)
if not cap.isOpened():
    print("Error: Could not open video stream from the Pi.")
    exit()

CONSECUTIVE_REQUIRED = 3
detection_counter = 0
FRAME_WIDTH, FRAME_HEIGHT = 640, 640
CENTER_X, CENTER_Y = FRAME_WIDTH // 2, FRAME_HEIGHT // 2
TOLERANCE = 50
prev_time = time.time()
in_trash_mode = False

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        continue

    resized_frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
    current_time = time.time()
    fps = 1 / (current_time - prev_time) if (current_time - prev_time) > 0 else 0
    prev_time = current_time

    results = model(resized_frame)
    annotated_frame = results[0].plot()

    # Check detections for a trash bin near the center.
    trash_detected = False
    if results[0].boxes is not None:
        for box in results[0].boxes:
            coords = box.xyxy[0].tolist()
            x1, y1, x2, y2 = coords
            box_center_x = (x1 + x2) / 2
            box_center_y = (y1 + y2) / 2
            if abs(box_center_x - CENTER_X) < TOLERANCE and abs(box_center_y - CENTER_Y) < TOLERANCE:
                trash_detected = True
                break

    if trash_detected and not in_trash_mode:
        detection_counter += 1
    else:
        detection_counter = 0

    if detection_counter >= CONSECUTIVE_REQUIRED and not in_trash_mode:
        try:
            response = requests.get(trash_url)
            print("Sent trash trigger to Pi; response:", response.text)
            in_trash_mode = True
        except Exception as e:
            print("Error sending trash trigger:", e)

    if in_trash_mode:
        print("Polling Pi status every 5 seconds...")
        try:
            while True:
                status_response = requests.get(status_url, timeout=10)
                mode = status_response.json().get("mode", "")
                print("Current Pi mode:", mode)
                if mode == "navigation":
                    in_trash_mode = False
                    detection_counter = 0
                    print("Trash mode complete. Resuming detection.")
                    break
                time.sleep(5)
        except Exception as e:
            print("Error polling status:", e)
            print("Assuming connection lost; exiting detection loop.")
            break

    cv2.putText(annotated_frame, f"FPS: {fps:.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("YOLOv8 Detection", annotated_frame)
    
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
