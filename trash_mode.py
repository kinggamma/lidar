#!/usr/bin/env python3
import asyncio
import math
import sys
import serial
import time
from lds_driver import LDSDriver

NUM_MEASUREMENTS = 360
lidar_distances = [0] * NUM_MEASUREMENTS  # in pixels
SCALING_FACTOR = 200      # 1 meter = 200 pixels
ROBOT_LENGTH_M = 0.5      # 50 cm robot length

SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_BAUD = 9600
ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
last_command = None

def send_robot_command(command):
    global last_command, ser
    if command != last_command:
        try:
            ser.write((command + "\n").encode("utf-8"))
            print("Sending command:", command)
            last_command = command
        except Exception as e:
            print("Error sending command:", e)

def adjusted_reading_for_index(i, reading):
    angle = i if i <= 180 else i - 360
    if -30 <= angle <= 30:
        offset = 30   # Front offset (15cm)
    elif 30 < angle <= 90 or -90 <= angle < -30:
        offset = 38   # Left/Right offset (19cm)
    else:
        offset = 76   # Back offset (38cm)
    adj = reading - offset
    return adj if adj > 0 else 0

def get_adjusted_forward(lidar_data):
    indices = [i for i in range(360) if -30 <= (i if i<=180 else i-360) <= 30]
    if indices:
        values = [adjusted_reading_for_index(i, lidar_data[i]) for i in indices]
        return sum(values) / len(values)
    return 0

def get_adjusted_back(lidar_data):
    indices = [i for i in range(360) if abs(i if i<=180 else i-360) >= 150]
    if indices:
        values = [adjusted_reading_for_index(i, lidar_data[i]) for i in indices]
        return sum(values) / len(values)
    return 0

async def update_lidar(driver):
    global lidar_distances
    while True:
        try:
            distances_m = await driver.poll()
            new_pixels = [d * SCALING_FACTOR for d in distances_m]
            lidar_distances = new_pixels
        except Exception as e:
            print("Error polling lidar:", e)
        await asyncio.sleep(0.01)

async def trash_mode():
    driver = LDSDriver(port=SERIAL_PORT, baudrate=230400)
    await driver.connect()
    asyncio.create_task(update_lidar(driver))
    await asyncio.sleep(2)

    send_robot_command("STOP")
    await asyncio.sleep(1)
    
    baseline_back = get_adjusted_back(lidar_distances)
    attempts = 0
    while baseline_back == 0.0 and attempts < 20:
        print("Baseline back reading is 0.0, waiting for valid lidar data...")
        await asyncio.sleep(0.5)
        baseline_back = get_adjusted_back(lidar_distances)
        attempts += 1
    if baseline_back == 0.0:
        print("Could not get a valid baseline. Aborting trash mode.")
        send_robot_command("STOP")
        await driver.disconnect()
        ser.close()
        sys.exit(1)
    print("Baseline adjusted back distance:", baseline_back)
    
    # Spin 180° so that the back now faces forward.
    send_robot_command("SPIN")
    print("Starting 180-degree spin...")
    spin_start_time = time.time()
    MIN_SPIN_TIME = 2  # Minimum spin time in seconds
    while True:
        current_fwd = get_adjusted_forward(lidar_distances)
        elapsed = time.time() - spin_start_time
        print("Current adjusted forward reading:", current_fwd, "Elapsed:", elapsed)
        if elapsed >= MIN_SPIN_TIME and abs(current_fwd - baseline_back) < 20:
            break
        await asyncio.sleep(0.1)
    send_robot_command("STOP")
    print("Spin complete (180° achieved).")
    
    # Instead of waiting for ultrasonic trigger (which is now handled by Arduino),
    # back up for a fixed period while the Arduino monitors the ultrasonic sensor.
    send_robot_command("BACK")
    print("Backing up for 3 seconds...")
    await asyncio.sleep(3)
    send_robot_command("STOP")
    print("Backup complete (assumed ultrasonic trigger).")
    
    # Activate dumping mechanism.
    send_robot_command("DUMP")
    print("Dumping mechanism activated.")
    await asyncio.sleep(2)
    
    send_robot_command("STOP")
    print("Trash mode complete.")
    await driver.disconnect()
    ser.close()
    sys.exit()

if __name__ == "__main__":
    try:
        asyncio.run(trash_mode())
    except SystemExit:
        pass
    except Exception as e:
        print("Unhandled exception:", e)
