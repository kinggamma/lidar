#!/usr/bin/env python3
import asyncio
import math
import sys
import pygame
import serial
from lds_driver import LDSDriver  # Your custom lidar driver module

# Parameters for lidar
NUM_MEASUREMENTS = 360
lidar_distances = [0] * NUM_MEASUREMENTS  # in pixels
SCALING_FACTOR = 200       # 1 meter = 200 pixels
ROBOT_WIDTH_M = 0.4        # 40 cm
ROBOT_LENGTH_M = 0.5       # 50 cm
ROBOT_WIDTH_PIX = ROBOT_WIDTH_M * SCALING_FACTOR
ROBOT_LENGTH_PIX = ROBOT_LENGTH_M * SCALING_FACTOR

# Define ports (adjust as needed)
LIDAR_PORT = "/dev/ttyUSB0"
ARDUINO_PORT = "/dev/ttyACM0"
LIDAR_BAUD = 230400
ARDUINO_BAUD = 9600

# Create a serial connection for Arduino commands.
arduino_ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
last_command = None

def generate_dot_positions(num_measurements):
    positions = []
    for i in range(num_measurements):
        angle_rad = math.radians(i)  # Assuming 0° is forward.
        positions.append((math.cos(angle_rad), math.sin(angle_rad)))
    return positions

def init_visualization():
    pygame.init()
    screen = pygame.display.set_mode((800, 800))
    pygame.display.set_caption("Robot Navigation")
    return screen

def filterDistances(new_data, old_data, threshold=1500):
    filtered = []
    for new_val, old_val in zip(new_data, old_data):
        filtered.append(old_val if new_val > threshold else new_val)
    return filtered

def send_robot_command(command):
    global last_command, arduino_ser
    if command != last_command:
        try:
            arduino_ser.write((command + "\n").encode("utf-8"))
            print("Sending command:", command)
            last_command = command
        except Exception as e:
            print("Error sending command:", e)

def decide_command(lidar_data):
    if all(d == 0 for d in lidar_data):
        return "STOP"
    # Use a clearance threshold based on the front region.
    obstacle_threshold = ((ROBOT_LENGTH_M / 2) + 0.2) * SCALING_FACTOR
    forward_indices = list(range(350, 360)) + list(range(0, 11))
    fwd = [lidar_data[i] for i in forward_indices]
    if sum(fwd) / len(fwd) >= obstacle_threshold:
        return "FRONT"
    else:
        best_distance = -1
        best_angle = None
        for i in range(360):
            angle = i if i <= 180 else i - 360
            if -90 <= angle <= 90:
                if lidar_data[i] > best_distance:
                    best_distance = lidar_data[i]
                    best_angle = angle
        if best_angle is None or abs(best_angle) < 10:
            return "FRONT"
        elif best_angle > 0:
            return "LEFT"
        else:
            return "RIGHT"

async def update_lidar(driver):
    global lidar_distances
    while True:
        try:
            distances_m = await driver.poll()  # distances in meters from the lidar
            new_pixels = [d * SCALING_FACTOR for d in distances_m]
            # Use filtering only if there are any nonzero values
            if any(lidar_distances):
                lidar_distances = filterDistances(new_pixels, lidar_distances)
            else:
                lidar_distances = new_pixels
        except Exception as e:
            print("Error polling lidar:", e)
        await asyncio.sleep(0.01)

async def control_robot():
    global lidar_distances
    await asyncio.sleep(2)  # Allow the sensor time to initialize.
    while True:
        cmd = decide_command(lidar_distances)
        send_robot_command(cmd)
        await asyncio.sleep(0.05)

async def visualization_loop():
    screen = init_visualization()
    clock = pygame.time.Clock()
    dots = generate_dot_positions(NUM_MEASUREMENTS)
    center = (400, 400)
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        screen.fill((255, 255, 255))
        for i in range(NUM_MEASUREMENTS):
            dist = lidar_distances[i]
            x = center[0] + dots[i][0] * dist
            y = center[1] + dots[i][1] * dist
            # Only draw points outside the robot’s "ignored" area.
            if not (-ROBOT_WIDTH_PIX/2 <= x - center[0] <= ROBOT_WIDTH_PIX/2 and
                    -ROBOT_LENGTH_PIX/2 <= y - center[1] <= ROBOT_LENGTH_PIX/2):
                pygame.draw.circle(screen, (0, 0, 0), (int(x), int(y)), 3)
        # Draw a rectangle representing the robot's physical body (ignored zone).
        robot_body_rect = pygame.Rect(center[0] - 38, center[1] - 30, 76, 106)
        pygame.draw.rect(screen, (200, 200, 200), robot_body_rect, 2)
        pygame.display.flip()
        clock.tick(30)
        await asyncio.sleep(0)
    pygame.quit()
    return

async def main():
    driver = LDSDriver(port=LIDAR_PORT, baudrate=LIDAR_BAUD)
    await driver.connect()
    try:
        await asyncio.gather(
            update_lidar(driver),
            control_robot(),
            visualization_loop()
        )
    except asyncio.CancelledError:
        pass
    finally:
        await driver.disconnect()
        arduino_ser.close()

if __name__ == "__main__":
    asyncio.run(main())
