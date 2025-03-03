import asyncio
import math
import sys
import pygame
import serial

from lds_driver import LDSDriver  # your custom driver module

# Global configuration
NUM_MEASUREMENTS = 360  # Full scan measurements count
lidar_distances = [0] * NUM_MEASUREMENTS  # Global list (in pixels) for current measurements

# Visualization parameters
SCALING_FACTOR = 200  # 1 meter = 200 pixels
ROBOT_WIDTH_M = 0.4   # 40 cm width
ROBOT_LENGTH_M = 0.5  # 50 cm length
ROBOT_WIDTH_PIX = ROBOT_WIDTH_M * SCALING_FACTOR   # 80 pixels
ROBOT_LENGTH_PIX = ROBOT_LENGTH_M * SCALING_FACTOR   # 100 pixels

# Serial port configuration for communicating with Arduino
SERIAL_PORT = '/dev/ttyUSB0'  # Update this port as needed for your Raspberry Pi
SERIAL_BAUD = 9600            # Must match the Arduino's Serial.begin() setting

# Open the serial connection
ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)

# Global variable to track the last command sent
last_command = None

def generate_dot_positions(num_measurements):
    positions = []
    for i in range(num_measurements):
        angle_rad = math.radians(i)
        positions.append((math.cos(angle_rad), math.sin(angle_rad)))
    return positions

def init_visualization():
    pygame.init()
    screen = pygame.display.set_mode((800, 800))
    pygame.display.set_caption("LDS‑01 Dot Visualization with Robot Mask")
    return screen

def filterDistances(new_data, old_data, threshold=1500):
    filtered = []
    for new_val, old_val in zip(new_data, old_data):
        if new_val > threshold:
            filtered.append(old_val)
        else:
            filtered.append(new_val)
    return filtered

def send_robot_command(command):
    global last_command, ser
    # Only send the command if it's different from the last command
    if command != last_command:
        try:
            ser.write((command + '\n').encode('utf-8'))
            print("Sending command:", command)
            last_command = command
        except Exception as e:
            print("Error sending command:", e)

def decide_command(lidar_data):
    """
    Determine the command based on current lidar readings.
    - If no valid lidar data, return "STOP".
    - If forward clearance exceeds the threshold, return "FRONT".
    - Otherwise, if an obstacle is in front, choose "LEFT" or "RIGHT"
      based on which side has the most clearance.
    """
    if all(d == 0 for d in lidar_data):
        return "STOP"  # No valid data yet, so stop

    # Calculate clearance threshold:
    # The front edge is ROBOT_LENGTH_M/2 from the center.
    # For a 20 cm gap from the front edge, we need clearance = ROBOT_LENGTH_M/2 + 0.2.
    obstacle_threshold = ((ROBOT_LENGTH_M / 2) + 0.2) * SCALING_FACTOR  # e.g., (0.25+0.2)*200 = 90 pixels

    # Define a forward window (e.g., near 0°)
    forward_indices = list(range(350, 360)) + list(range(0, 11))
    forward_measurements = [lidar_data[i] for i in forward_indices]
    forward_distance = sum(forward_measurements) / len(forward_measurements)

    if forward_distance >= obstacle_threshold:
        return "FRONT"  # Clear path ahead
    else:
        best_distance = -1
        best_angle = None
        # Check only the front half (-90° to +90°)
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
            distances_m = await driver.poll()  # returns distances in meters
            new_pixels = [d * SCALING_FACTOR for d in distances_m]
            if any(lidar_distances):
                lidar_distances = filterDistances(new_pixels, lidar_distances, threshold=1500)
            else:
                lidar_distances = new_pixels
        except Exception as e:
            print("Error polling driver:", e)
        await asyncio.sleep(0.01)

async def control_robot():
    global lidar_distances
    # Wait 2 seconds for lidar to initialize
    await asyncio.sleep(2)
    while True:
        command = decide_command(lidar_distances)
        send_robot_command(command)
        await asyncio.sleep(0.05)  # 20 Hz control loop

async def visualization_loop():
    screen = init_visualization()
    clock = pygame.time.Clock()
    dot_positions = generate_dot_positions(NUM_MEASUREMENTS)
    center = (400, 400)
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        screen.fill((255, 255, 255))
        for i in range(NUM_MEASUREMENTS):
            dist = lidar_distances[i]
            dot_x = center[0] + dot_positions[i][0] * dist
            dot_y = center[1] + dot_positions[i][1] * dist
            dx = dot_x - center[0]
            dy = dot_y - center[1]
            if not (-ROBOT_WIDTH_PIX/2 <= dx <= ROBOT_WIDTH_PIX/2 and -ROBOT_LENGTH_PIX/2 <= dy <= ROBOT_LENGTH_PIX/2):
                pygame.draw.circle(screen, (0, 0, 0), (int(dot_x), int(dot_y)), 3)
        robot_rect = pygame.Rect(center[0]-ROBOT_WIDTH_PIX/2, center[1]-ROBOT_LENGTH_PIX/2,
                                 ROBOT_WIDTH_PIX, ROBOT_LENGTH_PIX)
        pygame.draw.rect(screen, (200,200,200), robot_rect)
        pygame.draw.rect(screen, (0,0,0), robot_rect,2)
        pygame.display.flip()
        clock.tick(30)
        await asyncio.sleep(0)
    pygame.quit()
    sys.exit()

async def main():
    # Update the port to your Raspberry Pi's appropriate port (e.g., /dev/ttyUSB0)
    driver = LDSDriver(port='/dev/ttyUSB0', baudrate=230400)
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
        ser.close()

if __name__ == '__main__':
    asyncio.run(main())
