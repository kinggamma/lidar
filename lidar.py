import asyncio
import math
import sys

import pygame

from lds_driver import LDSDriver  # your custom driver module

# Global configuration
NUM_MEASUREMENTS = 360  # Full scan measurements count
lidar_distances = [0] * NUM_MEASUREMENTS  # Global list (in pixels) for current measurements

# Visualization parameters
SCALING_FACTOR = 200  # 1 meter equals 200 pixels
ROBOT_WIDTH_M = 0.4  # 40 cm width
ROBOT_LENGTH_M = 0.5  # 50 cm length
ROBOT_WIDTH_PIX = ROBOT_WIDTH_M * SCALING_FACTOR  # 80 pixels
ROBOT_LENGTH_PIX = ROBOT_LENGTH_M * SCALING_FACTOR  # 100 pixels


def generate_dot_positions(num_measurements):
    """
    Generate unit vectors for each measurement angle.
    These unit vectors will later be scaled by the measured distance.
    """
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
    """
    Filters new lidar data (in pixel units) by comparing each value with a threshold.
    If a new value exceeds the threshold (indicating an outlier), the old value is retained.
    """
    filtered = []
    for new_val, old_val in zip(new_data, old_data):
        if new_val > threshold:
            filtered.append(old_val)
        else:
            filtered.append(new_val)
    return filtered


async def update_lidar(driver):
    """
    Continuously polls the driver for new data, converts distances from meters to pixels,
    and applies the filter.
    """
    global lidar_distances
    while True:
        try:
            # Get a list of 360 distances (in meters) from the driver.
            distances_m = await driver.poll()
            # Convert distances to pixel units using the scaling factor.
            new_pixels = [d * SCALING_FACTOR for d in distances_m]
            if any(lidar_distances):
                lidar_distances = filterDistances(new_pixels, lidar_distances, threshold=1500)
            else:
                lidar_distances = new_pixels
        except Exception as e:
            print("Error polling driver:", e)
        await asyncio.sleep(0.01)


async def visualization_loop():
    """
    Pygame loop that draws each measurement as a dot.
    Each dot is placed at (center + unit_vector * filtered_distance).
    Dots that fall within the robot's rectangular footprint are not drawn.
    The robot is visualized as a filled rectangle.
    """
    screen = init_visualization()
    clock = pygame.time.Clock()
    dot_positions = generate_dot_positions(NUM_MEASUREMENTS)
    center = (400, 400)
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Set background to white.
        screen.fill((255, 255, 255))
        for i in range(NUM_MEASUREMENTS):
            dist = lidar_distances[i]
            # Compute dot position relative to the center.
            dot_x = center[0] + dot_positions[i][0] * dist
            dot_y = center[1] + dot_positions[i][1] * dist
            # Calculate position relative to the center to check against robot footprint.
            dx = dot_x - center[0]
            dy = dot_y - center[1]
            # Only draw the dot if it's outside the robot's rectangular footprint.
            if not (
                    -ROBOT_WIDTH_PIX / 2 <= dx <= ROBOT_WIDTH_PIX / 2 and -ROBOT_LENGTH_PIX / 2 <= dy <= ROBOT_LENGTH_PIX / 2):
                pygame.draw.circle(screen, (0, 0, 0), (int(dot_x), int(dot_y)), 3)
        # Draw the robot as a filled rectangle in a light gray with a black border.
        robot_rect = pygame.Rect(center[0] - ROBOT_WIDTH_PIX / 2, center[1] - ROBOT_LENGTH_PIX / 2,
                                 ROBOT_WIDTH_PIX, ROBOT_LENGTH_PIX)
        pygame.draw.rect(screen, (200, 200, 200), robot_rect)
        pygame.draw.rect(screen, (0, 0, 0), robot_rect, 2)
        pygame.display.flip()
        clock.tick(30)
        await asyncio.sleep(0)
    pygame.quit()
    sys.exit()


async def main():
    # Create the driver instance using your specified port.
    driver = LDSDriver(port='/dev/tty.usbserial-0001', baudrate=230400)
    await driver.connect()
    try:
        await asyncio.gather(
            update_lidar(driver),
            visualization_loop()
        )
    except asyncio.CancelledError:
        pass
    finally:
        await driver.disconnect()


if __name__ == '__main__':
    asyncio.run(main())
