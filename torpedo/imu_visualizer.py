import pygame
import math
import time
try:
    from mpu6050 import mpu6050
except ImportError:
    print("The 'mpu6050-rpi' library is not installed. Please install it using 'pip install mpu6050-rpi'")
    exit()


# Screen dimensions
WIDTH, HEIGHT = 800, 600

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
SKY_BLUE = (135, 206, 235)
GROUND_BROWN = (139, 69, 19)

# Simple low-pass filter factor
FILTER_ALPHA = 0.1

class IMUVisualizer:
    def __init__(self):
        """Initialize the visualizer, Pygame, and the IMU."""
        self.pitch = 0.0
        self.roll = 0.0
        
        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("IMU Attitude Visualizer")
        self.font = pygame.font.Font(None, 36)
        self.clock = pygame.time.Clock()

        # Initialize MPU6050
        try:
            self.imu = mpu6050(0x68)
            print("MPU6050 found at 0x68")
        except:
            try:
                self.imu = mpu6050(0x69)
                print("MPU6050 found at 0x69")
            except Exception as e:
                print(f"Error initializing MPU6050: {e}")
                print("Please check the I2C connection and address.")
                self.imu = None

    def get_imu_data(self):
        """Get and process data from the IMU."""
        if not self.imu:
            return False
            
        try:
            accel_data = self.imu.get_accel_data()
            
            # Calculate roll and pitch from accelerometer data
            # These calculations are sensitive to acceleration, so they are best for static or slow-moving orientation
            acc_x = accel_data['x']
            acc_y = accel_data['y']
            acc_z = accel_data['z']

            # Roll calculation
            roll_rad = math.atan2(acc_y, acc_z)
            current_roll = math.degrees(roll_rad)

            # Pitch calculation
            pitch_rad = math.atan2(-acc_x, math.sqrt(acc_y * acc_y + acc_z * acc_z))
            current_pitch = math.degrees(pitch_rad)

            # Apply a simple low-pass filter to smooth the values
            self.roll = FILTER_ALPHA * current_roll + (1 - FILTER_ALPHA) * self.roll
            self.pitch = FILTER_ALPHA * current_pitch + (1 - FILTER_ALPHA) * self.pitch
            
            return True
        except Exception as e:
            print(f"Failed to read from MPU6050: {e}")
            return False

    def draw(self):
        """Draw the attitude indicator on the screen."""
        self.screen.fill(SKY_BLUE)

        # Define the corners of a large polygon that will represent the ground.
        # It's much larger than the screen to ensure it fills the space after rotation.
        poly_size = max(WIDTH, HEIGHT) * 2
        points = [
            (-poly_size, 0),
            (poly_size, 0),
            (poly_size, poly_size),
            (-poly_size, poly_size)
        ]

        # Convert roll to radians for math functions
        roll_rad = math.radians(self.roll)
        cos_roll = math.cos(roll_rad)
        sin_roll = math.sin(roll_rad)

        # The center of the screen is the pivot point for rotation and pitch offset
        center_x = WIDTH // 2
        # Pitch moves the horizon line up and down
        center_y = HEIGHT // 2 + self.pitch * 5

        transformed_points = []
        for x, y in points:
            # Rotate the point around the origin (0,0)
            x_rot = x * cos_roll - y * sin_roll
            y_rot = x * sin_roll + y * cos_roll
            
            # Translate the point to its final position on the screen
            final_x = center_x + x_rot
            final_y = center_y + y_rot
            transformed_points.append((final_x, final_y))

        # Draw the transformed polygon representing the ground
        pygame.draw.polygon(self.screen, GROUND_BROWN, transformed_points)

        # Draw a static indicator for the vehicle
        pygame.draw.line(self.screen, BLACK, (WIDTH // 2 - 40, HEIGHT // 2), (WIDTH // 2 - 20, HEIGHT // 2), 3)
        pygame.draw.line(self.screen, BLACK, (WIDTH // 2 + 40, HEIGHT // 2), (WIDTH // 2 + 20, HEIGHT // 2), 3)
        pygame.draw.line(self.screen, BLACK, (WIDTH // 2, HEIGHT // 2 - 20), (WIDTH // 2, HEIGHT // 2 + 20), 3)
        
        # Update the window caption with the current values
        caption = f"Pitch: {self.pitch:.2f}°, Roll: {self.roll:.2f}°"
        pygame.display.set_caption(caption)

        pygame.display.flip()

    def run(self):
        """Main loop of the application."""
        if not self.imu:
            print("Cannot run without an IMU.")
            # Keep window open to show error message
            running = True
            while running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                self.screen.fill(WHITE)
                error_text = self.font.render("IMU not found. Check connection.", True, BLACK)
                self.screen.blit(error_text, (50, HEIGHT // 2 - 20))
                pygame.display.flip()
            pygame.quit()
            return

        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            if self.get_imu_data():
                self.draw()
            
            self.clock.tick(60) # Limit to 60 FPS

        pygame.quit()

if __name__ == '__main__':
    visualizer = IMUVisualizer()
    visualizer.run()
