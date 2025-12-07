#!/usr/bin/env python3
"""
Main control script for the torpedo, combining dual camera feeds, IMU visualization,
remote control, and thruster management.
"""

import cv2
import numpy as np
import time
import socket
import json
import math
import smbus
from collections import defaultdict
import threading

# Suppress Pygame's welcome message
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame

# --- Library Import Error Handling ---
try:
    from picamera2 import Picamera2
except ImportError:
    print("Error: picamera2 library not found. Please install with 'pip install picamera2'")
    exit()

try:
    from mpu6050 import mpu6050
except ImportError:
    print("Error: mpu6050-rpi library not found. Please install with 'pip install mpu6050-rpi'")
    exit()

# ============================================================================
# --- CONFIGURATION ---
# ============================================================================

# Camera Settings
PREVIEW_WIDTH = 640
PREVIEW_HEIGHT = 480
WINDOW_NAME = "Torpedo Control Center (Press 'q' to quit)"

# Network Settings
UDP_IP = "0.0.0.0"
UDP_PORT = 5005

# PCA9685 & Thruster Settings
PWM_FREQ = 50
IDLE_PULSE = 1500
FORWARD_PULSE = 2000
BACKWARD_PULSE = 1000

THRUSTER_CHANNELS = {
    'thruster1': 2,
    'thruster2': 3,
    'thruster3': 4,
    'thruster4': 5,
    'thruster5': 6,
}

THRUSTER_DIRECTIONS = {
    'thruster1': 1.0,
    'thruster2': -1.0,
    'thruster3': 1.0,
    'thruster4': -1.0,
    'thruster5': 1.0,
}

# IMU Settings
IMU_ADDRESS_1 = 0x68
IMU_ADDRESS_2 = 0x69
FILTER_ALPHA = 0.1 # Simple low-pass filter for smoothing IMU data

# Display Settings
PITCH_REFERENCE_ANGLE = 9.0 # The target pitch angle to show as a reference line
PITCH_REF_COLOR = (255, 100, 0) # Blue/Cyan for reference line
HORIZON_COLOR = (0, 255, 0)  # Green
TEXT_COLOR = (255, 255, 255) # White
FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.6
FONT_THICKNESS = 1


# ============================================================================
# --- HARDWARE CLASSES (PCA9685, Thruster, SysfsPWM) ---
# ============================================================================

class SysfsPWM:
    """通过 sysfs 控制 PIO-PWM 的类。"""
    def __init__(self, pwm_chip_path="/sys/class/pwm/pwmchip0", channel=0):
        self.chip_path = pwm_chip_path
        self.channel = channel
        self.dir_path = os.path.join(self.chip_path, f"pwm{self.channel}")
        self.export_path = os.path.join(self.chip_path, "export")
        self.unexport_path = os.path.join(self.chip_path, "unexport")
        self.period_path = os.path.join(self.dir_path, "period")
        self.duty_cycle_path = os.path.join(self.dir_path, "duty_cycle")
        self.enable_path = os.path.join(self.dir_path, "enable")
        self.frequency = 20000  # 默认频率

        if not os.path.exists(self.chip_path):
            raise IOError(f"错误：找不到 PWM 控制器 '{self.chip_path}'。请检查 dtoverlay 配置。")

    def _write_to_file(self, path, value):
        """以 root 权限向文件写入值。"""
        # 使用 os.system 是因为需要 sudo 权限
        command = f"echo {value} | sudo tee {path} > /dev/null"
        result = os.system(command)
        if result != 0:
            print(f"警告: 执行 'echo {value} | sudo tee {path}' 失败。")

    def setup(self, frequency, duty_cycle):
        """设置并启用 PWM。"""
        self.frequency = frequency
        period_ns = int(1_000_000_000 / self.frequency)
        duty_cycle_ns = int(period_ns * (duty_cycle / 100.0))

        if not os.path.exists(self.dir_path):
            self._write_to_file(self.export_path, self.channel)
            time.sleep(0.5) # 等待 sysfs 创建目录

        if not os.path.exists(self.dir_path):
            raise IOError(f"错误：导出 PWM 通道后，目录 '{self.dir_path}' 未创建。")

        self._write_to_file(self.enable_path, 0) # 禁用以应用新设置
        self._write_to_file(self.period_path, period_ns)
        self._write_to_file(self.duty_cycle_path, duty_cycle_ns)
        self._write_to_file(self.enable_path, 1)

    def stop(self):
        """停止并清理 PWM。"""
        if os.path.exists(self.dir_path):
            self._write_to_file(self.enable_path, 0)
            self._write_to_file(self.unexport_path, self.channel)

class PCA9685:
    __MODE1 = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    
    def __init__(self, address=0x40, bus_num=1):
        self.bus = smbus.SMBus(bus_num)
        self.address = address
        self.write(self.__MODE1, 0x00)

    def write(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)

    def read(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def setPWMFreq(self, freq):
        self.freq = freq
        prescaleval = 25000000.0 / 4096.0 / float(freq) - 1.0
        prescale = math.floor(prescaleval + 0.5)
        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10
        self.write(self.__MODE1, newmode)
        self.write(self.__PRESCALE, int(prescale))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def setPWM(self, channel, on, off):
        self.write(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(self.__LED0_ON_L + 4 * channel + 1, on >> 8)
        self.write(self.__LED0_ON_L + 4 * channel + 2, off & 0xFF)
        self.write(self.__LED0_ON_L + 4 * channel + 3, off >> 8)

    def setServoPulse(self, channel, pulse_us):
        period_us = 1_000_000.0 / self.freq
        ticks = int(round((pulse_us / period_us) * 4096.0))
        self.setPWM(channel, 0, ticks)

class Thruster:
    def __init__(self, pwm_driver, channel):
        self.pwm = pwm_driver
        self.channel = channel

    def set_speed(self, pulse_width):
        self.pwm.setServoPulse(self.channel, int(pulse_width))

class SysfsPWM:
    """通过 sysfs 控制 PIO-PWM 的类。"""
    def __init__(self, pwm_chip_path="/sys/class/pwm/pwmchip0", channel=0):
        self.chip_path = pwm_chip_path
        self.channel = channel
        self.dir_path = os.path.join(self.chip_path, f"pwm{self.channel}")
        self.export_path = os.path.join(self.chip_path, "export")
        self.unexport_path = os.path.join(self.chip_path, "unexport")
        self.period_path = os.path.join(self.dir_path, "period")
        self.duty_cycle_path = os.path.join(self.dir_path, "duty_cycle")
        self.enable_path = os.path.join(self.dir_path, "enable")
        self.frequency = 20000  # 默认频率

        if not os.path.exists(self.chip_path):
            raise IOError(f"错误：找不到 PWM 控制器 '{self.chip_path}'。请检查 dtoverlay 配置。")

    def _write_to_file(self, path, value):
        """以 root 权限向文件写入值。"""
        # 使用 os.system 是因为需要 sudo 权限
        command = f"echo {value} | sudo tee {path} > /dev/null"
        result = os.system(command)
        if result != 0:
            print(f"警告: 执行 'echo {value} | sudo tee {path}' 失败。")

    def setup(self, frequency, duty_cycle):
        """设置并启用 PWM。"""
        self.frequency = frequency
        period_ns = int(1_000_000_000 / self.frequency)
        duty_cycle_ns = int(period_ns * (duty_cycle / 100.0))

        if not os.path.exists(self.dir_path):
            self._write_to_file(self.export_path, self.channel)
            time.sleep(0.5) # 等待 sysfs 创建目录

        if not os.path.exists(self.dir_path):
            raise IOError(f"错误：导出 PWM 通道后，目录 '{self.dir_path}' 未创建。")

        self._write_to_file(self.enable_path, 0) # 禁用以应用新设置
        self._write_to_file(self.period_path, period_ns)
        self._write_to_file(self.duty_cycle_path, duty_cycle_ns)
        self._write_to_file(self.enable_path, 1)

    def stop(self):
        """停止并清理 PWM。"""
        if os.path.exists(self.dir_path):
            self._write_to_file(self.enable_path, 0)
            self._write_to_file(self.unexport_path, self.channel)

# ============================================================================
# --- GLOBAL STATE & HELPER FUNCTIONS ---
# ============================================================================

# Shared state between threads, with locks for safety
shared_state = {
    'joystick_command': defaultdict(float),
    'imu_data': {'pitch': 0.0, 'roll': 0.0},
    'running': True,
    'duty_cycle': None,
}

def setup_hardware():
    """Initializes cameras, IMU, and thrusters."""
    # Cameras
    cam0, cam1 = None, None
    try:
        cameras_found = Picamera2.global_camera_info()
        if len(cameras_found) < 2:
            print("Warning: Found fewer than 2 cameras. Video feed will be disabled.")
        if len(cameras_found) > 0:
            cam0 = Picamera2(0)
            config = cam0.create_preview_configuration(main={"size": (PREVIEW_WIDTH, PREVIEW_HEIGHT)})
            cam0.configure(config)
            cam0.start()
            print("Camera 0 initialized.")
        if len(cameras_found) > 1:
            cam1 = Picamera2(1)
            config1 = cam1.create_preview_configuration(main={"size": (PREVIEW_WIDTH, PREVIEW_HEIGHT)})
            cam1.configure(config1)
            cam1.start()
            print("Camera 1 initialized.")
    except Exception as e:
        print(f"Error initializing cameras: {e}. Running in headless mode.")
        cam0, cam1 = None, None

    # IMU
    imu = None
    try:
        imu = mpu6050(IMU_ADDRESS_1)
        print(f"MPU6050 found at {hex(IMU_ADDRESS_1)}")
    except:
        try:
            imu = mpu6050(IMU_ADDRESS_2)
            print(f"MPU6050 found at {hex(IMU_ADDRESS_2)}")
        except Exception as e:
            print(f"Error initializing MPU6050: {e}")
    
    # Thrusters
    thrusters = None
    try:
        pwm = PCA9685(address=0x40, bus_num=1)
        pwm.setPWMFreq(PWM_FREQ)
        thrusters = {name: Thruster(pwm, ch) for name, ch in THRUSTER_CHANNELS.items()}
        print("Arming ESCs...")
        for thruster in thrusters.values():
            thruster.set_speed(IDLE_PULSE)
        time.sleep(2)
        print("ESCs armed.")
    except Exception as e:
        print(f"Error initializing PCA9685/Thrusters: {e}")

    # Aux PWM
    aux_pwm = None
    try:
        aux_pwm = SysfsPWM()
        print("Auxiliary PWM controller initialized.")
    except (IOError, FileNotFoundError) as e:
        print(e)
        print("Could not initialize auxiliary PWM, this feature will be disabled.")

    return cam0, cam1, imu, thrusters, aux_pwm

def get_pulse_from_thrust(thrust_value, direction):
    """Converts a thrust value (-1 to 1) to a pulse width."""
    thrust_value *= direction
    if thrust_value > 0:
        return int(IDLE_PULSE + thrust_value * (FORWARD_PULSE - IDLE_PULSE))
    else:
        return int(IDLE_PULSE + thrust_value * (IDLE_PULSE - BACKWARD_PULSE))

def draw_horizon(frame, pitch, roll, color, thickness, gap_width=100):
    """Draws a horizon line with a central gap on the frame."""
    h, w, _ = frame.shape
    center_x, center_y = w // 2, h // 2
    
    # Pitch moves the line up/down. Adjust multiplier for sensitivity.
    pitch_offset = pitch * 5
    
    # Roll rotates the line.
    roll_rad = math.radians(roll)
    cos_roll = math.cos(roll_rad)
    sin_roll = math.sin(roll_rad)
    
    # Define line segment lengths
    line_len = w  # A long line to ensure it crosses the frame
    gap_half = gap_width // 2

    # Calculate the 4 points for the two line segments
    # Left line
    x1_start = center_x - line_len
    y1_start = center_y - pitch_offset
    x1_end = center_x - gap_half
    y1_end = center_y - pitch_offset
    
    # Right line
    x2_start = center_x + gap_half
    y2_start = center_y - pitch_offset
    x2_end = center_x + line_len
    y2_end = center_y - pitch_offset

    # Rotate points around the center
    def rotate_point(px, py, cx, cy, s, c):
        return cx + (px - cx) * c - (py - cy) * s, cy + (px - cx) * s + (py - cy) * c

    lx1, ly1 = rotate_point(x1_start, y1_start, center_x, center_y - pitch_offset, sin_roll, cos_roll)
    lx2, ly2 = rotate_point(x1_end, y1_end, center_x, center_y - pitch_offset, sin_roll, cos_roll)
    rx1, ry1 = rotate_point(x2_start, y2_start, center_x, center_y - pitch_offset, sin_roll, cos_roll)
    rx2, ry2 = rotate_point(x2_end, y2_end, center_x, center_y - pitch_offset, sin_roll, cos_roll)

    cv2.line(frame, (int(lx1), int(ly1)), (int(lx2), int(ly2)), color, thickness)
    cv2.line(frame, (int(rx1), int(ry1)), (int(rx2), int(ry2)), color, thickness)

def draw_pitch_reference_line(frame, target_pitch, color, thickness, line_width=80):
    """Draws a horizontal reference line for the target pitch angle."""
    h, w, _ = frame.shape
    center_x, center_y = w // 2, h // 2

    # Pitch moves the line up/down. Use the same sensitivity as the horizon.
    pitch_offset = int(target_pitch * 5)
    y_pos = center_y - pitch_offset

    # Draw the horizontal line segment
    cv2.line(frame, (center_x - line_width // 2, y_pos), (center_x + line_width // 2, y_pos), color, thickness)


def draw_static_markers(frame, color, thickness):
    """Draws a static horizontal reference marker in the center of the frame."""
    h, w, _ = frame.shape
    center_x, center_y = w // 2, h // 2
    
    # Central circle
    cv2.circle(frame, (center_x, center_y), 5, color, -1)
    # Horizontal lines
    cv2.line(frame, (center_x - 50, center_y), (center_x - 10, center_y), color, thickness)
    cv2.line(frame, (center_x + 10, center_y), (center_x + 50, center_y), color, thickness)

def draw_info_overlay(frame, pitch, roll, command, thruster_pulses, duty_cycle):
    """Draws IMU, joystick, thruster, and PWM info on the frame."""
    h, w, _ = frame.shape
    
    # --- Left side (IMU & Joystick) ---
    y_pos_left = 30
    # IMU data
    cv2.putText(frame, f"Pitch: {pitch:.2f}", (10, y_pos_left), FONT, FONT_SCALE, TEXT_COLOR, FONT_THICKNESS)
    y_pos_left += 20
    cv2.putText(frame, f"Roll: {roll:.2f}", (10, y_pos_left), FONT, FONT_SCALE, TEXT_COLOR, FONT_THICKNESS)
    
    y_pos_left += 30
    # Joystick data
    cv2.putText(frame, "Joystick:", (10, y_pos_left), FONT, FONT_SCALE, TEXT_COLOR, FONT_THICKNESS)
    for key, val in command.items():
        y_pos_left += 20
        cv2.putText(frame, f"  {key}: {val:.2f}", (10, y_pos_left), FONT, FONT_SCALE, TEXT_COLOR, FONT_THICKNESS)

    # --- Right side (Thrusters, PWM) ---
    y_pos_right = 30
    right_margin = 250 # Adjust this margin based on text width
    
    # Thruster data
    cv2.putText(frame, "Thrusters (Pulse us):", (w - right_margin, y_pos_right), FONT, FONT_SCALE, TEXT_COLOR, FONT_THICKNESS)
    for name, pulse in thruster_pulses.items():
        y_pos_right += 20
        cv2.putText(frame, f"  {name}: {pulse}", (w - right_margin, y_pos_right), FONT, FONT_SCALE, TEXT_COLOR, FONT_THICKNESS)

    y_pos_right += 30
    # PWM Duty Cycle
    cv2.putText(frame, "PWM Duty Cycle:", (w - right_margin, y_pos_right), FONT, FONT_SCALE, TEXT_COLOR, FONT_THICKNESS)
    y_pos_right += 20
    duty_cycle_text = f"{duty_cycle:.1f}%" if duty_cycle is not None else "N/A"
    cv2.putText(frame, f"  {duty_cycle_text}", (w - right_margin, y_pos_right), FONT, FONT_SCALE, TEXT_COLOR, FONT_THICKNESS)


# ============================================================================
# --- THREADED WORKER FUNCTIONS ---
# ============================================================================

def joystick_and_thruster_thread(thrusters, aux_pwm):
    """Handles UDP joystick input and controls thrusters."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(1.0)
    last_buttons_value = -1

    print(f"Listening for joystick commands on UDP port {UDP_PORT}...")

    while shared_state['running']:
        try:
            data, _ = sock.recvfrom(1024)
            command = json.loads(data.decode())
            shared_state['joystick_command'].update(command)

            # Handle button presses for PWM
            if aux_pwm:
                current_buttons_value = command.get('buttons')
                if current_buttons_value is not None and current_buttons_value != last_buttons_value:
                    duty_cycle = None
                    if current_buttons_value == 0:
                        duty_cycle = 99.0
                    elif current_buttons_value == 16:
                        duty_cycle = 50.0
                    elif current_buttons_value == 32:
                        duty_cycle = 0.0
                    
                    if duty_cycle is not None:
                        aux_pwm.setup(frequency=20000, duty_cycle=duty_cycle)
                        shared_state['duty_cycle'] = duty_cycle
                    
                    last_buttons_value = current_buttons_value

        except socket.timeout:
            print("Joystick connection timed out. Stopping motors.")
            shared_state['joystick_command'].clear()
        except (json.JSONDecodeError, KeyError):
            print("Received malformed joystick data.")
            continue

        # Extract commands, defaulting to 0
        cmd = shared_state['joystick_command']
        forward = cmd.get('forward', 0.0)
        pitch = cmd.get('pitch', 0.0)
        yaw = cmd.get('yaw', 0.0)
        roll = cmd.get('roll', 0.0)
        roller = cmd.get('roller', 0.0)

        # --- Thruster Mixing Logic ---
        t1 = forward + pitch + yaw + roll
        t2 = forward + pitch - yaw - roll
        t3 = forward - pitch + yaw - roll
        t4 = forward - pitch - yaw + roll
        t5 = roller / 660.0 # Normalize roller

        final_thrusts = {
            'thruster1': max(-1.0, min(1.0, t1)),
            'thruster2': max(-1.0, min(1.0, t2)),
            'thruster3': max(-1.0, min(1.0, t3)),
            'thruster4': max(-1.0, min(1.0, t4)),
            'thruster5': max(-1.0, min(1.0, t5)),
        }

        # Set thruster speeds
        for name, thrust_val in final_thrusts.items():
            direction = THRUSTER_DIRECTIONS[name]
            pulse = get_pulse_from_thrust(thrust_val, direction)
            if thrusters:
                thrusters[name].set_speed(pulse)
        
        time.sleep(0.02) # ~50Hz loop

    # Cleanup
    if thrusters:
        print("Stopping all motors...")
        for thruster in thrusters.values():
            thruster.set_speed(IDLE_PULSE)
        time.sleep(0.5)
        for thruster in thrusters.values():
            thruster.set_speed(0) # Off
    if aux_pwm:
        print("Stopping auxiliary PWM...")
        aux_pwm.stop()
    sock.close()

def imu_thread(imu):
    """Continuously reads and filters IMU data."""
    if not imu: return
    
    pitch, roll = 0.0, 0.0
    while shared_state['running']:
        try:
            accel_data = imu.get_accel_data()
            acc_x, acc_y, acc_z = accel_data['x'], accel_data['y'], accel_data['z']

            acc_y = -accel_data['y']
            acc_x = -accel_data['x']
            
            

            # Standard formulas for rotation based on accelerometer data:
            # Rotation around X-axis (traditionally Roll)
            rotation_around_X = math.degrees(math.atan2(acc_y, acc_z))
            # Rotation around Y-axis (traditionally Pitch)
            rotation_around_Y = math.degrees(math.atan2(-acc_x, math.sqrt(acc_y * acc_y + acc_z * acc_z)))

            # Assign to new axes as per user request: Y-axis is Roll, X-axis is Pitch
            current_roll = rotation_around_Y
            current_pitch = rotation_around_X

            # Low-pass filter
            roll = FILTER_ALPHA * current_roll + (1 - FILTER_ALPHA) * roll
            pitch = FILTER_ALPHA * current_pitch + (1 - FILTER_ALPHA) * pitch
            
            shared_state['imu_data']['pitch'] = pitch
            shared_state['imu_data']['roll'] = roll
            
            time.sleep(0.02) # ~50Hz
        except Exception as e:
            print(f"IMU read failed: {e}")
            time.sleep(0.5)

# ============================================================================
# --- MAIN EXECUTION ---
# ============================================================================

def main():
    cam0, cam1, imu, thrusters, aux_pwm = setup_hardware()
    
    # Start worker threads
    imu_worker = threading.Thread(target=imu_thread, args=(imu,))
    joystick_worker = threading.Thread(target=joystick_and_thruster_thread, args=(thrusters, aux_pwm))
    imu_worker.start()
    joystick_worker.start()

    if cam0 or cam1:
        print("Displaying streams. Press 'q' in the window to quit.")
    else:
        print("Running in headless mode. Press Ctrl+C in the console to quit.")

    try:
        while shared_state['running']:
            # --- Frame Capture and Processing ---
            frame0, frame1 = None, None
            if cam0:
                frame0_raw = cam0.capture_array()
                frame0 = cv2.cvtColor(frame0_raw, cv2.COLOR_RGB2BGR)
                frame0 = cv2.rotate(frame0, cv2.ROTATE_90_CLOCKWISE)
            if cam1:
                # Assuming cam1 is physically rotated, we rotate the image back
                frame1_raw = cam1.capture_array()
                frame1 = cv2.cvtColor(frame1_raw, cv2.COLOR_RGB2BGR)
                frame1 = cv2.rotate(frame1, cv2.ROTATE_90_CLOCKWISE)

            # If no cameras are connected at all, create blank frames to show info
            if frame0 is None and frame1 is None:
                frame0 = np.zeros((PREVIEW_HEIGHT, PREVIEW_WIDTH, 3), dtype=np.uint8)
                frame1 = np.zeros((PREVIEW_HEIGHT, PREVIEW_WIDTH, 3), dtype=np.uint8)
            # Create a black frame if one camera is missing for consistent layout
            elif frame0 is None:
                h, w, _ = frame1.shape
                frame0 = np.zeros((h, w, 3), dtype=np.uint8)
            elif frame1 is None:
                h, w, _ = frame0.shape
                frame1 = np.zeros((h, w, 3), dtype=np.uint8)


            # Resize frame1 to match frame0's height for hstack
            if frame0 is not None and frame1 is not None:
                h0, w0, _ = frame0.shape
                h1, w1, _ = frame1.shape
                if h0 != h1:
                    scale = h0 / h1
                    new_w1 = int(w1 * scale)
                    frame1 = cv2.resize(frame1, (new_w1, h0), interpolation=cv2.INTER_AREA)

            # Get latest data from shared state
            pitch = shared_state['imu_data']['pitch']
            roll = shared_state['imu_data']['roll']
            joystick_cmd = shared_state['joystick_command'].copy()
            duty_cycle = shared_state['duty_cycle']

            # Draw overlays on available frames
            if frame0 is not None:
                draw_horizon(frame0, pitch, roll, HORIZON_COLOR, 2)
                draw_pitch_reference_line(frame0, PITCH_REFERENCE_ANGLE, PITCH_REF_COLOR, 2)
                draw_static_markers(frame0, HORIZON_COLOR, 2)
            if frame1 is not None:
                draw_horizon(frame1, pitch, roll, HORIZON_COLOR, 2)
                draw_pitch_reference_line(frame1, PITCH_REFERENCE_ANGLE, PITCH_REF_COLOR, 2)
                draw_static_markers(frame1, HORIZON_COLOR, 2)

            # Combine frames
            combined_frame = np.hstack((frame0, frame1))

            # --- Calculate thruster pulses for display ---
            # This duplicates some logic but keeps display separate from control
            t1 = joystick_cmd.get('forward',0) + joystick_cmd.get('pitch',0) + joystick_cmd.get('yaw',0) - joystick_cmd.get('roll',0)
            t2 = joystick_cmd.get('forward',0) + joystick_cmd.get('pitch',0) - joystick_cmd.get('yaw',0) + joystick_cmd.get('roll',0)
            t3 = joystick_cmd.get('forward',0) - joystick_cmd.get('pitch',0) + joystick_cmd.get('yaw',0) + joystick_cmd.get('roll',0)
            t4 = joystick_cmd.get('forward',0) - joystick_cmd.get('pitch',0) - joystick_cmd.get('yaw',0) - joystick_cmd.get('roll',0)
            t5 = joystick_cmd.get('roller', 0.0) / 660.0
            
            thrust_values = {
                'thruster1': max(-1, min(1, t1)), 'thruster2': max(-1, min(1, t2)),
                'thruster3': max(-1, min(1, t3)), 'thruster4': max(-1, min(1, t4)),
                'thruster5': max(-1, min(1, t5))
            }
            pulse_info = {name: get_pulse_from_thrust(val, THRUSTER_DIRECTIONS[name]) for name, val in thrust_values.items()}

            # Draw text overlay on the combined frame
            draw_info_overlay(combined_frame, pitch, roll, joystick_cmd, pulse_info, duty_cycle)

            # Display the final frame
            cv2.imshow(WINDOW_NAME, combined_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                shared_state['running'] = False
                break
            
            # Main loop can run slower as threads handle high-frequency tasks
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Ctrl+C detected. Shutting down.")
    except Exception as e:
        print(f"An error occurred in the main loop: {e}")
    finally:
        # Signal threads to stop and wait for them
        print("\nShutting down...")
        shared_state['running'] = False
        joystick_worker.join()
        imu_worker.join()

        # Cleanup hardware
        if cam0 and cam0.started: cam0.stop()
        if cam1 and cam1.started: cam1.stop()
        if aux_pwm: aux_pwm.stop()
        cv2.destroyAllWindows()
        print("Cleanup complete. Exiting.")

if __name__ == '__main__':
    main()
