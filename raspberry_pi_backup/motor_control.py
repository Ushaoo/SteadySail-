#!/usr/bin/env python3

from motor_test import PCA9685
import mpu6050
import time
import math
from imu_data import simple_imu_logger
from collections import deque
from queue import SimpleQueue, Empty
from threading import Thread, Event

try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except ImportError:  # Matplotlib might be unavailable on headless setups
    plt = None
    FuncAnimation = None

mode = True  # Set True for gyro+accel PID, False for angaccel PID

ENABLE_PLOTTER = True  # Set True to view IMU/PID signals live

# Plot visibility controls
PLOT_ANGLE = True
PLOT_ANGULAR_VELOCITY = True
PLOT_PID_OUTPUT = False
PLOT_ANGULAR_ACCELERATION = True
PLOT_MOTOR_PULSE = True


PLOT_WINDOW_SEC = 5.0
PLOT_MAX_WINDOW_SEC = 30.0
PLOT_SAMPLE_RATE_HZ = 100.0

angle_low = 5
angle_high = 10
angle_capsized = 35
 
motor_channel_1 = 0
# motor_channel_2 = 1
# motor_channel_3 = 2
# motor_channel_4 = 3
# motor_channel_5 = 4
# motor_channel_6 = 5



 
class IMUPIDPlotter:
    """Background matplotlib visualizer for IMU angle, angular velocity, and PID output."""

    def __init__(self, window_sec, max_window_sec, sample_rate_hz):
        if plt is None or FuncAnimation is None:
            raise RuntimeError("Matplotlib is required for ENABLE_PLOTTER mode")

        self.window_sec = window_sec
        max_samples = int(max_window_sec * sample_rate_hz)
        self.time_data = deque(maxlen=max_samples)
        self.angle_data = deque(maxlen=max_samples)
        self.angvel_data = deque(maxlen=max_samples)
        self.pid_data = deque(maxlen=max_samples)
        self.angaccel_data = deque(maxlen=max_samples)
        self.motorpulse_data = deque(maxlen=max_samples)
        self.queue: SimpleQueue[tuple[float, float, float, float, float, float]] = SimpleQueue()

    def record(self, timestamp, angle_deg, angular_velocity_deg_s, pid_output, angular_acceleration_deg_s2, motor_pulse):
        self.queue.put((timestamp, angle_deg, angular_velocity_deg_s, pid_output, angular_acceleration_deg_s2, motor_pulse))

    def run_plotter(self):
        plot_configs = [
            {"label": "Angle (deg)", "color": "tab:blue", "enabled": PLOT_ANGLE, "data": self.angle_data},
            {"label": "Angular Velocity (deg/s)", "color": "tab:orange", "enabled": PLOT_ANGULAR_VELOCITY, "data": self.angvel_data},
            {"label": "PID Output", "color": "tab:green", "enabled": PLOT_PID_OUTPUT, "data": self.pid_data},
            {"label": "Angular Accel (deg/s^2)", "color": "tab:red", "enabled": PLOT_ANGULAR_ACCELERATION, "data": self.angaccel_data},
            {"label": "Motor Pulse (us)", "color": "tab:purple", "enabled": PLOT_MOTOR_PULSE, "data": self.motorpulse_data},
        ]

        enabled_plots = [p for p in plot_configs if p["enabled"]]
        if not enabled_plots:
            print("Plotter enabled, but no plots are configured to display. Exiting plotter.")
            return

        num_plots = len(enabled_plots)
        fig, axes = plt.subplots(num_plots, 1, sharex=True, figsize=(9, 2.5 * num_plots))
        if num_plots == 1:
            axes = [axes]

        self.lines = []
        self.value_texts = []
        
        for i, plot_info in enumerate(enabled_plots):
            ax = axes[i]
            line, = ax.plot([], [], color=plot_info["color"], lw=1.5)
            ax.set_ylabel(plot_info["label"])
            ax.grid(True, alpha=0.3, linestyle="--")
            self.lines.append(line)
            value_text = ax.text(0.98, 0.95, '', transform=ax.transAxes, verticalalignment='top', horizontalalignment='right',
                                 fontsize=10, bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='gray', alpha=0.8))
            self.value_texts.append(value_text)

        axes[-1].set_xlabel("Time (s)")
        fig.subplots_adjust(left=0.1, right=0.98, top=0.96, bottom=0.1, hspace=0.4)

        def update(_frame):
            updated = False
            while True:
                try:
                    sample = self.queue.get_nowait()
                except Empty:
                    break
                timestamp, angle_deg, ang_vel, pid_output, ang_accel, motor_pulse = sample
                self.time_data.append(timestamp)
                self.angle_data.append(angle_deg)
                self.angvel_data.append(ang_vel)
                self.pid_data.append(pid_output)
                self.angaccel_data.append(ang_accel)
                self.motorpulse_data.append(motor_pulse)
                updated = True

            if not updated or not self.time_data:
                return self.lines + self.value_texts

            cutoff = self.time_data[-1] - self.window_sec
            while self.time_data and self.time_data[0] < cutoff:
                self.time_data.popleft()
                self.angle_data.popleft()
                self.angvel_data.popleft()
                self.pid_data.popleft()
                self.angaccel_data.popleft()
                self.motorpulse_data.popleft()

            for i, plot_info in enumerate(enabled_plots):
                line = self.lines[i]
                text_artist = self.value_texts[i]
                data = plot_info["data"]
                
                line.set_data(self.time_data, data)
                ax = line.axes
                ax.relim()
                ax.autoscale_view()
                if data:
                    text_artist.set_text(f'{data[-1]:.2f}')

            return self.lines + self.value_texts

        anim = FuncAnimation(fig, update, interval=20, blit=False, cache_frame_data=False)
        fig._imu_anim = anim  # Keep reference to avoid garbage collection
        plt.show()


class PIDController:
    def __init__(self, kp, ki, kd, target_value=0):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.target = target_value
        
        self.previous_error = 0
        self.integral = 0
        
    def gyro_update(self, current_value, dt):
        # Calculate error
        error = self.target - current_value
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term
        self.integral += error * dt
        integral = self.ki * self.integral
        
        # Derivative term
        derivative = self.kd * (error - self.previous_error) / dt
        
        # Calculate PID output
        output = proportional + integral + derivative
        
        # Store error for next iteration
        self.previous_error = error
        
        return output
    
    def acc_update(self, current_value, dt):
        # Calculate error
        error = self.target - current_value
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term
        self.integral += error * dt
        integral = self.ki * self.integral
        
        # Derivative term
        derivative = self.kd * (error - self.previous_error) / dt
        
        # Calculate PID output
        output = proportional + integral + derivative
        
        # Store error for next iteration
        self.previous_error = error
        
        return output
    
    def angacc_update(self, current_value, dt):
        # Calculate error
        error = self.target - current_value
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term
        self.integral += error * dt
        integral = self.ki * self.integral
        
        # Derivative term
        derivative = self.kd * (error - self.previous_error) / dt
        
        # Calculate PID output
        output = proportional + integral + derivative
        
        # Store error for next iteration
        self.previous_error = error
        
        return output
    
    def set_gyro_target(self, target):
        self.target = target

    def set_acc_target(self, target):
        self.target = target

    def set_angacc_target(self, target):
        self.target = target
        
    def reset(self):
        self.previous_error = 0
        self.integral = 0

def _control_loop(pwm, imu, gyro_pid, acc_pid, angacc_pid, plotter, stop_event):
    motor_channel = 0  # The channel connected to the motor (0-15)
    
    # Motor parameters
    base_pulse = 1500  # Neutral pulse width (1500 us for most ESCs)
    min_pulse = 1000   # Minimum pulse width
    max_pulse = 2000   # Maximum pulse width
    
    # Initialize motor (send neutral signal)
    pwm.setServoPulse(motor_channel_1, base_pulse)
    time.sleep(3)
    
    control_start = time.time()
    previous_time = control_start
    previous_angular_velocity_x = 0.0
    angular_acceleration_x = 0.0

    parameter = 0
    
    while not stop_event.is_set():
        current_time = time.time()
        dt = current_time - previous_time
        
        # Read IMU data
        gyro_data = imu.get_gyro_data()
        acc_data = imu.get_accel_data()

        acc_x =  acc_data['x']
        acc_y =  acc_data['y']
        acc_z =  acc_data['z']

        angle = math.atan(acc_y/acc_z) * 180 / math.pi
        
        angular_velocity_x = math.radians(gyro_data['x'])

        if dt > 0:
            angular_acceleration_x = (angular_velocity_x - previous_angular_velocity_x) / dt
        else:
            angular_acceleration_x = 0.0
        previous_angular_velocity_x = angular_velocity_x

        if mode:
            angle_abs = abs(angle)

            if angle_abs < angle_low:
                pid_output = gyro_pid.gyro_update(angular_velocity_x, dt)
            elif angle_abs >= angle_low and angle_abs <= angle_high:
                parameter = (angle_abs - angle_low)/(angle_high - angle_low)
                pid_output = (1 - parameter) * gyro_pid.gyro_update(angular_velocity_x, dt) + parameter * acc_pid.acc_update(angle, dt)
            elif angle_abs > angle_high:
                pid_output = acc_pid.acc_update(angle, dt)
            elif angle_abs > angle_capsized:
                pid_output = acc_pid.acc_update(angle, dt) * 2000
            else:
                pid_output = gyro_pid.gyro_update(angular_velocity_x, dt)
        else:
            pid_output = angacc_pid.angacc_update(angular_acceleration_x, dt)

        motor_adjustment = max(min(pid_output, 500), -500)
        motor_pulse = int(base_pulse + motor_adjustment)
        motor_pulse = max(min(motor_pulse, max_pulse), min_pulse)
        
        pwm.setServoPulse(motor_channel_1, motor_pulse)

        if plotter:
            plotter.record(current_time - control_start,
                           angle,
                           math.degrees(angular_velocity_x),
                           pid_output,
                           math.degrees(angular_acceleration_x),
                           motor_pulse)

        print(f"Angular Vel X: {angular_velocity_x:.3f} rad/s, "
              f"Angle: {angle:.3f} deg, "
              f"PID Output: {pid_output:.2f}, "
              f"Motor Pulse: {motor_pulse} us"
              f"parameter = {parameter}")
        
        previous_time = current_time
        time.sleep(0.01)

def main():
    # Initialize the PCA9685 module for motor control
    pwm = PCA9685(address=0x40, debug=True, bus_num=1)
    pwm.setPWMFreq(50)  # Set frequency to 50 Hz
    
    # Initialize the MPU6050 IMU sensor
    imu = mpu6050.mpu6050(0x68)
    
    # PID Controller setup
    gyro_kp = 294.0
    gyro_ki = 0.08
    gyro_kd = 0.0
    gyro_target = 0.0
    
    acc_kp = 29.0
    acc_ki = 0.000
    acc_kd = 0.008
    acc_target = 0.0

    angacc_kp = 292.0
    angacc_ki = 0.000
    angacc_kd = 0.008
    angacc_target = 0.0
    
    gyro_pid = PIDController(gyro_kp, gyro_ki, gyro_kd, gyro_target)
    acc_pid = PIDController(acc_kp, acc_ki, acc_kd, acc_target)
    angacc_pid = PIDController(angacc_kp, angacc_ki, angacc_kd, angacc_target)
    
    print("Starting IMU-based motor control with PID...")
    print("Press Ctrl+C to stop")
    
    plotter = None
    if ENABLE_PLOTTER and plt is not None and FuncAnimation is not None:
        try:
            plotter = IMUPIDPlotter(PLOT_WINDOW_SEC, PLOT_MAX_WINDOW_SEC, PLOT_SAMPLE_RATE_HZ)
        except RuntimeError as exc:
            print(f"Plotter unavailable: {exc}")

    stop_event = Event()
    control_thread = Thread(target=_control_loop, args=(pwm, imu, gyro_pid, acc_pid, angacc_pid, plotter, stop_event))
    control_thread.start()

    if plotter:
        plotter.run_plotter()

    try:
        control_thread.join()
    except KeyboardInterrupt:
        print("\nMotor control interrupted by user.")
    finally:
        stop_event.set()
        control_thread.join()
        base_pulse = 1500
        pwm.setServoPulse(motor_channel_1, base_pulse)
        time.sleep(0.5)
        pwm.setServoPulse(motor_channel_1, 0)

if __name__ == "__main__":
    main()

# Additional utility functions for advanced control

def calibrate_imu(imu, samples=100):
    """Calibrate the IMU by collecting baseline readings"""
    print(f"Calibrating IMU with {samples} samples...")
    
    gyro_offsets = {'x': 0, 'y': 0, 'z': 0}
    
    for i in range(samples):
        gyro_data = imu.get_gyro_data()
        gyro_offsets['x'] += gyro_data['x']
        gyro_offsets['y'] += gyro_data['y'] 
        gyro_offsets['z'] += gyro_data['z']
        time.sleep(0.01)
        
    # Calculate averages
    gyro_offsets['x'] /= samples
    gyro_offsets['y'] /= samples
    gyro_offsets['z'] /= samples
    
    print(f"IMU Calibration complete. Offsets: {gyro_offsets}")
    return gyro_offsets

def stabilized_motor_control():
    """Advanced version with IMU calibration and stabilization"""
    # Initialize hardware
    pwm = PCA9685(address=0x40, debug=True, bus_num=1)
    pwm.setPWMFreq(50)
    imu = mpu6050.mpu6050(0x68)
    
    # Calibrate IMU
    gyro_offsets = calibrate_imu(imu)
    
    # PID setup for stabilization (keeping angular velocity at 0)
    pid_roll = PIDController(kp=30.0, ki=2.0, kd=8.0, target_value=0.0)
    pid_pitch = PIDController(kp=30.0, ki=2.0, kd=8.0, target_value=0.0)
    
    
    base_pulse = 1500
    
    print("Starting stabilized motor control...")
    print("The system will try to maintain stability using IMU feedback")
    print("Press Ctrl+C to stop")
    
    try:
        pwm.setServoPulse(motor_channel_1, base_pulse)
        # pwm.setServoPulse(motor_channel_2, base_pulse)
        # pwm.setServoPulse(motor_channel_3, base_pulse)
        # pwm.setServoPulse(motor_channel_4, base_pulse)
        # pwm.setServoPulse(motor_channel_5, base_pulse)
        # pwm.setServoPulse(motor_channel_6, base_pulse)
        time.sleep(2)
        
        previous_time = time.time()
        
        while True:
            current_time = time.time()
            dt = current_time - previous_time
            
            # Read and correct gyro data
            gyro_data = imu.get_gyro_data()
            corrected_roll = math.radians(gyro_data['x'] - gyro_offsets['x'])
            corrected_pitch = math.radians(gyro_data['y'] - gyro_offsets['y'])
            
            # Calculate PID outputs for stabilization
            roll_output = pid_roll.update(corrected_roll, dt)
            pitch_output = pid_pitch.update(corrected_pitch, dt)
            
            # Combine outputs (you might want to weight these differently)
            combined_output = (roll_output + pitch_output) / 2
            
            # Apply to motor
            motor_adjustment = max(min(combined_output, 400), -400)
            motor_pulse = int(base_pulse + motor_adjustment)
            motor_pulse = max(min(motor_pulse, 2000), 1000)
            
            pwm.setServoPulse(motor_channel, motor_pulse)
            pwm.setServoPulse(1, motor_pulse)
            
            print(f"Roll: {math.degrees(corrected_roll):.2f}°/s, "
                  f"Pitch: {math.degrees(corrected_pitch):.2f}°/s, "
                  f"Motor: {motor_pulse} us")
            
            previous_time = current_time
            time.sleep(0.02)
            
    except KeyboardInterrupt:
        print("\nStabilized control stopped.")
        pwm.setServoPulse(motor_channel_1, base_pulse)
        # pwm.setServoPulse(motor_channel_2, base_pulse)
        # pwm.setServoPulse(motor_channel_3, base_pulse)
        # pwm.setServoPulse(motor_channel_4, base_pulse)
        # pwm.setServoPulse(motor_channel_5, base_pulse)
        # pwm.setServoPulse(motor_channel_6, base_pulse)
        time.sleep(0.5)
        pwm.setServoPulse(motor_channel_1, 0)
        # pwm.setServoPulse(motor_channel_2, 0)
        # pwm.setServoPulse(motor_channel_3, 0)
        # pwm.setServoPulse(motor_channel_4, 0)
        # pwm.setServoPulse(motor_channel_5, 0)
        # pwm.setServoPulse(motor_channel_6, 0)