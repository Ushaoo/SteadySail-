import socket
import json
import time
from collections import defaultdict
import math
import smbus
import os

# ============================================================================
# Sysfs PIO-PWM Controller
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

# ============================================================================
# Raspi PCA9685 16-Channel PWM Servo/ESC Driver
# ============================================================================
class PCA9685:
  # Registers/etc.
  __SUBADR1            = 0x02
  __SUBADR2            = 0x03
  __SUBADR3            = 0x04
  __MODE1              = 0x00
  __PRESCALE           = 0xFE
  __LED0_ON_L          = 0x06
  __LED0_ON_H          = 0x07
  __LED0_OFF_L         = 0x08
  __LED0_OFF_H         = 0x09
  __ALLLED_ON_L        = 0xFA
  __ALLLED_ON_H        = 0xFB
  __ALLLED_OFF_L       = 0xFC
  __ALLLED_OFF_H       = 0xFD

  def __init__(self, address=0x40, debug=False, bus_num=1):
    self.bus = smbus.SMBus(bus_num)
    self.address = address
    self.debug = debug
    self.freq = 50
    if self.debug:
      print("Resetting PCA9685")
    self.write(self.__MODE1, 0x00)

  def write(self, reg, value):
    "Writes an 8-bit value to the specified register/address"
    self.bus.write_byte_data(self.address, reg, value)

  def read(self, reg):
    "Read an unsigned byte from the I2C device"
    return self.bus.read_byte_data(self.address, reg)

  def setPWMFreq(self, freq):
    "Sets the PWM frequency"
    self.freq = freq
    prescaleval = 25000000.0    # 25MHz
    prescaleval /= 4096.0       # 12-bit
    prescaleval /= float(freq)
    prescaleval -= 1.0
    prescale = math.floor(prescaleval + 0.5)
    
    oldmode = self.read(self.__MODE1)
    newmode = (oldmode & 0x7F) | 0x10
    self.write(self.__MODE1, newmode)
    self.write(self.__PRESCALE, int(prescale))
    self.write(self.__MODE1, oldmode)
    time.sleep(0.005)
    self.write(self.__MODE1, oldmode | 0x80)

  def setPWM(self, channel, on, off):
    "Sets a single PWM channel"
    self.write(self.__LED0_ON_L + 4*channel, on & 0xFF)
    self.write(self.__LED0_ON_H + 4*channel, (on >> 8) & 0xFF)
    self.write(self.__LED0_OFF_L + 4*channel, off & 0xFF)
    self.write(self.__LED0_OFF_H + 4*channel, (off >> 8) & 0xFF)

  def setServoPulse(self, channel, pulse_us):
    "Sets the pulse width in microseconds"
    period_us = 1_000_000.0 / float(self.freq)
    ticks = int(round((pulse_us / period_us) * 4096.0))
    if ticks < 0: ticks = 0
    if ticks > 4095: ticks = 4095
    self.setPWM(channel, 0, ticks)

# PCA9685 channel definitions
THRUSTER_1_CHANNEL = 2
THRUSTER_2_CHANNEL = 3
THRUSTER_3_CHANNEL = 4
THRUSTER_4_CHANNEL = 5
THRUSTER_5_CHANNEL = 6

# PWM settings
PWM_FREQ = 50
MIN_PULSE = 1000
MAX_PULSE = 2000 
IDLE_PULSE = 1500

# Thruster speed settings
FORWARD_PULSE = 2000
BACKWARD_PULSE = 1000

# Thruster inversion flags
THRUSTER_DIRECTIONS = {
    'thruster1': 1.0,
    'thruster2': -1.0,
    'thruster3': 1.0,
    'thruster4': -1.0,
    'thruster5': 1.0,
}

class Thruster:
    def __init__(self, pwm_driver, channel):
        self.pwm = pwm_driver
        self.channel = channel

    def set_speed(self, pulse_width):
        """Sets the speed of the thruster based on pulse width in microseconds."""
        self.pwm.setServoPulse(self.channel, int(pulse_width))

    def stop(self):
        # For PCA9685, stopping means setting to idle pulse.
        self.set_speed(IDLE_PULSE)

def setup_thrusters():
    """Initializes PCA9685 and thruster objects."""
    try:
        pwm = PCA9685(address=0x40, bus_num=1)
        pwm.setPWMFreq(PWM_FREQ)
    except Exception as e:
        print(f"Error initializing PCA9685: {e}")
        print("Please check I2C connection and address.")
        return None

    thrusters = {
        'thruster1': Thruster(pwm, THRUSTER_1_CHANNEL),
        'thruster2': Thruster(pwm, THRUSTER_2_CHANNEL),
        'thruster3': Thruster(pwm, THRUSTER_3_CHANNEL),
        'thruster4': Thruster(pwm, THRUSTER_4_CHANNEL),
        'thruster5': Thruster(pwm, THRUSTER_5_CHANNEL),
    }
    
    print("Arming ESCs...")
    for thruster in thrusters.values():
        thruster.set_speed(IDLE_PULSE)
    time.sleep(2)
    print("ESCs armed.")
    return thrusters

def cleanup(thrusters, aux_pwm):
    """Stops all thrusters and auxiliary PWM."""
    print("Stopping all motors...")
    if thrusters:
        for thruster in thrusters.values():
            thruster.set_speed(IDLE_PULSE)
        time.sleep(0.5)
        for thruster in thrusters.values():
            thruster.set_speed(0)
    
    print("Stopping external PWM...")
    if aux_pwm:
        aux_pwm.stop()


def get_pulse_from_thrust(thrust_value, direction):
    """Converts a thrust value (-1 to 1) to a pulse width."""
    if thrust_value == 0:
        return IDLE_PULSE
    
    # Apply direction inversion
    thrust_value *= direction

    if thrust_value > 0: # Forward
        return int(IDLE_PULSE + thrust_value * (FORWARD_PULSE - IDLE_PULSE))
    else: # Backward
        return int(IDLE_PULSE + thrust_value * (IDLE_PULSE - BACKWARD_PULSE))


def handle_button_press(aux_pwm, buttons_value):
    """根据按钮值设置辅助 PWM 的占空比，并返回设置的占空比。"""
    duty_cycle = None
    if buttons_value == 0:
        duty_cycle = 99.0
    elif buttons_value == 16:
        duty_cycle = 50.0
    elif buttons_value == 32:
        duty_cycle = 0.0
    
    if duty_cycle is not None:
        # print(f"按钮值为 {buttons_value}, 设置 PWM 占空比为 {duty_cycle}%")
        aux_pwm.setup(frequency=20000, duty_cycle=duty_cycle)
    
    return duty_cycle


def main():
    duty_cycle = None
    # Network settings
    UDP_IP = "0.0.0.0"  # Listen on all available network interfaces
    UDP_PORT = 5005
    
    # Setup socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(1.0) # Set a 1-second timeout for receiving data

    print(f"Listening for joystick commands on UDP port {UDP_PORT}...")

    thrusters = setup_thrusters()
    if not thrusters:
        return

    # 初始化辅助 PWM
    aux_pwm = None
    try:
        aux_pwm = SysfsPWM()
        print("辅助 PWM 控制器已初始化。")
    except (IOError, FileNotFoundError) as e:
        print(e)
        print("无法初始化辅助 PWM，将禁用此功能。")

    # thrust_levels stores the contribution from each axis of movement
    thrust_levels = defaultdict(float)
    last_command = {}
    last_buttons_value = -1 # 用于检测按钮状态变化
    
    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
                command = json.loads(data.decode())
                last_command = command
                
                # Update thrust levels from received command
                thrust_levels['forward'] = command.get('forward', 0.0)
                thrust_levels['pitch'] = command.get('pitch', 0.0)
                thrust_levels['yaw'] = command.get('yaw', 0.0)
                thrust_levels['roll'] = command.get('roll', 0.0)
                thrust_levels['roller'] = command.get('roller', 0.0)

                # 处理 'buttons' 信号
                if aux_pwm:
                    current_buttons_value = command.get('buttons')
                    if current_buttons_value is not None and current_buttons_value != last_buttons_value:
                        new_duty_cycle = handle_button_press(aux_pwm, current_buttons_value)
                        if new_duty_cycle is not None:
                            duty_cycle = new_duty_cycle
                        last_buttons_value = current_buttons_value

            except socket.timeout:
                # If no data is received for 1 second, stop all motors
                print("Connection timed out. Stopping motors.")
                thrust_levels.clear()
            except (json.JSONDecodeError, KeyError):
                # Handle malformed data
                print("Received malformed data. Stopping motors.")
                thrust_levels.clear()


            # Combine thrust levels for each thruster
            # Initialize with base thrust (forward/backward)
            t1_thrust = thrust_levels['forward']
            t2_thrust = thrust_levels['forward']
            t3_thrust = thrust_levels['forward']
            t4_thrust = thrust_levels['forward']

            # Add pitch
            t1_thrust += thrust_levels['pitch']
            t2_thrust += thrust_levels['pitch']
            t3_thrust -= thrust_levels['pitch']
            t4_thrust -= thrust_levels['pitch']

            # Add yaw
            t1_thrust += thrust_levels['yaw']
            t2_thrust -= thrust_levels['yaw']
            t3_thrust += thrust_levels['yaw']
            t4_thrust -= thrust_levels['yaw']

            # Add roll
            t1_thrust += thrust_levels['roll']
            t2_thrust -= thrust_levels['roll']
            t3_thrust -= thrust_levels['roll']
            t4_thrust += thrust_levels['roll']

            # Clamp values between -1 and 1
            final_thrusts = {
                'thruster1': max(-1.0, min(1.0, t1_thrust)),
                'thruster2': max(-1.0, min(1.0, t2_thrust)),
                'thruster3': max(-1.0, min(1.0, t3_thrust)),
                'thruster4': max(-1.0, min(1.0, t4_thrust)),
                'thruster5': max(-1.0, min(1.0, thrust_levels['roller']/660.0)), # Normalize roller input
            }

            # Set thruster speeds
            for name, thrust_val in final_thrusts.items():
                direction = THRUSTER_DIRECTIONS[name]
                pulse = get_pulse_from_thrust(thrust_val, direction)
                thrusters[name].set_speed(pulse)

            # Display info
            print("\033c", end="") # Clear console
            print("Receiving joystick commands. Press Ctrl+C to exit.")
            print(f"Received command: {last_command}")
            for i, (name, thrust_val) in enumerate(final_thrusts.items()):
                pulse = get_pulse_from_thrust(thrust_val, THRUSTER_DIRECTIONS[name])
                print(f"{name}: Thrust={thrust_val: .2f}, Pulse={pulse}us")
            if duty_cycle is not None:
                print(f"设置 PWM 占空比为 {duty_cycle}%")
            
            time.sleep(0.01) # Loop at ~20Hz

    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        cleanup(thrusters, aux_pwm)

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"An error occurred: {e}")
        # The cleanup function for PCA9685 does not need GPIO cleanup
        # but we can add a placeholder for other cleanup if needed.
        print("Exiting due to an error.")
