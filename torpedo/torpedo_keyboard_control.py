import curses
import time
from collections import defaultdict
import math
import smbus

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
THRUSTER_1_CHANNEL = 0
THRUSTER_2_CHANNEL = 4
THRUSTER_3_CHANNEL = 2
THRUSTER_4_CHANNEL = 3

# PWM settings
PWM_FREQ = 50
MIN_PULSE = 1100
MAX_PULSE = 1900
IDLE_PULSE = 1500

# Thruster speed settings
FORWARD_PULSE = 1800
BACKWARD_PULSE = 1200

# Thruster inversion flags
THRUSTER_DIRECTIONS = {
    'thruster1': 1.0,
    'thruster2': 1.0,
    'thruster3': 1.0,
    'thruster4': -1.0,
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
    }
    
    print("Arming ESCs...")
    for thruster in thrusters.values():
        thruster.set_speed(IDLE_PULSE)
    time.sleep(2)
    print("ESCs armed.")
    return thrusters

def cleanup(thrusters):
    """Stops all thrusters."""
    print("Stopping all motors...")
    for thruster in thrusters.values():
        thruster.set_speed(IDLE_PULSE)
    time.sleep(0.5)
    for thruster in thrusters.values():
        thruster.set_speed(0)


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


def main(stdscr):
    # Setup curses
    curses.curs_set(0)
    stdscr.nodelay(1)
    stdscr.timeout(100)

    thrusters = setup_thrusters()
    
    # thrust_levels stores the contribution from each axis of movement
    # e.g., {'forward': 0.5, 'pitch': -0.3, ...}
    thrust_levels = defaultdict(float)
    
    try:
        while True:
            key = stdscr.getch()
            
            # Reset thrust levels for keys that are not pressed
            thrust_levels.clear()

            if key == curses.KEY_UP:
                thrust_levels['forward'] = 1.0
            elif key == curses.KEY_DOWN:
                thrust_levels['forward'] = -1.0
            
            if key == ord('w'):
                thrust_levels['pitch'] = 1.0 # Pitch Up
            elif key == ord('s'):
                thrust_levels['pitch'] = -1.0 # Pitch Down

            if key == ord('d'):
                thrust_levels['yaw'] = 1.0 # Yaw Right
            elif key == ord('a'):
                thrust_levels['yaw'] = -1.0 # Yaw Left

            if key == ord('e'):
                thrust_levels['roll'] = 1.0 # Roll Right
            elif key == ord('q'):
                thrust_levels['roll'] = -1.0 # Roll Left
            
            if key == ord(' '): # Stop all
                pass # All levels will be 0
            
            if key == 27: # ESC key
                break

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
            t1_thrust -= thrust_levels['yaw']
            t2_thrust += thrust_levels['yaw']
            t3_thrust -= thrust_levels['yaw']
            t4_thrust += thrust_levels['yaw']

            # Add roll
            t1_thrust -= thrust_levels['roll']
            t2_thrust += thrust_levels['roll']
            t3_thrust += thrust_levels['roll']
            t4_thrust -= thrust_levels['roll']

            # Clamp values between -1 and 1
            final_thrusts = {
                'thruster1': max(-1.0, min(1.0, t1_thrust)),
                'thruster2': max(-1.0, min(1.0, t2_thrust)),
                'thruster3': max(-1.0, min(1.0, t3_thrust)),
                'thruster4': max(-1.0, min(1.0, t4_thrust)),
            }

            # Set thruster speeds
            for name, thrust_val in final_thrusts.items():
                direction = THRUSTER_DIRECTIONS[name]
                pulse = get_pulse_from_thrust(thrust_val, direction)
                thrusters[name].set_speed(pulse)

            # Display info
            stdscr.clear()
            stdscr.addstr(0, 0, "Use arrow keys (Up/Down), WS, AD, QE to control. Space to stop. ESC to exit.")
            for i, (name, thrust_val) in enumerate(final_thrusts.items()):
                pulse = get_pulse_from_thrust(thrust_val, THRUSTER_DIRECTIONS[name])
                stdscr.addstr(i + 2, 0, f"{name}: Thrust={thrust_val: .2f}, Pulse={pulse}us")
            stdscr.refresh()

    finally:
        cleanup(thrusters)

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except Exception as e:
        print(f"An error occurred: {e}")
        # The cleanup function for PCA9685 does not need GPIO cleanup
        # but we can add a placeholder for other cleanup if needed.
        print("Exiting due to an error.")
