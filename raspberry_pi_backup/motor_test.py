#!/usr/bin/env python3

import time
import math
import smbus

# ============================================================================
# Raspi PCA9685 16-Channel PWM Servo/ESC Driver (interactive % input)
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

  def __init__(self, address=0x40, debug=True, bus_num=1):
    self.bus = smbus.SMBus(bus_num)
    self.address = address
    self.debug = debug
    self.freq = 50  # default; will be set by setPWMFreq()
    if self.debug:
      print("Resetting PCA9685")
    self.write(self.__MODE1, 0x00)

  def write(self, reg, value):
    "Writes an 8-bit value to the specified register/address"
    self.bus.write_byte_data(self.address, reg, value)
    if self.debug:
      # print("I2C: Write 0x%02X to register 0x%02X" % (value & 0xFF, reg))
      pass

  def read(self, reg):
    "Read an unsigned byte from the I2C device"
    result = self.bus.read_byte_data(self.address, reg)
    if self.debug:
      print("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" % (self.address, result & 0xFF, reg))
    return result

  def setPWMFreq(self, freq):
    "Sets the PWM frequency"
    prescaleval = 25000000.0    # 25MHz
    prescaleval /= 4096.0       # 12-bit
    prescaleval /= float(freq)
    prescaleval -= 1.0
    if self.debug:
      print("Setting PWM frequency to %d Hz" % freq)
      print("Estimated pre-scale: %f" % prescaleval)
    prescale = math.floor(prescaleval + 0.5)
    if self.debug:
      print("Final pre-scale: %d" % prescale)

    oldmode = self.read(self.__MODE1)
    newmode = (oldmode & 0x7F) | 0x10        # sleep
    self.write(self.__MODE1, newmode)        # go to sleep
    self.write(self.__PRESCALE, int(prescale))
    self.write(self.__MODE1, oldmode)
    time.sleep(0.005)
    self.write(self.__MODE1, oldmode | 0x80)
    self.freq = freq  # remember current frequency

  def setPWM(self, channel, on, off):
    "Sets a single PWM channel"
    self.write(self.__LED0_ON_L + 4*channel, on & 0xFF)
    self.write(self.__LED0_ON_H + 4*channel, (on >> 8) & 0xFF)
    self.write(self.__LED0_OFF_L + 4*channel, off & 0xFF)
    self.write(self.__LED0_OFF_H + 4*channel, (off >> 8) & 0xFF)
    '''if self.debug:
      print("channel: %d  LED_ON: %d LED_OFF: %d" % (channel, on, off))'''

  def setServoPulse(self, channel, pulse_us):
    """
    Sets the pulse width in microseconds at the current frequency.
    Default usage is 50 Hz for servos/ESCs.
    """
    period_us = 1_000_000.0 / float(self.freq)
    ticks = int(round((pulse_us / period_us) * 4096.0))
    # Clamp to 0..4095
    if ticks < 0: ticks = 0
    if ticks > 4095: ticks = 4095
    self.setPWM(channel, 0, ticks)