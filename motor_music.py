"""
motor_music.py

Plays a simple melody by modulating ESC servo pulse widths on a PCA9685.

WARNING: Use with care. Ensure motors are disconnected or props removed while testing.
"""

from motor_test import PCA9685
import time
import math

# Configuration
PWM_ADDRESS = 0x40
PWM_BUS = 1
PWM_FREQ = 50  # Hz
BASE_PULSE = 1500  # neutral
MIN_PULSE = 1500
MAX_PULSE = 2000

# Map note names to frequencies (A4 = 440 Hz)
NOTE_FREQS = {
    'C4': 261.63, 'D4': 293.66, 'E4': 329.63, 'F4': 349.23,
    'G4': 392.00, 'A4': 440.00, 'B4': 493.88, 'C5': 523.25,
}

# Extended melody (Twinkle Twinkle + Happy Birthday snippet)
MELODY = [
    # Twinkle Twinkle
    ('C4', 0.5), ('C4', 0.5), ('G4', 0.5), ('G4', 0.5),
    ('A4', 0.5), ('A4', 0.5), ('G4', 1.0),
    ('F4', 0.5), ('F4', 0.5), ('E4', 0.5), ('E4', 0.5),
    ('D4', 0.5), ('D4', 0.5), ('C4', 1.0),
    # Repeat phrase
    ('G4', 0.5), ('G4', 0.5), ('F4', 0.5), ('F4', 0.5),
    ('E4', 0.5), ('E4', 0.5), ('D4', 1.0),
    ('G4', 0.5), ('G4', 0.5), ('F4', 0.5), ('F4', 0.5),
    ('E4', 0.5), ('E4', 0.5), ('D4', 1.0),

    # Happy Birthday (first line)
    ('C4', 0.5), ('C4', 0.25), ('D4', 0.75), ('C4', 0.75), ('F4', 0.75), ('E4', 1.5),
    # Happy Birthday (second line)
    ('C4', 0.5), ('C4', 0.25), ('D4', 0.75), ('C4', 0.75), ('G4', 0.75), ('F4', 1.5),
]


def freq_to_pulse(freq, min_freq=100.0, max_freq=1000.0):
    """Map a frequency to a servo pulse width between MIN_PULSE and MAX_PULSE.

    Frequencies below min_freq map to MIN_PULSE, above max_freq map to MAX_PULSE.
    The mapping is linear in frequency.
    """
    if freq <= min_freq:
        return MIN_PULSE
    if freq >= max_freq:
        return MAX_PULSE
    # linear interpolation
    t = (freq - min_freq) / (max_freq - min_freq)
    pulse = int(MIN_PULSE + t * (MAX_PULSE - MIN_PULSE))
    # clamp to configured range
    if pulse < MIN_PULSE:
        return MIN_PULSE
    if pulse > MAX_PULSE:
        return MAX_PULSE
    return pulse


def play_melody(channel=0):
    pwm = PCA9685(address=PWM_ADDRESS, debug=False, bus_num=PWM_BUS)
    pwm.setPWMFreq(PWM_FREQ)

    # initialize to neutral
    pwm.setServoPulse(channel, BASE_PULSE)
    time.sleep(2)

    try:
        for note, dur in MELODY:
            freq = NOTE_FREQS.get(note, 440.0)
            pulse = freq_to_pulse(freq)
            print(f"Playing {note} ({freq}Hz) -> pulse {pulse} on channel {channel} for {dur}s")
            pwm.setServoPulse(channel, pulse)
            time.sleep(dur)
            # return to neutral between notes
            pwm.setServoPulse(channel, BASE_PULSE)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Interrupted, stopping motor")
    finally:
        pwm.setServoPulse(channel, BASE_PULSE)
        time.sleep(0.2)
        pwm.setServoPulse(channel, 0)


if __name__ == '__main__':
    # Play on channel 0 by default
    play_melody(channel=0)
