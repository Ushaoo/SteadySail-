#!/usr/bin/env python3
"""Real-time IMU visualizer for angle, angular speed, and angular acceleration."""

import argparse
import math
import sys
import time
from collections import deque
from typing import Deque, Tuple

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider

import mpu6050


def create_imu(address: int):
    """Instantiate the legacy mpu6050 driver with simple address fallback."""
    try:
        return mpu6050.mpu6050(address)
    except Exception:
        # Allow automatic fallback to secondary address (commonly 0x69)
        if address != 0x69:
            return mpu6050.mpu6050(0x69)
        raise


def compute_angle_deg(acc_data) -> float:
    """Compute tilt angle (deg) using arctan2 of Y over Z axes."""
    acc_y = acc_data.get("y", 0.0)
    acc_z = acc_data.get("z", 1e-6)
    return math.degrees(math.atan2(acc_y, acc_z if acc_z else 1e-6))


class IMUStream:
    def __init__(self, bus: int, address: int):
        # Legacy driver ignores bus argument; keep signature for CLI parity
        self.imu = create_imu(address)
        self.start_time = time.perf_counter()
        self.prev_timestamp = None
        self.prev_ang_vel = None

    def read(self) -> Tuple[float, float, float, float]:
        """Return (elapsed_time, angle_deg, angular_velocity_dps, angular_accel_dps2)."""
        now = time.perf_counter()
        acc = self.imu.get_accel_data()
        gyro = self.imu.get_gyro_data()

        elapsed = now - self.start_time
        angle = compute_angle_deg(acc)
        angular_velocity = float(gyro.get("x", 0.0))

        if self.prev_timestamp is None or self.prev_ang_vel is None:
            angular_acc = 0.0
        else:
            dt = max(now - self.prev_timestamp, 1e-6)
            angular_acc = (angular_velocity - self.prev_ang_vel) / dt

        self.prev_timestamp = now
        self.prev_ang_vel = angular_velocity
        return elapsed, angle, angular_velocity, angular_acc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Real-time IMU visualizer")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus number (default: 1)")
    parser.add_argument("--address", type=lambda v: int(v, 0), default=0x68,
                        help="IMU I2C address (default: 0x68)")
    parser.add_argument("--sample-rate", type=float, default=50.0,
                        help="Target samples per second (for buffer sizing)")
    parser.add_argument("--window", type=float, default=5.0,
                        help="Initial time window in seconds")
    parser.add_argument("--max-window", type=float, default=30.0,
                        help="Maximum time window for the slider")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.window > args.max_window:
        print("Initial window cannot exceed max window", file=sys.stderr)
        sys.exit(1)

    stream = IMUStream(args.bus, args.address)

    max_samples = int(args.max_window * args.sample_rate * 1.5)
    time_buffer: Deque[float] = deque(maxlen=max_samples)
    angle_buffer: Deque[float] = deque(maxlen=max_samples)
    gyro_buffer: Deque[float] = deque(maxlen=max_samples)
    angacc_buffer: Deque[float] = deque(maxlen=max_samples)

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    labels = [("Angle (deg)", angle_buffer, "tab:blue"),
              ("Angular Speed (deg/s)", gyro_buffer, "tab:orange"),
              ("Angular Accel (deg/s^2)", angacc_buffer, "tab:green")]
    lines = []
    for ax, (ylabel, _, color) in zip(axes, labels):
        line, = ax.plot([], [], color=color, lw=1.5)
        ax.set_ylabel(ylabel)
        ax.grid(True, linestyle="--", alpha=0.3)
        lines.append(line)
    axes[-1].set_xlabel("Time (s)")

    slider_ax = fig.add_axes([0.15, 0.02, 0.7, 0.03])
    window_slider = Slider(slider_ax, "Window (s)", 1.0, args.max_window, valinit=args.window,
                           valstep=0.5)

    def slice_buffers(window_seconds: float):
        if not time_buffer:
            return [], [], [], []
        cutoff = time_buffer[-1] - window_seconds
        start_idx = 0
        for idx, ts in enumerate(time_buffer):
            if ts >= cutoff:
                start_idx = idx
                break
        return (list(time_buffer)[start_idx:],
                list(angle_buffer)[start_idx:],
                list(gyro_buffer)[start_idx:],
                list(angacc_buffer)[start_idx:])

    def update_plot():
        window = window_slider.val
        times, angles, gyros, angaccs = slice_buffers(window)
        if not times:
            return
        datasets = [angles, gyros, angaccs]
        for ax, line, data in zip(axes, lines, datasets):
            line.set_data(times, data)
            ax.set_xlim(times[0], times[-1])
            ax.relim()
            ax.autoscale_view()
        fig.canvas.draw_idle()

    def on_slider_change(_):
        update_plot()

    window_slider.on_changed(on_slider_change)

    def update(_frame):
        elapsed, angle, gyro_val, ang_acc = stream.read()
        time_buffer.append(elapsed)
        angle_buffer.append(angle)
        gyro_buffer.append(gyro_val)
        angacc_buffer.append(ang_acc)
        update_plot()
        return lines

    anim = FuncAnimation(fig, update, interval=5, blit=False, cache_frame_data=False)
    fig._imu_anim = anim  # Prevent garbage collection before the window closes

    try:
        fig.subplots_adjust(left=0.08, right=0.98, top=0.97, bottom=0.12, hspace=0.25)
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        anim.event_source.stop()


if __name__ == "__main__":
    main()
