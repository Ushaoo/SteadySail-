#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
数据接收器与绘图器 - 通过 UDP 接收树莓派发送的数据并实时绘图
运行在电脑上
"""

import socket
import struct
import time
import threading
from collections import deque
from queue import SimpleQueue, Empty

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button

# ==================== 网络配置 ====================
UDP_IP = "0.0.0.0"           # 监听所有网卡
UDP_PORT = 5005              # 接收端口

# ==================== 绘图配置 ====================
PLOT_WINDOW_SEC = 10.0       # 默认显示窗口（秒）
PLOT_MAX_WINDOW_SEC = 60.0   # 最大窗口（秒）
MAX_SAMPLES = 6000           # 最大缓存样本数（60秒 * 100Hz）
UPDATE_INTERVAL = 50         # 绘图刷新间隔（ms）

# PWM 中位值
PWM_NEUTRAL = 1500


class DataReceiver:
    """UDP 数据接收器"""
    
    def __init__(self, ip=UDP_IP, port=UDP_PORT):
        self.ip = ip
        self.port = port
        self.socket = None
        self.queue = SimpleQueue()
        self.running = False
        self.recv_thread = None
        self.packet_count = 0
        self.last_packet_time = 0
        
    def start(self):
        """启动接收线程"""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.ip, self.port))
        self.socket.settimeout(0.1)  # 100ms 超时
        
        self.running = True
        self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.recv_thread.start()
        print(f"数据接收器已启动，监听 {self.ip}:{self.port}")
        
    def stop(self):
        """停止接收线程"""
        self.running = False
        if self.recv_thread:
            self.recv_thread.join(timeout=1.0)
        if self.socket:
            self.socket.close()
        print(f"数据接收器已停止，共接收 {self.packet_count} 个数据包")
    
    def get_data(self):
        """获取接收到的数据"""
        data_list = []
        while True:
            try:
                data_list.append(self.queue.get_nowait())
            except Empty:
                break
        return data_list
    
    def _recv_loop(self):
        """接收循环"""
        while self.running:
            try:
                data, addr = self.socket.recvfrom(1024)
                
                if len(data) == 64:  # 8 * 8 bytes (8 doubles)
                    values = struct.unpack('!8d', data)
                    timestamp, angle, angvel, pwm_l, pwm_r, M_ff, M_fb, protection = values
                    
                    self.queue.put({
                        'timestamp': timestamp,
                        'angle': angle,
                        'angvel': angvel,
                        'pwm_left': int(pwm_l),
                        'pwm_right': int(pwm_r),
                        'M_ff': M_ff,
                        'M_fb': M_fb,
                        'protection': protection > 0.5
                    })
                    
                    self.packet_count += 1
                    self.last_packet_time = time.time()
                    
            except socket.timeout:
                pass
            except Exception as e:
                if self.running:
                    print(f"接收错误: {e}")


class RealtimePlotter:
    """实时绘图器"""
    
    def __init__(self, receiver: DataReceiver):
        self.receiver = receiver
        self.window_sec = PLOT_WINDOW_SEC
        
        # 数据缓冲区
        self.time_data = deque(maxlen=MAX_SAMPLES)
        self.angle_data = deque(maxlen=MAX_SAMPLES)
        self.angvel_data = deque(maxlen=MAX_SAMPLES)
        self.pwm_left_data = deque(maxlen=MAX_SAMPLES)
        self.pwm_right_data = deque(maxlen=MAX_SAMPLES)
        self.M_ff_data = deque(maxlen=MAX_SAMPLES)
        self.M_fb_data = deque(maxlen=MAX_SAMPLES)
        self.protection_data = deque(maxlen=MAX_SAMPLES)
        
        # 状态
        self.paused = False
        self.connected = False
        self.last_update = 0
        
    def run(self):
        """运行绘图器"""
        # 创建图形
        self.fig, self.axes = plt.subplots(4, 1, sharex=True, figsize=(12, 10))
        
        # 1. 角度子图
        ax_angle = self.axes[0]
        self.line_angle, = ax_angle.plot([], [], 'b-', lw=1.5, label='Roll Angle')
        ax_angle.set_ylabel('Angle (deg)')
        ax_angle.set_ylim(-45, 45)
        ax_angle.grid(True, alpha=0.3, linestyle='--')
        ax_angle.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        ax_angle.axhline(y=2.8, color='red', linestyle=':', alpha=0.5, label='Deadzone')
        ax_angle.axhline(y=-2.8, color='red', linestyle=':', alpha=0.5)
        ax_angle.legend(loc='upper right')
        self.text_angle = ax_angle.text(0.02, 0.95, '', transform=ax_angle.transAxes,
                                        verticalalignment='top', fontsize=11,
                                        bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='gray', alpha=0.9))
        
        # 2. 角速度子图
        ax_angvel = self.axes[1]
        self.line_angvel, = ax_angvel.plot([], [], 'orange', lw=1.5, label='Angular Velocity')
        ax_angvel.set_ylabel('Angular Vel (deg/s)')
        ax_angvel.set_ylim(-90, 90)
        ax_angvel.grid(True, alpha=0.3, linestyle='--')
        ax_angvel.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        self.text_angvel = ax_angvel.text(0.02, 0.95, '', transform=ax_angvel.transAxes,
                                          verticalalignment='top', fontsize=11,
                                          bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='gray', alpha=0.9))
        
        # 3. 力矩子图
        ax_torque = self.axes[2]
        self.line_Mff, = ax_torque.plot([], [], 'g-', lw=1.5, label='M_ff (Feedforward)')
        self.line_Mfb, = ax_torque.plot([], [], 'r-', lw=1.5, label='M_fb (Feedback)')
        ax_torque.set_ylabel('Torque (N·m)')
        ax_torque.set_ylim(-150, 150)
        ax_torque.grid(True, alpha=0.3, linestyle='--')
        ax_torque.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        ax_torque.legend(loc='upper right')
        self.text_torque = ax_torque.text(0.02, 0.95, '', transform=ax_torque.transAxes,
                                          verticalalignment='top', fontsize=11,
                                          bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='gray', alpha=0.9))
        
        # 4. PWM 子图
        ax_pwm = self.axes[3]
        self.line_pwm_left, = ax_pwm.plot([], [], 'g-', lw=1.5, label='Left PWM')
        self.line_pwm_right, = ax_pwm.plot([], [], 'r-', lw=1.5, label='Right PWM')
        ax_pwm.set_ylabel('Motor PWM (us)')
        ax_pwm.set_xlabel('Time (s)')
        ax_pwm.set_ylim(900, 2100)
        ax_pwm.grid(True, alpha=0.3, linestyle='--')
        ax_pwm.axhline(y=PWM_NEUTRAL, color='gray', linestyle='--', alpha=0.5)
        ax_pwm.legend(loc='upper right')
        self.text_pwm = ax_pwm.text(0.02, 0.95, '', transform=ax_pwm.transAxes,
                                    verticalalignment='top', fontsize=11,
                                    bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='gray', alpha=0.9))
        
        # 状态显示
        self.status_text = self.fig.text(0.5, 0.98, '等待数据...', ha='center', va='top',
                                         fontsize=12, color='gray')
        
        # 滑块 - 窗口大小
        slider_ax = self.fig.add_axes([0.15, 0.02, 0.55, 0.02])
        self.window_slider = Slider(slider_ax, 'Window (s)', 2.0, PLOT_MAX_WINDOW_SEC,
                                    valinit=PLOT_WINDOW_SEC, valstep=1.0)
        self.window_slider.on_changed(self._on_slider_change)
        
        # 按钮 - 暂停/继续
        pause_ax = self.fig.add_axes([0.75, 0.02, 0.08, 0.03])
        self.pause_button = Button(pause_ax, 'Pause')
        self.pause_button.on_clicked(self._on_pause_click)
        
        # 按钮 - 清除
        clear_ax = self.fig.add_axes([0.85, 0.02, 0.08, 0.03])
        self.clear_button = Button(clear_ax, 'Clear')
        self.clear_button.on_clicked(self._on_clear_click)
        
        self.fig.subplots_adjust(left=0.08, right=0.98, top=0.95, bottom=0.1, hspace=0.25)
        self.fig.suptitle('SUP Stabilizer Remote Monitor', fontsize=14)
        
        # 启动动画
        self.anim = FuncAnimation(self.fig, self._update, interval=UPDATE_INTERVAL, 
                                  blit=False, cache_frame_data=False)
        
        plt.show()
    
    def _on_slider_change(self, val):
        self.window_sec = val
    
    def _on_pause_click(self, event):
        self.paused = not self.paused
        self.pause_button.label.set_text('Resume' if self.paused else 'Pause')
    
    def _on_clear_click(self, event):
        self.time_data.clear()
        self.angle_data.clear()
        self.angvel_data.clear()
        self.pwm_left_data.clear()
        self.pwm_right_data.clear()
        self.M_ff_data.clear()
        self.M_fb_data.clear()
        self.protection_data.clear()
    
    def _update(self, frame):
        # 获取新数据
        if not self.paused:
            new_data = self.receiver.get_data()
            for d in new_data:
                self.time_data.append(d['timestamp'])
                self.angle_data.append(d['angle'])
                self.angvel_data.append(d['angvel'])
                self.pwm_left_data.append(d['pwm_left'])
                self.pwm_right_data.append(d['pwm_right'])
                self.M_ff_data.append(d['M_ff'])
                self.M_fb_data.append(d['M_fb'])
                self.protection_data.append(d['protection'])
        
        # 检查连接状态
        now = time.time()
        if self.receiver.last_packet_time > 0:
            time_since_last = now - self.receiver.last_packet_time
            if time_since_last < 1.0:
                self.connected = True
                self.status_text.set_text(f'● 已连接 | 数据包: {self.receiver.packet_count} | {"暂停" if self.paused else "运行中"}')
                self.status_text.set_color('green')
            else:
                self.connected = False
                self.status_text.set_text(f'○ 连接断开 ({time_since_last:.1f}s)')
                self.status_text.set_color('red')
        
        if not self.time_data:
            return
        
        # 裁剪数据到窗口
        t_list = list(self.time_data)
        cutoff = t_list[-1] - self.window_sec
        
        # 找到裁剪起始索引
        start_idx = 0
        for i, t in enumerate(t_list):
            if t >= cutoff:
                start_idx = i
                break
        
        t_window = t_list[start_idx:]
        angle_window = list(self.angle_data)[start_idx:]
        angvel_window = list(self.angvel_data)[start_idx:]
        pwm_l_window = list(self.pwm_left_data)[start_idx:]
        pwm_r_window = list(self.pwm_right_data)[start_idx:]
        Mff_window = list(self.M_ff_data)[start_idx:]
        Mfb_window = list(self.M_fb_data)[start_idx:]
        
        if not t_window:
            return
        
        # 更新曲线
        self.line_angle.set_data(t_window, angle_window)
        self.line_angvel.set_data(t_window, angvel_window)
        self.line_Mff.set_data(t_window, Mff_window)
        self.line_Mfb.set_data(t_window, Mfb_window)
        self.line_pwm_left.set_data(t_window, pwm_l_window)
        self.line_pwm_right.set_data(t_window, pwm_r_window)
        
        # 更新 X 轴范围
        x_min, x_max = t_window[0], t_window[-1]
        for ax in self.axes:
            ax.set_xlim(x_min, x_max)
        
        # 更新文本显示
        self.text_angle.set_text(f'Angle: {angle_window[-1]:+.2f}°')
        self.text_angvel.set_text(f'ω: {angvel_window[-1]:+.2f}°/s')
        self.text_torque.set_text(f'M_ff: {Mff_window[-1]:+.1f}  M_fb: {Mfb_window[-1]:+.1f}')
        self.text_pwm.set_text(f'L: {pwm_l_window[-1]}us  R: {pwm_r_window[-1]}us')
        
        return (self.line_angle, self.line_angvel, self.line_Mff, self.line_Mfb,
                self.line_pwm_left, self.line_pwm_right)


def main():
    print("=" * 50)
    print("SUP 稳定器远程监视器")
    print("=" * 50)
    print(f"监听 UDP 端口: {UDP_PORT}")
    print("等待树莓派数据...")
    print("按 Ctrl+C 退出")
    print("=" * 50)
    
    receiver = DataReceiver()
    receiver.start()
    
    plotter = RealtimePlotter(receiver)
    
    try:
        plotter.run()
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        receiver.stop()


if __name__ == "__main__":
    main()
