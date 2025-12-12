#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
数据发送器 - 通过 UDP 发送 IMU 和电机数据到局域网
运行在树莓派上，与 feedforward_test.py 配合使用
"""

import socket
import struct
import time
import threading
from queue import SimpleQueue, Empty

# ==================== 网络配置 ====================
UDP_IP = "0.0.0.0"           # 监听所有网卡（用于接收命令）
UDP_PORT = 5005              # 发送端口
BROADCAST_IP = "255.255.255.255"  # 广播地址（也可以指定电脑IP）
SEND_RATE = 50               # 发送频率 Hz

class DataSender:
    """通过 UDP 发送数据的类"""
    
    def __init__(self, target_ip=BROADCAST_IP, port=UDP_PORT):
        self.target_ip = target_ip
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.queue = SimpleQueue()
        self.running = False
        self.send_thread = None
        
    def start(self):
        """启动发送线程"""
        self.running = True
        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.send_thread.start()
        print(f"数据发送器已启动，目标: {self.target_ip}:{self.port}")
        
    def stop(self):
        """停止发送线程"""
        self.running = False
        if self.send_thread:
            self.send_thread.join(timeout=1.0)
        self.socket.close()
        print("数据发送器已停止")
    
    def record(self, timestamp, angle_deg, angular_velocity_deg_s, pwm_left, pwm_right,
               M_ff=0.0, M_fb=0.0, bangbang_protection=False):
        """
        记录一帧数据（由控制循环调用）
        
        参数:
            timestamp: 时间戳（秒）
            angle_deg: 角度（度）
            angular_velocity_deg_s: 角速度（度/秒）
            pwm_left: 左电机PWM
            pwm_right: 右电机PWM
            M_ff: 前馈力矩
            M_fb: 反馈力矩
            bangbang_protection: 是否处于防护状态
        """
        self.queue.put((timestamp, angle_deg, angular_velocity_deg_s, 
                        pwm_left, pwm_right, M_ff, M_fb, bangbang_protection))
    
    def _send_loop(self):
        """发送循环"""
        send_interval = 1.0 / SEND_RATE
        last_send = 0
        
        while self.running:
            try:
                # 批量获取数据，只发送最新的
                latest_data = None
                while True:
                    try:
                        latest_data = self.queue.get_nowait()
                    except Empty:
                        break
                
                if latest_data is not None:
                    now = time.time()
                    if now - last_send >= send_interval:
                        self._send_packet(latest_data)
                        last_send = now
                else:
                    time.sleep(0.001)
                    
            except Exception as e:
                print(f"发送错误: {e}")
                time.sleep(0.1)
    
    def _send_packet(self, data):
        """
        发送数据包
        格式: 8个float (64字节) = timestamp, angle, angvel, pwm_l, pwm_r, M_ff, M_fb, protection
        """
        timestamp, angle, angvel, pwm_l, pwm_r, M_ff, M_fb, protection = data
        
        # 打包数据：8个double
        packet = struct.pack('!8d', 
                             timestamp, 
                             angle, 
                             angvel, 
                             float(pwm_l), 
                             float(pwm_r),
                             M_ff,
                             M_fb,
                             1.0 if protection else 0.0)
        
        try:
            self.socket.sendto(packet, (self.target_ip, self.port))
        except Exception as e:
            print(f"UDP发送失败: {e}")


# ==================== 独立测试 ====================
if __name__ == "__main__":
    import math
    
    print("数据发送器测试模式")
    print("发送模拟数据到局域网...")
    
    sender = DataSender()
    sender.start()
    
    start_time = time.time()
    
    try:
        while True:
            t = time.time() - start_time
            
            # 模拟数据
            angle = 10 * math.sin(t * 0.5)  # 10度振幅，周期约12秒
            angvel = 5 * math.cos(t * 0.5)  # 角速度
            pwm_left = 1500 + int(200 * math.sin(t))
            pwm_right = 1500 - int(200 * math.sin(t))
            M_ff = 50 * math.sin(t * 0.5)
            M_fb = 20 * math.cos(t * 0.3)
            protection = abs(angle) < 2.8
            
            sender.record(t, angle, angvel, pwm_left, pwm_right, M_ff, M_fb, protection)
            
            print(f"\r发送: 角度={angle:+6.2f}° 角速度={angvel:+6.2f}°/s PWM_L={pwm_left} PWM_R={pwm_right}", end="")
            
            time.sleep(0.02)  # 50Hz
            
    except KeyboardInterrupt:
        print("\n停止发送")
    finally:
        sender.stop()
