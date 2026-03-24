#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32 MicroPython - boot.py
系统启动脚本，在 ESP32 启动时自动运行
"""

import machine
import network
import time

# ==================== 配置 ====================
WIFI_SSID = "your_wifi_ssid"  # 修改为你的 WiFi SSID
WIFI_PASSWORD = "your_password"  # 修改为你的 WiFi 密码
AUTO_START = True  # 是否自动启动控制器

# ==================== WiFi 连接 ====================
def connect_wifi():
    """连接到 WiFi"""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    if not wlan.isconnected():
        print(f"正在连接 WiFi: {WIFI_SSID}...")
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        
        # 等待连接
        timeout = 20
        while not wlan.isconnected() and timeout > 0:
            time.sleep(1)
            timeout -= 1
            print(f"  连接中... ({timeout}s)")
        
        if wlan.isconnected():
            print("WiFi 已连接!")
            print(f"IP 地址: {wlan.ifconfig()[0]}")
            return True
        else:
            print("WiFi 连接失败!")
            return False
    else:
        print(f"已连接到 {WIFI_SSID}")
        print(f"IP 地址: {wlan.ifconfig()[0]}")
        return True

# ==================== 启动主程序 ====================
def main():
    """启动主程序"""
    print("\n" + "="*50)
    print("ESP32 MicroPython 启动")
    print("="*50)
    
    # 连接 WiFi
    connect_wifi()
    
    # 自动启动主程序
    if AUTO_START:
        print("\n自动启动控制器...")
        try:
            import main
        except ImportError:
            print("Error: main.py 未找到!")
        except Exception as e:
            print(f"Error: {e}")
    else:
        print("\n自动启动已禁用，可通过 REPL 手动运行 main.py")

# 执行启动
if __name__ == "__main__":
    main()
