#!/usr/bin/env python3
"""
IMU data collector and logger
"""

import mpu6050
import time
import csv
from datetime import datetime

def simple_imu_logger(sample_rate=50, duration=None):
    try:
        imu = mpu6050.mpu6050(0x68)
    except:
        imu = mpu6050.mpu6050(0x69)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"imu_simple_{timestamp}.csv"
    
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['timestamp', 'gyro_x', 'gyro_y', 'gyro_z', 'acc_x', 'acc_y', 'acc_z', 'temp'])
        
        print(f"Start to collect imu data to: {filename}")
        print("Press Ctrl-C to stop recording.")
        
        start_time = time.time()
        sample_count = 0
        
        try:
            while True:
                if duration and (time.time() - start_time) >= duration:
                    break
                
                gyro = imu.get_gyro_data()
                acc = imu.get_accel_data()
                temp = imu.get_temp()
                
                writer.writerow([
                    datetime.now().isoformat(),
                    gyro['x'], gyro['y'], gyro['z'],
                    acc['x'], acc['y'], acc['z'],
                    temp
                ])
                
                sample_count += 1
                
                if sample_count % 50 == 0:
                    elapsed = time.time() - start_time
                    current_rate = sample_count / elapsed
                    print(f"Sampling: {sample_count}, Rate: {current_rate:.1f} Hz")
                
                time.sleep(1.0 / sample_rate)
                
        except KeyboardInterrupt:
            pass
        
        total_time = time.time() - start_time
        avg_rate = sample_count / total_time
        
        print(f"\nSuccessfully Collected!")
        print(f"Sample count: {sample_count}")
        print(f"Time: {total_time:.2f} 秒")
        print(f"Rate: {avg_rate:.2f} Hz")
        print(f"File: {filename}")

if __name__ == "__main__":
    simple_imu_logger(sample_rate=100)  