#!/usr/bin/env python3
"""
Raspberry Pi Temperature Monitor with Stress Test
Continuously monitors CPU/SoC temperature using vcgencmd
Optionally applies CPU stress test to observe thermal behavior
"""

import subprocess
import time
import sys
import os
import multiprocessing
import argparse
from datetime import datetime

# Configuration
INTERVAL = 1.0  # seconds between readings
WARN_TEMP = 70.0  # warning threshold (°C)
CRITICAL_TEMP = 80.0  # critical threshold (°C)

# Stress test workers
stress_processes = []

def cpu_stress_worker():
    """CPU stress worker - performs intensive calculations"""
    while True:
        # Intensive floating point operations
        x = 0
        for i in range(1000000):
            x += i * i * 0.123456789
            x = x ** 0.5

def start_stress_test(num_cores=None):
    """Start CPU stress test on specified number of cores"""
    global stress_processes
    
    if num_cores is None:
        num_cores = multiprocessing.cpu_count()
    
    print(f"\n  🔥 Starting stress test on {num_cores} CPU core(s)...")
    
    for _ in range(num_cores):
        p = multiprocessing.Process(target=cpu_stress_worker)
        p.daemon = True
        p.start()
        stress_processes.append(p)
    
    return num_cores

def stop_stress_test():
    """Stop all stress test workers"""
    global stress_processes
    
    for p in stress_processes:
        p.terminate()
        p.join(timeout=1)
    
    stress_processes = []
    print("\n  ⏹  Stress test stopped.")

def get_temperature():
    """Get CPU temperature using vcgencmd"""
    try:
        result = subprocess.run(
            ['vcgencmd', 'measure_temp'],
            capture_output=True,
            text=True,
            timeout=5
        )
        # Parse "temp=45.0'C" format
        temp_str = result.stdout.strip()
        temp = float(temp_str.replace("temp=", "").replace("'C", ""))
        return temp
    except Exception as e:
        print(f"Error reading temperature: {e}")
        return None

def get_status_icon(temp):
    """Return status icon based on temperature"""
    if temp < WARN_TEMP:
        return "✓"  # OK
    elif temp < CRITICAL_TEMP:
        return "⚠"  # Warning
    else:
        return "🔥"  # Critical

def get_cpu_usage():
    """Get current CPU usage percentage"""
    try:
        # Read /proc/stat for CPU usage
        with open('/proc/stat', 'r') as f:
            line = f.readline()
        parts = line.split()
        idle = int(parts[4])
        total = sum(int(p) for p in parts[1:])
        return idle, total
    except:
        return None, None

def main():
    parser = argparse.ArgumentParser(description='Raspberry Pi Temperature Monitor with Stress Test')
    parser.add_argument('-s', '--stress', action='store_true', help='Enable CPU stress test')
    parser.add_argument('-c', '--cores', type=int, default=None, help='Number of cores to stress (default: all)')
    parser.add_argument('-d', '--duration', type=int, default=None, help='Stress test duration in seconds (default: unlimited)')
    parser.add_argument('-i', '--interval', type=float, default=1.0, help='Update interval in seconds (default: 1.0)')
    args = parser.parse_args()
    
    global INTERVAL
    INTERVAL = args.interval
    
    num_cores = multiprocessing.cpu_count()
    
    print("=" * 60)
    print("  Raspberry Pi Temperature Monitor + Stress Test")
    print("=" * 60)
    print(f"  CPU Cores: {num_cores}")
    print(f"  Warn: {WARN_TEMP}°C | Critical: {CRITICAL_TEMP}°C")
    print(f"  Update interval: {INTERVAL}s")
    if args.stress:
        stress_cores = args.cores if args.cores else num_cores
        print(f"  Stress test: ENABLED ({stress_cores} cores)")
        if args.duration:
            print(f"  Stress duration: {args.duration}s")
    else:
        print("  Stress test: DISABLED (use -s to enable)")
    print("  Press Ctrl+C to stop")
    print("=" * 60)
    print()

    min_temp = float('inf')
    max_temp = float('-inf')
    readings = []
    stress_active = False
    start_time = time.time()
    prev_idle, prev_total = get_cpu_usage()
    
    # Start stress test if requested
    if args.stress:
        stress_cores = start_stress_test(args.cores)
        stress_active = True

    try:
        while True:
            elapsed = time.time() - start_time
            
            # Calculate CPU usage
            curr_idle, curr_total = get_cpu_usage()
            if prev_idle is not None and curr_idle is not None:
                idle_delta = curr_idle - prev_idle
                total_delta = curr_total - prev_total
                cpu_usage = 100.0 * (1.0 - idle_delta / total_delta) if total_delta > 0 else 0
            else:
                cpu_usage = 0
            prev_idle, prev_total = curr_idle, curr_total
            
            # Check if stress duration exceeded
            if args.stress and args.duration and elapsed >= args.duration and stress_active:
                stop_stress_test()
                stress_active = False
                print("\n  ⏱  Stress test duration completed. Continuing monitoring...\n")
            
            temp = get_temperature()
            
            if temp is not None:
                # Update stats
                min_temp = min(min_temp, temp)
                max_temp = max(max_temp, temp)
                readings.append(temp)
                
                # Keep only last 60 readings for average
                if len(readings) > 60:
                    readings.pop(0)
                avg_temp = sum(readings) / len(readings)
                
                # Get status
                icon = get_status_icon(temp)
                timestamp = datetime.now().strftime("%H:%M:%S")
                
                # Print with color coding
                if temp >= CRITICAL_TEMP:
                    color = "\033[91m"  # Red
                elif temp >= WARN_TEMP:
                    color = "\033[93m"  # Yellow
                else:
                    color = "\033[92m"  # Green
                reset = "\033[0m"
                
                stress_indicator = "🔥" if stress_active else "  "
                
                print(f"\r{timestamp} | {icon} Temp: {color}{temp:5.1f}°C{reset} | "
                      f"CPU: {cpu_usage:5.1f}% | "
                      f"Min: {min_temp:.1f}°C | Max: {max_temp:.1f}°C | "
                      f"Avg: {avg_temp:.1f}°C {stress_indicator}", end="", flush=True)
            
            time.sleep(INTERVAL)
            
    except KeyboardInterrupt:
        print("\n")
        
        # Stop stress test if running
        if stress_active:
            stop_stress_test()
        
        print("=" * 60)
        print("  Session Summary")
        print("=" * 60)
        if readings:
            print(f"  Duration:  {elapsed:.1f}s")
            print(f"  Minimum:   {min_temp:.1f}°C")
            print(f"  Maximum:   {max_temp:.1f}°C")
            print(f"  Average:   {sum(readings)/len(readings):.1f}°C")
            print(f"  Readings:  {len(readings)}")
            if args.stress:
                print(f"  Stress:    {'Completed' if args.duration else 'Interrupted'}")
        print("=" * 60)
        print("  Monitor stopped.")
        sys.exit(0)

if __name__ == "__main__":
    main()
