#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Quick launch script for motor control stability analysis
Runs both single-test and comparative analysis
"""

import subprocess
import sys
import os

def run_analysis():
    """Run complete analysis pipeline"""
    
    print("\n" + "=" * 100)
    print("MOTOR CONTROL STABILITY ANALYSIS PIPELINE")
    print("=" * 100)
    
    python_exe = "/home/pi/Desktop/motor_env/bin/python"
    
    # Check environment
    if not os.path.exists(python_exe):
        print(f"\n✗ Python executable not found: {python_exe}")
        sys.exit(1)
    
    # Step 1: Single test analysis
    print("\n[Step 1/2] Running single-test stability analysis...")
    print("-" * 100)
    
    result = subprocess.run(
        [python_exe, "/home/pi/Desktop/system_stability_analyzer.py"],
        cwd="/home/pi/Desktop"
    )
    
    if result.returncode != 0:
        print("\n✗ Single-test analysis failed")
        sys.exit(1)
    
    # Step 2: Comparative analysis
    print("\n[Step 2/2] Running comparative stability analysis...")
    print("-" * 100)
    
    result = subprocess.run(
        [python_exe, "/home/pi/Desktop/stability_comparator.py"],
        cwd="/home/pi/Desktop"
    )
    
    if result.returncode != 0:
        print("\n✗ Comparative analysis failed")
        sys.exit(1)
    
    # Summary
    print("\n" + "=" * 100)
    print("✓ ANALYSIS COMPLETE")
    print("=" * 100)
    print("\nOutput files:")
    print("  Single-test analysis:")
    print("    - analysis_results/figures/*.png        (Detailed 8-panel charts)")
    print("    - analysis_results/stability_report.txt (Complete metrics)")
    print("    - analysis_results/analysis_data.json   (Machine-readable data)")
    print("\n  Comparative analysis:")
    print("    - comparison_results/comparison_analysis.png  (9-panel comparison)")
    print("    - comparison_results/comparison_report.txt    (Statistical analysis)")
    print("\nFor help interpreting results, see: STABILITY_ANALYSIS_GUIDE.md")
    print("=" * 100 + "\n")

if __name__ == "__main__":
    run_analysis()
