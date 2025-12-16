#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
System Stability Comparison - Compare specific metrics across tests
"""

import os
import json
import glob
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['font.family'] = 'DejaVu Sans'
matplotlib.rcParams['axes.unicode_minus'] = False
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path

class StabilityComparator:
    """Compare stability metrics across test runs"""
    
    def __init__(self, json_file="analysis_results/analysis_data.json", 
                 output_dir="comparison_results"):
        self.json_file = json_file
        self.output_dir = output_dir
        Path(self.output_dir).mkdir(exist_ok=True)
    
    def load_results(self):
        """Load analysis results from JSON"""
        if not os.path.exists(self.json_file):
            print(f"✗ JSON file not found: {self.json_file}")
            return None
        
        with open(self.json_file, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        return data
    
    def extract_metrics(self, data):
        """Extract metrics for comparison"""
        comparison_data = {}
        
        for test_name, metrics in data.items():
            comparison_data[test_name] = {
                'roll_mean': metrics['roll']['mean_roll'],
                'roll_std': metrics['roll']['std_roll'],
                'roll_range': metrics['roll']['roll_range'],
                'roll_rms': metrics['roll']['rms_roll'],
                'omega_max': metrics['omega']['max_omega'],
                'omega_std': metrics['omega']['std_omega'],
                'accel_max': metrics['accel']['max_accel'],
                'accel_std': metrics['accel']['std_accel'],
                'dominant_freq': metrics['frequency']['dominant_freq'],
                'pwm_sync_error': metrics['pwm']['pwm_sync_error'],
            }
        
        return comparison_data
    
    def plot_comparison(self, comparison_data):
        """Create comparison visualization"""
        df = pd.DataFrame(comparison_data).T
        test_names = [name.replace('feedforward_dual_imu_', '').replace('.csv', '') 
                      for name in df.index]
        df.index = test_names
        
        fig = plt.figure(figsize=(18, 12))
        
        # 1. Roll Angle Metrics
        ax1 = plt.subplot(3, 3, 1)
        x = np.arange(len(df))
        width = 0.25
        ax1.bar(x - width, df['roll_mean'], width, label='Mean', color='#2E86AB', alpha=0.8)
        ax1.bar(x, df['roll_std'], width, label='Std Dev', color='#F18F01', alpha=0.8)
        ax1.bar(x + width, df['roll_rms'], width, label='RMS', color='#A23B72', alpha=0.8)
        ax1.set_xlabel('Test')
        ax1.set_ylabel('Angle (deg)')
        ax1.set_title('Roll Angle Metrics Comparison')
        ax1.set_xticks(x)
        ax1.set_xticklabels(df.index, rotation=45, ha='right', fontsize=9)
        ax1.legend()
        ax1.grid(True, alpha=0.3, axis='y')
        
        # 2. Roll Range
        ax2 = plt.subplot(3, 3, 2)
        colors = ['#2E86AB' if v < np.median(df['roll_range']) else '#F18F01' 
                  for v in df['roll_range']]
        ax2.bar(x, df['roll_range'], color=colors, alpha=0.8, edgecolor='black')
        ax2.set_xlabel('Test')
        ax2.set_ylabel('Range (deg)')
        ax2.set_title('Roll Angle Range (Lower is Better)')
        ax2.set_xticks(x)
        ax2.set_xticklabels(df.index, rotation=45, ha='right', fontsize=9)
        ax2.axhline(y=df['roll_range'].mean(), color='r', linestyle='--', 
                   label=f'Mean: {df["roll_range"].mean():.2f}°')
        ax2.legend()
        ax2.grid(True, alpha=0.3, axis='y')
        
        # 3. Angular Velocity
        ax3 = plt.subplot(3, 3, 3)
        ax3.bar(x - width/2, df['omega_max'], width, label='Max', color='#A23B72', alpha=0.8)
        ax3.bar(x + width/2, df['omega_std'], width, label='Std Dev', color='#2E86AB', alpha=0.8)
        ax3.set_xlabel('Test')
        ax3.set_ylabel('Angular Velocity (deg/s)')
        ax3.set_title('Angular Velocity Characteristics')
        ax3.set_xticks(x)
        ax3.set_xticklabels(df.index, rotation=45, ha='right', fontsize=9)
        ax3.legend()
        ax3.grid(True, alpha=0.3, axis='y')
        
        # 4. Angular Acceleration
        ax4 = plt.subplot(3, 3, 4)
        ax4.bar(x - width/2, df['accel_max'], width, label='Max', color='#F18F01', alpha=0.8)
        ax4.bar(x + width/2, df['accel_std'], width, label='Std Dev', color='#2E86AB', alpha=0.8)
        ax4.set_xlabel('Test')
        ax4.set_ylabel('Angular Acceleration (deg/s²)')
        ax4.set_title('Angular Acceleration Characteristics')
        ax4.set_xticks(x)
        ax4.set_xticklabels(df.index, rotation=45, ha='right', fontsize=9)
        ax4.legend()
        ax4.grid(True, alpha=0.3, axis='y')
        
        # 5. Dominant Frequency
        ax5 = plt.subplot(3, 3, 5)
        colors = ['#2E86AB' if v < 1 else '#F18F01' for v in df['dominant_freq']]
        ax5.bar(x, df['dominant_freq'], color=colors, alpha=0.8, edgecolor='black')
        ax5.set_xlabel('Test')
        ax5.set_ylabel('Frequency (Hz)')
        ax5.set_title('Dominant Frequency (Lower is More Stable)')
        ax5.set_xticks(x)
        ax5.set_xticklabels(df.index, rotation=45, ha='right', fontsize=9)
        ax5.grid(True, alpha=0.3, axis='y')
        
        # 6. PWM Synchronization
        ax6 = plt.subplot(3, 3, 6)
        colors = ['#2E86AB' if v < np.median(df['pwm_sync_error']) else '#F18F01' 
                  for v in df['pwm_sync_error']]
        ax6.bar(x, df['pwm_sync_error'], color=colors, alpha=0.8, edgecolor='black')
        ax6.set_xlabel('Test')
        ax6.set_ylabel('Error (us)')
        ax6.set_title('PWM Synchronization Error')
        ax6.set_xticks(x)
        ax6.set_xticklabels(df.index, rotation=45, ha='right', fontsize=9)
        ax6.axhline(y=df['pwm_sync_error'].mean(), color='r', linestyle='--',
                   label=f'Mean: {df["pwm_sync_error"].mean():.1f} us')
        ax6.legend()
        ax6.grid(True, alpha=0.3, axis='y')
        
        # 7. Overall Stability Index (multi-metric comparison)
        ax7 = plt.subplot(3, 3, 7)
        
        # Normalize metrics to 0-1 range for comparison
        normalized = pd.DataFrame()
        for col in ['roll_std', 'roll_range', 'omega_max', 'accel_max', 'dominant_freq']:
            min_val = df[col].min()
            max_val = df[col].max()
            if max_val > min_val:
                normalized[col] = (df[col] - min_val) / (max_val - min_val)
            else:
                normalized[col] = 0.5
        
        # Average of normalized values (lower is better)
        stability_index = normalized.mean(axis=1)
        colors = ['#2E86AB' if v < np.median(stability_index) else '#F18F01' 
                  for v in stability_index]
        ax7.bar(x, stability_index, color=colors, alpha=0.8, edgecolor='black')
        ax7.set_xlabel('Test')
        ax7.set_ylabel('Relative Stability')
        ax7.set_title('Composite Stability Index (Lower is Better)')
        ax7.set_xticks(x)
        ax7.set_xticklabels(df.index, rotation=45, ha='right', fontsize=9)
        ax7.grid(True, alpha=0.3, axis='y')
        
        # 8. Metrics Heatmap
        ax8 = plt.subplot(3, 3, 8)
        
        # Normalize for heatmap
        heatmap_data = df[['roll_std', 'roll_range', 'omega_max', 'accel_max', 'dominant_freq']].copy()
        heatmap_normalized = (heatmap_data - heatmap_data.min()) / (heatmap_data.max() - heatmap_data.min())
        
        sns.heatmap(heatmap_normalized.T, annot=False, cmap='RdYlGn_r', 
                   ax=ax8, cbar_kws={'label': 'Normalized Value'})
        ax8.set_title('Normalized Metrics Heatmap')
        ax8.set_xticklabels(df.index, rotation=45, ha='right', fontsize=9)
        ax8.set_yticklabels(['Roll Std', 'Roll Range', 'Omega Max', 'Accel Max', 'Dom. Freq'], 
                           rotation=0)
        
        # 9. Summary Statistics
        ax9 = plt.subplot(3, 3, 9)
        ax9.axis('off')
        
        summary_text = "COMPARATIVE STATISTICS\n"
        summary_text += "─" * 40 + "\n\n"
        
        summary_text += "Roll Std Dev:\n"
        summary_text += f"  Best:  {df['roll_std'].idxmin()} ({df['roll_std'].min():.3f}°)\n"
        summary_text += f"  Worst: {df['roll_std'].idxmax()} ({df['roll_std'].max():.3f}°)\n\n"
        
        summary_text += "Angular Velocity Max:\n"
        summary_text += f"  Best:  {df['omega_max'].idxmin()} ({df['omega_max'].min():.3f}°/s)\n"
        summary_text += f"  Worst: {df['omega_max'].idxmax()} ({df['omega_max'].max():.3f}°/s)\n\n"
        
        summary_text += "Roll Range:\n"
        summary_text += f"  Best:  {df['roll_range'].idxmin()} ({df['roll_range'].min():.3f}°)\n"
        summary_text += f"  Worst: {df['roll_range'].idxmax()} ({df['roll_range'].max():.3f}°)\n"
        
        ax9.text(0.05, 0.95, summary_text, fontsize=10, family='monospace',
                verticalalignment='top', bbox=dict(boxstyle='round', 
                facecolor='wheat', alpha=0.5))
        
        plt.tight_layout()
        output_file = os.path.join(self.output_dir, 'comparison_analysis.png')
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"✓ Comparison chart saved: {output_file}")
        plt.close()
        
        return df
    
    def generate_comparison_report(self, comparison_data):
        """Generate detailed comparison report"""
        df = pd.DataFrame(comparison_data).T
        
        report_file = os.path.join(self.output_dir, 'comparison_report.txt')
        
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write("=" * 120 + "\n")
            f.write("STABILITY COMPARISON REPORT\n")
            f.write("Motor Control System Across Multiple Tests\n")
            f.write("=" * 120 + "\n\n")
            
            # Raw data table
            f.write("METRICS COMPARISON TABLE\n")
            f.write("-" * 120 + "\n")
            f.write(f"{'Test':<30} {'Roll Std':<12} {'Roll Rng':<12} {'Omega Max':<12} "
                   f"{'Accel Max':<12} {'Dom Freq':<12} {'PWM Err':<12}\n")
            f.write("-" * 120 + "\n")
            
            for test_name in df.index:
                row = df.loc[test_name]
                f.write(
                    f"{test_name:<30} "
                    f"{row['roll_std']:<12.4f} "
                    f"{row['roll_range']:<12.4f} "
                    f"{row['omega_max']:<12.4f} "
                    f"{row['accel_max']:<12.4f} "
                    f"{row['dominant_freq']:<12.4f} "
                    f"{row['pwm_sync_error']:<12.4f}\n"
                )
            
            f.write("\n" + "=" * 120 + "\n")
            f.write("STATISTICAL ANALYSIS\n")
            f.write("=" * 120 + "\n\n")
            
            metrics_to_analyze = ['roll_std', 'roll_range', 'omega_max', 'accel_max', 
                                 'dominant_freq', 'pwm_sync_error']
            
            for metric in metrics_to_analyze:
                f.write(f"\n[{metric.upper()}]\n")
                f.write("-" * 80 + "\n")
                f.write(f"  Mean:     {df[metric].mean():>12.4f}\n")
                f.write(f"  Std Dev:  {df[metric].std():>12.4f}\n")
                f.write(f"  Min:      {df[metric].min():>12.4f}  (Test: {df[metric].idxmin()})\n")
                f.write(f"  Max:      {df[metric].max():>12.4f}  (Test: {df[metric].idxmax()})\n")
                f.write(f"  Range:    {df[metric].max() - df[metric].min():>12.4f}\n")
            
            f.write("\n" + "=" * 120 + "\n")
            f.write("KEY FINDINGS\n")
            f.write("=" * 120 + "\n")
            
            f.write("\n1. ROLL ANGLE STABILITY\n")
            best_roll_std = df['roll_std'].idxmin()
            worst_roll_std = df['roll_std'].idxmax()
            improvement = ((df.loc[worst_roll_std, 'roll_std'] - df.loc[best_roll_std, 'roll_std']) 
                          / df.loc[worst_roll_std, 'roll_std'] * 100)
            
            f.write(f"   Most stable: {best_roll_std} (Std: {df.loc[best_roll_std, 'roll_std']:.4f}°)\n")
            f.write(f"   Least stable: {worst_roll_std} (Std: {df.loc[worst_roll_std, 'roll_std']:.4f}°)\n")
            f.write(f"   Improvement potential: {improvement:.1f}%\n")
            
            f.write("\n2. ROLL RANGE\n")
            best_range = df['roll_range'].idxmin()
            worst_range = df['roll_range'].idxmax()
            
            f.write(f"   Best containment: {best_range} ({df.loc[best_range, 'roll_range']:.4f}°)\n")
            f.write(f"   Worst containment: {worst_range} ({df.loc[worst_range, 'roll_range']:.4f}°)\n")
            
            f.write("\n3. ANGULAR VELOCITY CONTROL\n")
            best_omega = df['omega_max'].idxmin()
            worst_omega = df['omega_max'].idxmax()
            
            f.write(f"   Best control: {best_omega} (Max: {df.loc[best_omega, 'omega_max']:.4f}°/s)\n")
            f.write(f"   Worst control: {worst_omega} (Max: {df.loc[worst_omega, 'omega_max']:.4f}°/s)\n")
            
            f.write("\n4. FREQUENCY CHARACTERISTICS\n")
            avg_freq = df['dominant_freq'].mean()
            f.write(f"   Average dominant frequency: {avg_freq:.4f} Hz\n")
            
            low_freq_tests = df[df['dominant_freq'] < 1.0]
            if len(low_freq_tests) > 0:
                f.write(f"   Tests with low dominant frequency (<1 Hz): {len(low_freq_tests)}\n")
                for test in low_freq_tests.index:
                    f.write(f"     - {test}: {df.loc[test, 'dominant_freq']:.4f} Hz\n")
        
        print(f"✓ Comparison report saved: {report_file}")
    
    def run_comparison(self):
        """Run comparison analysis"""
        print("\n" + "=" * 100)
        print("STABILITY COMPARISON ANALYSIS")
        print("=" * 100)
        
        print("\n[1/3] Loading analysis results...")
        data = self.load_results()
        if data is None:
            return
        
        print(f"Loaded {len(data)} test results")
        
        print("\n[2/3] Comparing metrics...")
        comparison_data = self.extract_metrics(data)
        df = self.plot_comparison(comparison_data)
        
        print("\n[3/3] Generating comparison report...")
        self.generate_comparison_report(comparison_data)
        
        print("\n" + "=" * 100)
        print("✓ Comparison analysis complete!")
        print(f"✓ Results saved to: {self.output_dir}/")
        print("=" * 100)


if __name__ == "__main__":
    comparator = StabilityComparator()
    comparator.run_comparison()
