#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
System Stability Analysis - Motor Control for Roll Angle Reduction
Focus on concrete physical metrics: Roll angle, Angular acceleration, Frequency characteristics
"""

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['font.family'] = 'DejaVu Sans'
matplotlib.rcParams['axes.unicode_minus'] = False
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats, signal
from scipy.fft import fft, fftfreq
import os
import glob
from datetime import datetime
from pathlib import Path
import json
import warnings
warnings.filterwarnings('ignore')

class SystemStabilityAnalyzer:
    """System Stability Analyzer - Focus on Physical Metrics"""
    
    def __init__(self, data_dir=".", output_dir="analysis_results"):
        self.data_dir = data_dir
        self.output_dir = output_dir
        Path(self.output_dir).mkdir(exist_ok=True)
        Path(f"{self.output_dir}/figures").mkdir(exist_ok=True)
    
    def load_csv(self, filepath):
        """Load CSV data"""
        try:
            df = pd.read_csv(filepath)
            print(f"✓ Loaded: {Path(filepath).name} ({len(df)} rows)")
            return df
        except Exception as e:
            print(f"✗ Failed to load {filepath}: {e}")
            return None
    
    def load_all_data(self, pattern="feedforward_dual_imu_*.csv"):
        """Load all matching CSV files"""
        files = sorted(glob.glob(os.path.join(self.data_dir, pattern)))
        if not files:
            print(f"No files matching {pattern} found")
            return {}
        
        data = {}
        for filepath in files:
            filename = Path(filepath).stem
            df = self.load_csv(filepath)
            if df is not None:
                data[filename] = df
        
        return data
    
    def preprocess_data(self, df):
        """Preprocess data"""
        df = df.copy()
        df['t_sec'] = np.arange(len(df)) / 100  # Assume 100Hz sampling rate
        df['roll_angle'] = df['roll_deg'].fillna(0)
        
        # Calculate angular acceleration (first derivative of angular velocity)
        omega = df['omega_filtered'].fillna(0).values
        df['angular_accel'] = np.gradient(omega) * 100  # Convert to deg/s^2
        
        return df
    
    def analyze_roll_angle(self, df):
        """Analyze roll angle stability"""
        roll = df['roll_angle'].values
        
        metrics = {
            'mean_roll': float(np.mean(roll)),
            'std_roll': float(np.std(roll)),
            'min_roll': float(np.min(roll)),
            'max_roll': float(np.max(roll)),
            'roll_range': float(np.max(roll) - np.min(roll)),
            'abs_mean_roll': float(np.mean(np.abs(roll))),
            'rms_roll': float(np.sqrt(np.mean(roll**2))),
        }
        
        return metrics
    
    def analyze_angular_velocity(self, df):
        """Analyze angular velocity characteristics"""
        omega = df['omega_filtered'].fillna(0).values
        
        metrics = {
            'mean_omega': float(np.mean(omega)),
            'std_omega': float(np.std(omega)),
            'max_omega': float(np.max(np.abs(omega))),
            'rms_omega': float(np.sqrt(np.mean(omega**2))),
        }
        
        return metrics
    
    def analyze_angular_acceleration(self, df):
        """Analyze angular acceleration characteristics"""
        accel = df['angular_accel'].values
        
        metrics = {
            'mean_accel': float(np.mean(accel)),
            'std_accel': float(np.std(accel)),
            'max_accel': float(np.max(np.abs(accel))),
            'rms_accel': float(np.sqrt(np.mean(accel**2))),
        }
        
        return metrics
    
    def analyze_frequency_characteristics(self, df):
        """Analyze frequency domain characteristics"""
        duration = df['t_sec'].max() - df['t_sec'].min()
        fs = len(df) / duration if duration > 0 else 100
        
        roll = df['roll_angle'].values - df['roll_angle'].mean()
        
        freq_metrics = {}
        
        if len(roll) > 10:
            try:
                fft_vals = np.fft.fft(roll)
                freqs = np.fft.fftfreq(len(fft_vals), 1/fs)
                magnitude = np.abs(fft_vals)
                
                # Get positive frequencies only
                positive_idx = freqs > 0
                pos_freqs = freqs[positive_idx]
                pos_mag = magnitude[positive_idx]
                
                if len(pos_mag) > 0:
                    # Find dominant frequency
                    dominant_idx = np.argmax(pos_mag)
                    freq_metrics['dominant_freq'] = float(pos_freqs[dominant_idx]) if pos_freqs[dominant_idx] < 10 else 0.0
                    
                    # Energy in frequency bands
                    energy = pos_mag**2
                    total_energy = np.sum(energy)
                    
                    if total_energy > 1e-10:
                        low_energy = np.sum(energy[(pos_freqs > 0) & (pos_freqs < 0.5)])
                        mid_energy = np.sum(energy[(pos_freqs >= 0.5) & (pos_freqs < 2)])
                        high_energy = np.sum(energy[(pos_freqs >= 2)])
                        
                        freq_metrics['low_freq_ratio'] = float(low_energy / total_energy)
                        freq_metrics['mid_freq_ratio'] = float(mid_energy / total_energy)
                        freq_metrics['high_freq_ratio'] = float(high_energy / total_energy)
            except:
                pass
        
        # Default values if FFT failed
        freq_metrics.setdefault('dominant_freq', 0.0)
        freq_metrics.setdefault('low_freq_ratio', 0.33)
        freq_metrics.setdefault('mid_freq_ratio', 0.33)
        freq_metrics.setdefault('high_freq_ratio', 0.34)
        
        return freq_metrics
    
    def analyze_pwm_output(self, df):
        """Analyze PWM output characteristics"""
        pwm_left = df['pwm_left'].values
        pwm_right = df['pwm_right'].values
        pwm_mid = 1500
        
        metrics = {
            'mean_pwm_left': float(np.mean(pwm_left)),
            'mean_pwm_right': float(np.mean(pwm_right)),
            'std_pwm_left': float(np.std(pwm_left)),
            'std_pwm_right': float(np.std(pwm_right)),
            'pwm_sync_error': float(np.mean(np.abs(pwm_left - pwm_right))),
        }
        
        return metrics
    
    def generate_detailed_report(self, data_dict):
        """Generate detailed analysis report"""
        all_results = {}
        
        for filename, df in data_dict.items():
            print(f"\nAnalyzing: {filename}")
            
            df = self.preprocess_data(df)
            
            roll_metrics = self.analyze_roll_angle(df)
            omega_metrics = self.analyze_angular_velocity(df)
            accel_metrics = self.analyze_angular_acceleration(df)
            freq_metrics = self.analyze_frequency_characteristics(df)
            pwm_metrics = self.analyze_pwm_output(df)
            
            all_results[filename] = {
                'roll': roll_metrics,
                'omega': omega_metrics,
                'accel': accel_metrics,
                'frequency': freq_metrics,
                'pwm': pwm_metrics,
                'dataframe': df
            }
            
            # Print summary
            print(f"  Roll Angle - Mean: {roll_metrics['mean_roll']:.3f}°, "
                  f"Std: {roll_metrics['std_roll']:.3f}°, "
                  f"Range: {roll_metrics['roll_range']:.3f}°")
            print(f"  Angular Velocity - Std: {omega_metrics['std_omega']:.3f}°/s, "
                  f"Max: {omega_metrics['max_omega']:.3f}°/s")
            print(f"  Angular Acceleration - Std: {accel_metrics['std_accel']:.3f}°/s²")
            print(f"  Dominant Frequency: {freq_metrics['dominant_freq']:.3f} Hz")
        
        return all_results
    
    def plot_analysis(self, filename, results):
        """Plot detailed analysis"""
        df = results['dataframe']
        roll_metrics = results['roll']
        freq_metrics = results['frequency']
        
        fig = plt.figure(figsize=(16, 12))
        
        # 1. Roll angle time series
        ax1 = plt.subplot(3, 3, 1)
        ax1.plot(df['t_sec'], df['roll_angle'], linewidth=1.5, color='#2E86AB')
        ax1.axhline(y=roll_metrics['mean_roll'], color='r', linestyle='--', 
                   label=f"Mean: {roll_metrics['mean_roll']:.2f}°")
        ax1.fill_between(df['t_sec'],
                        roll_metrics['mean_roll'] - roll_metrics['std_roll'],
                        roll_metrics['mean_roll'] + roll_metrics['std_roll'],
                        alpha=0.2, color='r')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Roll Angle (deg)')
        ax1.set_title('Roll Angle Time Series')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # 2. Angular velocity
        ax2 = plt.subplot(3, 3, 2)
        ax2.plot(df['t_sec'], df['omega_filtered'], linewidth=1.5, color='#A23B72')
        ax2.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Angular Velocity (deg/s)')
        ax2.set_title('Angular Velocity')
        ax2.grid(True, alpha=0.3)
        
        # 3. Angular acceleration
        ax3 = plt.subplot(3, 3, 3)
        ax3.plot(df['t_sec'], df['angular_accel'], linewidth=1.5, color='#F18F01')
        ax3.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Angular Acceleration (deg/s²)')
        ax3.set_title('Angular Acceleration')
        ax3.grid(True, alpha=0.3)
        
        # 4. Roll angle histogram
        ax4 = plt.subplot(3, 3, 4)
        ax4.hist(df['roll_angle'], bins=50, color='#2E86AB', edgecolor='black', alpha=0.7)
        ax4.axvline(x=roll_metrics['mean_roll'], color='r', linestyle='--', 
                   linewidth=2, label=f"Mean: {roll_metrics['mean_roll']:.2f}°")
        ax4.set_xlabel('Roll Angle (deg)')
        ax4.set_ylabel('Frequency')
        ax4.set_title('Roll Angle Distribution')
        ax4.legend()
        ax4.grid(True, alpha=0.3, axis='y')
        
        # 5. Angular velocity histogram
        ax5 = plt.subplot(3, 3, 5)
        ax5.hist(df['omega_filtered'], bins=50, color='#A23B72', edgecolor='black', alpha=0.7)
        ax5.axvline(x=0, color='k', linestyle='-', alpha=0.3)
        ax5.set_xlabel('Angular Velocity (deg/s)')
        ax5.set_ylabel('Frequency')
        ax5.set_title('Angular Velocity Distribution')
        ax5.grid(True, alpha=0.3, axis='y')
        
        # 6. Control signals (tau_ff and tau_pid)
        ax6 = plt.subplot(3, 3, 6)
        ax6.plot(df['t_sec'], df['tau_ff'], label='Feedforward', linewidth=1.5, alpha=0.7)
        ax6.plot(df['t_sec'], df['tau_pid'], label='Feedback', linewidth=1.5, alpha=0.7)
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Torque (N.m)')
        ax6.set_title('Control Torque')
        ax6.legend()
        ax6.grid(True, alpha=0.3)
        
        # 7. Frequency spectrum
        ax7 = plt.subplot(3, 3, 7)
        duration = df['t_sec'].max() - df['t_sec'].min()
        fs = len(df) / duration
        roll = df['roll_angle'].values - df['roll_angle'].mean()
        fft_vals = np.fft.fft(roll)
        freqs = np.fft.fftfreq(len(fft_vals), 1/fs)
        magnitude = np.abs(fft_vals)
        
        positive_idx = freqs > 0
        ax7.semilogy(freqs[positive_idx], magnitude[positive_idx], color='#2E86AB', linewidth=1.5)
        ax7.set_xlabel('Frequency (Hz)')
        ax7.set_ylabel('Magnitude')
        ax7.set_title('Frequency Spectrum (Roll Angle)')
        ax7.set_xlim([0, 5])
        ax7.grid(True, alpha=0.3)
        
        # 8. Key metrics table
        ax8 = plt.subplot(3, 3, 8)
        ax8.axis('off')
        
        metrics_text = f"""
KEY METRICS SUMMARY
═════════════════════════════════════
Roll Angle:
  Mean: {roll_metrics['mean_roll']:>8.3f}°
  Std Dev: {roll_metrics['std_roll']:>8.3f}°
  Range: {roll_metrics['roll_range']:>8.3f}°
  RMS: {roll_metrics['rms_roll']:>8.3f}°

Angular Velocity:
  Max: {results['omega']['max_omega']:>8.3f}°/s
  Std Dev: {results['omega']['std_omega']:>8.3f}°/s

Angular Acceleration:
  Max: {results['accel']['max_accel']:>8.3f}°/s²
  Std Dev: {results['accel']['std_accel']:>8.3f}°/s²

Frequency Domain:
  Dominant: {freq_metrics['dominant_freq']:>8.3f} Hz
  Low Freq Energy: {freq_metrics['low_freq_ratio']*100:>6.1f}%
"""
        
        ax8.text(0.05, 0.95, metrics_text, fontsize=10, family='monospace',
                verticalalignment='top', bbox=dict(boxstyle='round', 
                facecolor='wheat', alpha=0.5))
        
        # 9. PWM sync analysis
        ax9 = plt.subplot(3, 3, 9)
        pwm_error = np.abs(df['pwm_left'] - df['pwm_right'])
        ax9.plot(df['t_sec'], pwm_error, linewidth=1.5, color='#C73E1D')
        ax9.axhline(y=results['pwm']['pwm_sync_error'], color='r', linestyle='--',
                   label=f"Mean Error: {results['pwm']['pwm_sync_error']:.1f} us")
        ax9.set_xlabel('Time (s)')
        ax9.set_ylabel('PWM Difference (us)')
        ax9.set_title('PWM Synchronization Error')
        ax9.legend()
        ax9.grid(True, alpha=0.3)
        
        plt.tight_layout()
        output_file = os.path.join(self.output_dir, 'figures', f'{filename}_analysis.png')
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"  ✓ Chart saved: {output_file}")
        plt.close()
    
    def export_detailed_report(self, all_results):
        """Export detailed text report"""
        report_file = os.path.join(self.output_dir, 'stability_report.txt')
        
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write("=" * 100 + "\n")
            f.write("SYSTEM STABILITY ANALYSIS REPORT\n")
            f.write("Motor Control System for Roll Angle Reduction\n")
            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("=" * 100 + "\n\n")
            
            # Summary table
            f.write("SUMMARY TABLE\n")
            f.write("-" * 100 + "\n")
            f.write(f"{'Test File':<40} {'Roll Mean':<12} {'Roll Std':<12} {'Roll Range':<12} {'Omega Max':<12}\n")
            f.write("-" * 100 + "\n")
            
            for filename in sorted(all_results.keys()):
                result = all_results[filename]
                f.write(
                    f"{filename:<40} "
                    f"{result['roll']['mean_roll']:<12.3f} "
                    f"{result['roll']['std_roll']:<12.3f} "
                    f"{result['roll']['roll_range']:<12.3f} "
                    f"{result['omega']['max_omega']:<12.3f}\n"
                )
            
            f.write("\n" + "=" * 100 + "\n")
            f.write("DETAILED ANALYSIS\n")
            f.write("=" * 100 + "\n\n")
            
            for filename in sorted(all_results.keys()):
                result = all_results[filename]
                f.write(f"\n{filename}\n")
                f.write("-" * 100 + "\n")
                
                f.write("\n[ROLL ANGLE METRICS]\n")
                for key, val in result['roll'].items():
                    f.write(f"  {key:<30}: {val:>12.4f}°\n")
                
                f.write("\n[ANGULAR VELOCITY METRICS]\n")
                for key, val in result['omega'].items():
                    f.write(f"  {key:<30}: {val:>12.4f}°/s\n")
                
                f.write("\n[ANGULAR ACCELERATION METRICS]\n")
                for key, val in result['accel'].items():
                    f.write(f"  {key:<30}: {val:>12.4f}°/s²\n")
                
                f.write("\n[FREQUENCY DOMAIN ANALYSIS]\n")
                for key, val in result['frequency'].items():
                    if isinstance(val, float):
                        if 'ratio' in key or 'energy' in key:
                            f.write(f"  {key:<30}: {val:>12.2%}\n")
                        else:
                            f.write(f"  {key:<30}: {val:>12.4f} Hz\n")
                
                f.write("\n[PWM OUTPUT ANALYSIS]\n")
                for key, val in result['pwm'].items():
                    f.write(f"  {key:<30}: {val:>12.4f} us\n")
        
        print(f"\n✓ Report saved: {report_file}")
    
    def export_json(self, all_results):
        """Export results as JSON"""
        json_data = {}
        
        for filename, result in all_results.items():
            json_data[filename] = {
                'roll': result['roll'],
                'omega': result['omega'],
                'accel': result['accel'],
                'frequency': result['frequency'],
                'pwm': result['pwm'],
            }
        
        json_file = os.path.join(self.output_dir, 'analysis_data.json')
        with open(json_file, 'w', encoding='utf-8') as f:
            json.dump(json_data, f, indent=2)
        
        print(f"✓ JSON data saved: {json_file}")
    
    def run_analysis(self, pattern="feedforward_dual_imu_*.csv"):
        """Run complete analysis"""
        print("=" * 100)
        print("MOTOR CONTROL STABILITY ANALYSIS")
        print("=" * 100)
        
        # Load data
        print("\n[1/4] Loading data...")
        data_dict = self.load_all_data(pattern)
        
        if not data_dict:
            print("No data files found, exiting")
            return
        
        # Analyze
        print("\n[2/4] Analyzing system stability...")
        all_results = self.generate_detailed_report(data_dict)
        
        # Visualize
        print("\n[3/4] Generating visualizations...")
        for filename, result in all_results.items():
            self.plot_analysis(filename, result)
        
        # Export
        print("\n[4/4] Exporting reports...")
        self.export_detailed_report(all_results)
        self.export_json(all_results)
        
        print("\n" + "=" * 100)
        print("✓ Analysis complete!")
        print(f"✓ Results saved to: {self.output_dir}/")
        print("=" * 100)


if __name__ == "__main__":
    analyzer = SystemStabilityAnalyzer(
        data_dir="/home/pi/Desktop",
        output_dir="/home/pi/Desktop/analysis_results"
    )
    analyzer.run_analysis()
