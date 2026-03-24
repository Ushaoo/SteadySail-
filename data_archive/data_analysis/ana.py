#!/usr/bin/env python3
"""
IMU Data Batch Analyzer with Advanced Stability Assessment
Focus on X-axis angle and angular velocity analysis with multiple stability metrics
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import glob
from scipy import stats
from scipy.signal import butter, filtfilt
import warnings
warnings.filterwarnings('ignore')

# Set plot style
plt.style.use('default')
plt.rcParams['figure.figsize'] = (16, 10)

class IMUDataAnalyzer:
    def __init__(self, data_folder, output_folder="analysis_results"):
        """
        Initialize analyzer
        
        Parameters:
            data_folder: Folder containing CSV files
            output_folder: Output folder for results
        """
        self.data_folder = data_folder
        self.output_folder = output_folder
        self.all_data = []
        self.stability_scores = []  # Store stability scores for all files
        
        # Create output folders
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)
        
        self.individual_plots_folder = os.path.join(output_folder, "individual_plots")
        if not os.path.exists(self.individual_plots_folder):
            os.makedirs(self.individual_plots_folder)
    
    def load_and_preprocess_data(self, file_path):
        """
        Load and preprocess single CSV file
        
        Parameters:
            file_path: CSV file path
            
        Returns:
            Preprocessed DataFrame
        """
        try:
            # Read CSV file
            df = pd.read_csv(file_path)
            print(f"Loaded: {os.path.basename(file_path)}")
            
            # Process timestamp
            if 'timestamp' in df.columns:
                df['datetime'] = pd.to_datetime(df['timestamp'])
                start_time = df['datetime'].min()
                df['time_seconds'] = (df['datetime'] - start_time).dt.total_seconds()
            else:
                # Create time based on sampling rate
                df['time_seconds'] = df.index / 100  # Assume 100Hz sampling
            
            return df
            
        except Exception as e:
            print(f"Failed to load {file_path}: {e}")
            return None
    
    def calculate_x_angle(self, df):
        """
        Calculate X-axis angle from accelerometer data
        
        Parameters:
            df: DataFrame with accelerometer data
            
        Returns:
            DataFrame with X-angle calculated
        """
        # Calculate roll angle (rotation around X-axis)
        # Using Y and Z accelerometer data
        df['x_angle'] = np.arctan2(df['acc_y'], df['acc_z']) * 180 / np.pi
        
        return df
    
    def calculate_x_angular_velocity(self, df):
        """
        Calculate X-axis angular velocity
        
        Parameters:
            df: DataFrame with gyroscope data
            
        Returns:
            DataFrame with X-angular velocity
        """
        # Use gyro_x directly as X-axis angular velocity
        df['x_angular_velocity'] = df['gyro_x']
        
        # Apply light smoothing
        window_size = min(5, len(df) // 10)  # Small window for light smoothing
        if window_size > 0:
            df['x_angular_velocity_smooth'] = df['x_angular_velocity'].rolling(
                window=window_size, center=True, min_periods=1
            ).mean()
        else:
            df['x_angular_velocity_smooth'] = df['x_angular_velocity']
        
        return df
    
    def remove_extreme_outliers(self, df, column, threshold=50):
        """
        Remove only extreme outliers with very lenient threshold
        
        Parameters:
            df: Input DataFrame
            column: Column to check
            threshold: Absolute value threshold
            
        Returns:
            Filtered DataFrame
        """
        # Very lenient filtering - only remove extreme values
        df_clean = df[np.abs(df[column]) < threshold].copy()
        
        removed_count = len(df) - len(df_clean)
        if removed_count > 0:
            print(f"  Removed {removed_count} extreme outliers from {column}")
        
        return df_clean
    
    def standardize_duration(self, df, trim_duration=5, target_duration=30):
        """
        Standardize data duration by trimming ends and taking fixed duration
        
        Parameters:
            df: Input DataFrame
            trim_duration: Duration to trim from both ends (seconds)
            target_duration: Target recording duration (seconds)
            
        Returns:
            Standardized DataFrame
        """
        total_duration = df['time_seconds'].max() - df['time_seconds'].min()
        
        # Check if we have enough data
        required_duration = 2 * trim_duration + target_duration
        
        if total_duration < required_duration:
            # If not enough data, use what we have but warn
            print(f"  Warning: Data duration {total_duration:.2f}s < required {required_duration}s")
            # Use middle section as much as possible
            middle_start = (total_duration - target_duration) / 2
            if middle_start < 0:
                middle_start = 0
                target_duration = total_duration
            
            start_time = df['time_seconds'].min() + middle_start
            end_time = start_time + target_duration
        else:
            # Trim from both ends and take fixed duration
            start_time = df['time_seconds'].min() + trim_duration
            end_time = start_time + target_duration
        
        df_standardized = df[
            (df['time_seconds'] >= start_time) & 
            (df['time_seconds'] <= end_time)
        ].copy()
        
        # Reset time to start from 0
        df_standardized['time_seconds'] = df_standardized['time_seconds'] - df_standardized['time_seconds'].min()
        
        print(f"  Standardized duration: {total_duration:.2f}s -> {df_standardized['time_seconds'].max():.2f}s")
        print(f"  (Trimmed {trim_duration}s from both ends, kept {target_duration}s recording)")
        
        return df_standardized
    
    def preprocess_data(self, df):
        """
        Complete data preprocessing pipeline
        
        Parameters:
            df: Raw data
            
        Returns:
            Preprocessed data
        """
        if df is None:
            return None
        
        print(f"  Original data points: {len(df)}")
        
        # Calculate X-axis angle and angular velocity
        df = self.calculate_x_angle(df)
        df = self.calculate_x_angular_velocity(df)
        
        # Remove only extreme outliers with very lenient thresholds
        df_clean = self.remove_extreme_outliers(df, 'x_angle', threshold=90)  # Very lenient
        df_clean = self.remove_extreme_outliers(df_clean, 'x_angular_velocity', threshold=100)  # Very lenient
        
        print(f"  After outlier removal: {len(df_clean)} points")
        
        # Standardize duration - trim 5s from both ends, keep 30s recording
        df_standardized = self.standardize_duration(df_clean, trim_duration=5, target_duration=30)
        
        print(f"  Final data points: {len(df_standardized)}")
        
        return df_standardized
    
    # ==================== STABILITY ASSESSMENT METHODS ====================
    
    def calculate_stability_metrics(self, df):
        """
        Method 1: Statistical Feature Comparison
        Calculate comprehensive stability metrics
        
        Parameters:
            df: Preprocessed DataFrame
            
        Returns:
            Dictionary of stability metrics
        """
        metrics = {}
        
        # Angle stability metrics
        metrics['angle_std'] = df['x_angle'].std()  # Standard deviation
        metrics['angle_range'] = df['x_angle'].max() - df['x_angle'].min()  # Range
        # Replace mad() with manual calculation
        metrics['angle_mad'] = np.mean(np.abs(df['x_angle'] - df['x_angle'].mean()))  # Mean absolute deviation
        metrics['angle_rms'] = np.sqrt(np.mean(df['x_angle']**2))  # RMS value
        
        # Angular velocity stability metrics  
        metrics['angular_velocity_std'] = df['x_angular_velocity'].std()
        metrics['angular_velocity_range'] = df['x_angular_velocity'].max() - df['x_angular_velocity'].min()
        # Replace mad() with manual calculation
        metrics['angular_velocity_mad'] = np.mean(np.abs(df['x_angular_velocity'] - df['x_angular_velocity'].mean()))
        metrics['angular_velocity_rms'] = np.sqrt(np.mean(df['x_angular_velocity']**2))
        
        # Rate of change metrics
        angle_diff = np.diff(df['x_angle'])
        angular_velocity_diff = np.diff(df['x_angular_velocity'])
        
        metrics['angle_change_rate_std'] = np.std(angle_diff)
        metrics['angular_velocity_change_rate_std'] = np.std(angular_velocity_diff)
        metrics['max_angle_change'] = np.max(np.abs(angle_diff))
        metrics['max_angular_velocity_change'] = np.max(np.abs(angular_velocity_diff))
        
        return metrics
    
    def frequency_analysis(self, df):
        """
        Method 2: Frequency Domain Analysis
        Analyze stability in frequency domain
        
        Parameters:
            df: Preprocessed DataFrame
            
        Returns:
            Dictionary of frequency domain metrics
        """
        # Estimate sampling frequency
        duration = df['time_seconds'].max() - df['time_seconds'].min()
        fs = len(df) / duration if duration > 0 else 100  # Fallback to 100Hz
        
        freq_metrics = {}
        
        # Angle frequency analysis
        angle_fft = np.fft.fft(df['x_angle'] - df['x_angle'].mean())
        freqs = np.fft.fftfreq(len(angle_fft), 1/fs)
        
        # Calculate magnitude spectrum
        magnitude = np.abs(angle_fft)
        
        # Frequency bands
        low_freq_mask = (np.abs(freqs) > 0.1) & (np.abs(freqs) < 1)  # 0.1-1Hz
        mid_freq_mask = (np.abs(freqs) >= 1) & (np.abs(freqs) < 5)   # 1-5Hz
        high_freq_mask = np.abs(freqs) >= 5                          # 5Hz+
        
        # Energy ratios
        total_energy = np.sum(magnitude**2)
        if total_energy > 0:
            freq_metrics['low_freq_energy_ratio'] = np.sum(magnitude[low_freq_mask]**2) / total_energy
            freq_metrics['mid_freq_energy_ratio'] = np.sum(magnitude[mid_freq_mask]**2) / total_energy
            freq_metrics['high_freq_energy_ratio'] = np.sum(magnitude[high_freq_mask]**2) / total_energy
        else:
            freq_metrics['low_freq_energy_ratio'] = 0
            freq_metrics['mid_freq_energy_ratio'] = 0
            freq_metrics['high_freq_energy_ratio'] = 0
        
        # Dominant frequency
        if len(magnitude) > 0:
            dominant_freq_idx = np.argmax(magnitude[1:]) + 1  # Skip DC component
            freq_metrics['dominant_frequency'] = np.abs(freqs[dominant_freq_idx])
        else:
            freq_metrics['dominant_frequency'] = 0
        
        return freq_metrics
    
    def sliding_window_stability(self, df, window_size=3):
        """
        Method 3: Sliding Window Stability Assessment
        Analyze local stability using sliding windows
        
        Parameters:
            df: Preprocessed DataFrame
            window_size: Window size in seconds
            
        Returns:
            Dictionary of window-based stability metrics
        """
        # Convert window size from seconds to data points
        duration = df['time_seconds'].max() - df['time_seconds'].min()
        window_points = int(window_size * len(df) / duration) if duration > 0 else 30
        
        if window_points < 5:  # Minimum window size
            window_points = min(5, len(df))
        
        window_metrics = {}
        angle_stability_scores = []
        angular_velocity_stability_scores = []
        
        for i in range(0, len(df) - window_points, window_points // 2):  # 50% overlap
            window_data = df.iloc[i:i+window_points]
            
            if len(window_data) < 3:  # Need at least 3 points for meaningful stats
                continue
            
            # Calculate window stability scores (inverse of variance)
            angle_std = window_data['x_angle'].std()
            angular_std = window_data['x_angular_velocity'].std()
            
            # Avoid division by zero
            angle_stability = 1 / (1 + angle_std) if angle_std > 0 else 1
            angular_stability = 1 / (1 + angular_std) if angular_std > 0 else 1
            
            angle_stability_scores.append(angle_stability)
            angular_velocity_stability_scores.append(angular_stability)
        
        if angle_stability_scores:
            window_metrics['mean_angle_stability'] = np.mean(angle_stability_scores)
            window_metrics['std_angle_stability'] = np.std(angle_stability_scores)
            window_metrics['min_angle_stability'] = np.min(angle_stability_scores)
            window_metrics['max_angle_stability'] = np.max(angle_stability_scores)
        else:
            window_metrics['mean_angle_stability'] = 0
            window_metrics['std_angle_stability'] = 0
            window_metrics['min_angle_stability'] = 0
            window_metrics['max_angle_stability'] = 0
            
        if angular_velocity_stability_scores:
            window_metrics['mean_angular_stability'] = np.mean(angular_velocity_stability_scores)
            window_metrics['std_angular_stability'] = np.std(angular_velocity_stability_scores)
            window_metrics['min_angular_stability'] = np.min(angular_velocity_stability_scores)
            window_metrics['max_angular_stability'] = np.max(angular_velocity_stability_scores)
        else:
            window_metrics['mean_angular_stability'] = 0
            window_metrics['std_angular_stability'] = 0
            window_metrics['min_angular_stability'] = 0
            window_metrics['max_angular_stability'] = 0
        
        return window_metrics
    
    def comprehensive_stability_score(self, df):
        """
        Method 4: Comprehensive Stability Scoring System
        Combine all metrics into a unified stability score
        
        Parameters:
            df: Preprocessed DataFrame
            
        Returns:
            Total stability score and individual component scores
        """
        # Calculate all metrics
        stats_metrics = self.calculate_stability_metrics(df)
        freq_metrics = self.frequency_analysis(df)
        window_metrics = self.sliding_window_stability(df)
        
        # Normalize and weight individual metrics
        scores = {
            # Statistical metrics (40% weight)
            'angle_steadiness': 1 / (1 + stats_metrics['angle_std']),  # 10%
            'angular_velocity_steadiness': 1 / (1 + stats_metrics['angular_velocity_std']),  # 10%
            'consistency': 1 / (1 + stats_metrics['angle_change_rate_std']),  # 10%
            'smoothness': 1 / (1 + stats_metrics['max_angle_change']),  # 10%
            
            # Frequency metrics (30% weight)
            'low_freq_dominance': freq_metrics['low_freq_energy_ratio'],  # 15%
            'high_freq_suppression': 1 - freq_metrics['high_freq_energy_ratio'],  # 15%
            
            # Window-based metrics (30% weight)
            'local_stability': window_metrics['mean_angle_stability'],  # 15%
            'stability_consistency': 1 / (1 + window_metrics['std_angle_stability']),  # 15%
        }
        
        # Weights for each component
        weights = {
            'angle_steadiness': 0.10,
            'angular_velocity_steadiness': 0.10,
            'consistency': 0.10,
            'smoothness': 0.10,
            'low_freq_dominance': 0.15,
            'high_freq_suppression': 0.15,
            'local_stability': 0.15,
            'stability_consistency': 0.15
        }
        
        # Calculate weighted total score
        total_score = sum(scores[metric] * weights[metric] for metric in scores)
        
        return total_score, scores, stats_metrics, freq_metrics, window_metrics
    
    def create_stability_radar_chart(self):
        """
        Create radar chart comparing stability metrics across all files
        """
        if len(self.stability_scores) == 0:
            print("No stability scores available for radar chart")
            return
        
        # Prepare data for radar chart
        metrics = list(self.stability_scores[0]['scores'].keys())
        
        fig, ax = plt.subplots(figsize=(12, 10), subplot_kw=dict(projection='polar'))
        
        # Use color map
        colors = plt.cm.tab10(np.linspace(0, 1, len(self.stability_scores)))
        
        for i, score_data in enumerate(self.stability_scores):
            values = [score_data['scores'][metric] for metric in metrics]
            # Close the radar plot
            values += values[:1]
            
            angles = np.linspace(0, 2*np.pi, len(metrics), endpoint=False).tolist()
            angles += angles[:1]
            
            label = f"{score_data['filename']} (总分: {score_data['total_score']:.3f})"
            ax.plot(angles, values, 'o-', linewidth=2, label=label, color=colors[i])
            ax.fill(angles, values, alpha=0.1, color=colors[i])
        
        # Set category labels
        ax.set_xticks(angles[:-1])
        ax.set_xticklabels(metrics, fontsize=10)
        ax.set_ylim(0, 1)
        ax.set_title('IMU数据稳定性多维度对比雷达图', size=16, fontweight='bold', pad=20)
        ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.0), fontsize=9)
        
        # Save radar chart
        radar_path = os.path.join(self.output_folder, "stability_radar_chart.png")
        plt.savefig(radar_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"Stability radar chart saved: {radar_path}")
    
    def create_stability_ranking_table(self):
        """
        Create stability ranking table
        """
        if len(self.stability_scores) == 0:
            print("No stability scores available for ranking table")
            return
        
        # Prepare ranking data
        ranking_data = []
        for score_data in self.stability_scores:
            row_data = {
                '文件': score_data['filename'],
                '综合稳定性得分': score_data['total_score'],
                **score_data['scores']
            }
            ranking_data.append(row_data)
        
        df_ranking = pd.DataFrame(ranking_data)
        df_ranking = df_ranking.sort_values('综合稳定性得分', ascending=False)
        
        # Save ranking table
        ranking_path = os.path.join(self.output_folder, "stability_ranking.csv")
        df_ranking.to_csv(ranking_path, index=False, encoding='utf-8-sig')
        
        # Create styled HTML table
        try:
            styled_df = df_ranking.style.background_gradient(
                subset=['综合稳定性得分'], cmap='RdYlGn_r'
            ).format({
                '综合稳定性得分': '{:.3f}',
                **{metric: '{:.3f}' for metric in list(self.stability_scores[0]['scores'].keys())}
            })
            
            # Save styled table as HTML
            html_path = os.path.join(self.output_folder, "stability_ranking.html")
            with open(html_path, 'w', encoding='utf-8') as f:
                f.write(styled_df.to_html())
            
            print(f"Styled ranking table saved: {html_path}")
        except Exception as e:
            print(f"Could not create styled table: {e}")
        
        print(f"Stability ranking table saved: {ranking_path}")
        return df_ranking
    
    def create_detailed_stability_report(self):
        """
        Create detailed stability analysis report
        """
        if len(self.stability_scores) == 0:
            return
        
        report_path = os.path.join(self.output_folder, "detailed_stability_analysis.txt")
        
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write("IMU数据详细稳定性分析报告\n")
            f.write("=" * 60 + "\n")
            f.write(f"分析时间: {pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"数据文件夹: {self.data_folder}\n")
            f.write(f"分析文件数: {len(self.stability_scores)}\n\n")
            
            # Overall ranking
            f.write("综合稳定性排名:\n")
            f.write("-" * 40 + "\n")
            sorted_scores = sorted(self.stability_scores, key=lambda x: x['total_score'], reverse=True)
            for i, score_data in enumerate(sorted_scores, 1):
                f.write(f"{i}. {score_data['filename']}: {score_data['total_score']:.3f}\n")
            
            f.write("\n")
            
            # Detailed analysis for each file
            for score_data in sorted_scores:
                f.write(f"\n文件: {score_data['filename']}\n")
                f.write(f"综合稳定性得分: {score_data['total_score']:.3f}\n")
                
                f.write("\n统计特征指标:\n")
                stats = score_data['stats_metrics']
                f.write(f"  角度标准差: {stats['angle_std']:.3f}°\n")
                f.write(f"  角度范围: {stats['angle_range']:.3f}°\n")
                f.write(f"  角速度标准差: {stats['angular_velocity_std']:.3f}°/s\n")
                f.write(f"  最大角度变化率: {stats['max_angle_change']:.3f}°/s\n")
                
                f.write("\n频域分析指标:\n")
                freq = score_data['freq_metrics']
                f.write(f"  低频能量占比: {freq['low_freq_energy_ratio']:.3f}\n")
                f.write(f"  高频能量占比: {freq['high_freq_energy_ratio']:.3f}\n")
                f.write(f"  主导频率: {freq['dominant_frequency']:.3f}Hz\n")
                
                f.write("\n滑动窗口指标:\n")
                window = score_data['window_metrics']
                f.write(f"  平均角度稳定性: {window['mean_angle_stability']:.3f}\n")
                f.write(f"  稳定性一致性: {window['std_angle_stability']:.3f}\n")
                
                f.write("-" * 40 + "\n")
        
        print(f"Detailed stability report saved: {report_path}")
    
    def create_individual_plot(self, df, filename):
        """
        Create individual plot for single file with stability metrics
        
        Parameters:
            df: Preprocessed data
            filename: Filename for title
        """
        if df is None or len(df) == 0:
            print(f"  Cannot create plot for {filename}: No valid data")
            return
        
        # Calculate stability scores for this file
        total_score, scores, stats_metrics, freq_metrics, window_metrics = self.comprehensive_stability_score(df)
        
        # Create figure with multiple subplots
        fig = plt.figure(figsize=(18, 14))
        
        # Set main title with stability score
        file_title = os.path.splitext(filename)[0]
        fig.suptitle(f'IMU数据稳定性分析 - {file_title} (稳定性得分: {total_score:.3f})', 
                    fontsize=16, fontweight='bold')
        
        # Create subplots: 3 rows, 2 columns
        gs = plt.GridSpec(3, 2, figure=fig)
        
        # Plot 1: X-axis angle
        ax1 = fig.add_subplot(gs[0, 0])
        ax1.plot(df['time_seconds'], df['x_angle'], 'blue', alpha=0.8, linewidth=2, label='X-角度')
        ax1.axhline(y=0, color='red', linestyle='--', alpha=0.5, label='零参考线')
        ax1.set_title('X轴角度趋势 (30秒记录)', fontsize=12, fontweight='bold')
        ax1.set_ylabel('角度 (°)', fontsize=10)
        ax1.legend(fontsize=9)
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: X-axis angular velocity
        ax2 = fig.add_subplot(gs[0, 1])
        ax2.plot(df['time_seconds'], df['x_angular_velocity'], 'green', alpha=0.7, linewidth=1.5, label='原始X轴角速度')
        ax2.plot(df['time_seconds'], df['x_angular_velocity_smooth'], 'purple', alpha=0.9, linewidth=2, label='平滑后X轴角速度')
        ax2.axhline(y=0, color='red', linestyle='--', alpha=0.5, label='零参考线')
        ax2.set_title('X轴角速度趋势 (30秒记录)', fontsize=12, fontweight='bold')
        ax2.set_ylabel('角速度 (°/s)', fontsize=10)
        ax2.legend(fontsize=9)
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Stability scores radar (miniature)
        ax3 = fig.add_subplot(gs[1, :], projection='polar')
        metrics = list(scores.keys())
        values = [scores[metric] for metric in metrics]
        values += values[:1]
        
        angles = np.linspace(0, 2*np.pi, len(metrics), endpoint=False).tolist()
        angles += angles[:1]
        
        ax3.plot(angles, values, 'o-', linewidth=2, color='red')
        ax3.fill(angles, values, alpha=0.3, color='red')
        ax3.set_xticks(angles[:-1])
        ax3.set_xticklabels(metrics, fontsize=8)
        ax3.set_ylim(0, 1)
        ax3.set_title('稳定性指标雷达图', fontsize=12, fontweight='bold', pad=20)
        
        # Plot 4: Statistical metrics bar chart
        ax4 = fig.add_subplot(gs[2, 0])
        stat_keys = ['angle_std', 'angle_range', 'angular_velocity_std', 'angle_change_rate_std']
        stat_values = [stats_metrics[key] for key in stat_keys]
        stat_labels = ['角度标准差', '角度范围', '角速度标准差', '角度变化率标准差']
        
        bars = ax4.bar(stat_labels, stat_values, color=['skyblue', 'lightcoral', 'lightgreen', 'gold'])
        ax4.set_title('关键统计指标', fontsize=12, fontweight='bold')
        ax4.set_ylabel('数值', fontsize=10)
        ax4.tick_params(axis='x', rotation=45)
        
        # Add value labels on bars
        for bar, value in zip(bars, stat_values):
            ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01 * max(stat_values),
                    f'{value:.3f}', ha='center', va='bottom', fontsize=8)
        
        # Plot 5: Frequency analysis
        ax5 = fig.add_subplot(gs[2, 1])
        freq_labels = ['低频(0.1-1Hz)', '中频(1-5Hz)', '高频(5Hz+)']
        freq_values = [freq_metrics['low_freq_energy_ratio'], 
                      freq_metrics['mid_freq_energy_ratio'], 
                      freq_metrics['high_freq_energy_ratio']]
        
        bars = ax5.bar(freq_labels, freq_values, color=['lightblue', 'lightgreen', 'lightcoral'])
        ax5.set_title('频域能量分布', fontsize=12, fontweight='bold')
        ax5.set_ylabel('能量占比', fontsize=10)
        
        # Add value labels on bars
        for bar, value in zip(bars, freq_values):
            ax5.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                    f'{value:.3f}', ha='center', va='bottom', fontsize=8)
        
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        
        # Save plot
        output_path = os.path.join(self.individual_plots_folder, f"{file_title}_stability_analysis.png")
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"  Individual stability plot saved: {output_path}")
        
        # Store stability scores for later comparison
        self.stability_scores.append({
            'filename': file_title,
            'total_score': total_score,
            'scores': scores,
            'stats_metrics': stats_metrics,
            'freq_metrics': freq_metrics,
            'window_metrics': window_metrics
        })
    
    def analyze_all_files(self):
        """
        Analyze all CSV files in folder with comprehensive stability assessment
        """
        # Find all CSV files
        csv_pattern = os.path.join(self.data_folder, "*.csv")
        csv_files = glob.glob(csv_pattern)
        
        if not csv_files:
            print(f"No CSV files found in folder: {self.data_folder}")
            return
        
        print(f"Found {len(csv_files)} CSV files")
        print("=" * 60)
        
        # Process each file
        for i, csv_file in enumerate(csv_files, 1):
            filename = os.path.basename(csv_file)
            print(f"Processing file {i}/{len(csv_files)}: {filename}")
            
            # Load and preprocess data
            df_raw = self.load_and_preprocess_data(csv_file)
            df_processed = self.preprocess_data(df_raw)
            
            if df_processed is not None and len(df_processed) > 0:
                # Create individual plot with stability analysis
                self.create_individual_plot(df_processed, filename)
            
            print("-" * 40)
        
        # Create comprehensive stability reports and visualizations
        if len(self.stability_scores) > 0:
            print("Creating comprehensive stability analysis...")
            self.create_stability_radar_chart()
            ranking_table = self.create_stability_ranking_table()
            self.create_detailed_stability_report()
            
            # Print summary to console
            print("\n稳定性排名总结:")
            print("-" * 50)
            sorted_scores = sorted(self.stability_scores, key=lambda x: x['total_score'], reverse=True)
            for i, score_data in enumerate(sorted_scores, 1):
                print(f"{i:2d}. {score_data['filename']:30} 稳定性得分: {score_data['total_score']:.3f}")
        
        # Generate analysis report
        self.generate_analysis_report()
        
        print("Analysis completed!")
    
    def generate_analysis_report(self):
        """
        Generate analysis report
        """
        if len(self.all_data) == 0:
            return
        
        report_path = os.path.join(self.output_folder, "analysis_report.txt")
        
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write("IMU Data Analysis Report\n")
            f.write("=" * 50 + "\n")
            f.write(f"Analysis Time: {pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Data Folder: {self.data_folder}\n")
            f.write(f"Files Analyzed: {len(self.all_data)}\n")
            f.write(f"Recording Duration: 30 seconds (trimmed 5s from both ends)\n\n")
            
            f.write("File Statistics:\n")
            f.write("-" * 40 + "\n")
            
            for df, filename in self.all_data:
                if df is not None and len(df) > 0:
                    f.write(f"\nFile: {filename}\n")
                    f.write(f"  Data Points: {len(df)}\n")
                    f.write(f"  Duration: {df['time_seconds'].max():.2f} seconds\n")
                    f.write(f"  X-Angle - Mean: {df['x_angle'].mean():.2f}°, Std: {df['x_angle'].std():.2f}°\n")
                    f.write(f"  X-Angular Velocity - Mean: {df['x_angular_velocity'].mean():.2f}°/s, Std: {df['x_angular_velocity'].std():.2f}°/s\n")
                    if 'temp' in df.columns:
                        f.write(f"  Temperature - Mean: {df['temp'].mean():.2f}°C\n")
        
        print(f"Analysis report generated: {report_path}")

def main():
    """
    Main function with automatic folder detection and processing
    """
    # Find all imu_data folders
    imu_data_folders = glob.glob("imu_data_*")
    
    if not imu_data_folders:
        print("No imu_data_* folders found in current directory")
        return
    
    print(f"Found {len(imu_data_folders)} imu_data folders")
    
    # Process each imu_data folder
    for imu_folder in imu_data_folders:
        # Extract suffix from folder name
        suffix = imu_folder.replace("imu_data_", "")
        output_folder = f"analysis_result_{suffix}"
        
        # Check if analysis already exists
        if os.path.exists(output_folder):
            print(f"Analysis already exists for {imu_folder}, skipping...")
            continue
        
        print(f"\nProcessing {imu_folder} -> {output_folder}")
        print("=" * 50)
        
        # Create analyzer and run analysis
        analyzer = IMUDataAnalyzer(imu_folder, output_folder)
        analyzer.analyze_all_files()
    
    # Report unprocessed folders
    print("\n" + "=" * 60)
    print("Processing Summary:")
    processed = [f for f in imu_data_folders if os.path.exists(f"analysis_result_{f.replace('imu_data_', '')}")]
    unprocessed = [f for f in imu_data_folders if not os.path.exists(f"analysis_result_{f.replace('imu_data_', '')}")]
    
    print(f"Processed folders: {len(processed)}")
    print(f"Unprocessed folders: {len(unprocessed)}")
    
    if unprocessed:
        print("\nUnprocessed imu_data folders:")
        for folder in unprocessed:
            print(f"  - {folder}")

if __name__ == "__main__":
    main()