#!/usr/bin/env python3
"""
IMU Data Batch Analyzer
Focus on X-axis angle and angular velocity analysis
Standardized 30-second data recording after trimming 5 seconds from both ends
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
    
    def create_individual_plot(self, df, filename):
        """
        Create individual plot for single file
        
        Parameters:
            df: Preprocessed data
            filename: Filename for title
        """
        if df is None or len(df) == 0:
            print(f"  Cannot create plot for {filename}: No valid data")
            return
        
        # Create figure with two subplots
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 12))
        
        # Set main title
        file_title = os.path.splitext(filename)[0]
        fig.suptitle(f'IMU Data Analysis - {file_title}', fontsize=16, fontweight='bold')
        
        # Plot X-axis angle
        time_seconds = df['time_seconds']
        
        # Angle plot
        ax1.plot(time_seconds, df['x_angle'], 'blue', alpha=0.8, linewidth=2, label='X-Angle')
        ax1.axhline(y=0, color='red', linestyle='--', alpha=0.5, label='Zero Reference')
        
        ax1.set_title('X-Axis Angle Trend (30s Recording)', fontsize=14, fontweight='bold')
        ax1.set_ylabel('Angle (degrees)', fontsize=12)
        ax1.set_xlabel('Time (seconds)', fontsize=12)
        ax1.legend(fontsize=10)
        ax1.grid(True, alpha=0.3)
        
        # Angular velocity plot
        ax2.plot(time_seconds, df['x_angular_velocity'], 'green', alpha=0.7, linewidth=1.5, label='Raw X-Angular Velocity')
        ax2.plot(time_seconds, df['x_angular_velocity_smooth'], 'purple', alpha=0.9, linewidth=2, label='Smoothed X-Angular Velocity')
        ax2.axhline(y=0, color='red', linestyle='--', alpha=0.5, label='Zero Reference')
        
        ax2.set_title('X-Axis Angular Velocity Trend (30s Recording)', fontsize=14, fontweight='bold')
        ax2.set_ylabel('Angular Velocity (degrees/s)', fontsize=12)
        ax2.set_xlabel('Time (seconds)', fontsize=12)
        ax2.legend(fontsize=10)
        ax2.grid(True, alpha=0.3)
        
        # Add statistics
        stats_text = f"""
Statistics:
X-Angle - Mean: {df['x_angle'].mean():.2f}°, Std: {df['x_angle'].std():.2f}°
X-Angular Velocity - Mean: {df['x_angular_velocity'].mean():.2f}°/s, Std: {df['x_angular_velocity'].std():.2f}°/s
Data Points: {len(df)}, Duration: {df['time_seconds'].max():.2f}s
        """
        
        fig.text(0.02, 0.02, stats_text, fontsize=10, 
                bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgray", alpha=0.7))
        
        plt.tight_layout(rect=[0, 0.05, 1, 0.95])
        
        # Save plot
        output_path = os.path.join(self.individual_plots_folder, f"{file_title}_analysis.png")
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"  Individual plot saved: {output_path}")
    
    def create_comparison_plot(self):
        """
        Create comparison plot for all files
        """
        if len(self.all_data) == 0:
            print("No valid data for comparison plot")
            return
        
        # Create comparison plots
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(18, 14))
        
        # Use color map
        colors = plt.cm.tab10(np.linspace(0, 1, len(self.all_data)))
        
        # Plot X-angle comparison
        for i, (df, filename) in enumerate(self.all_data):
            if df is not None and len(df) > 0:
                label = os.path.splitext(filename)[0]
                ax1.plot(df['time_seconds'], df['x_angle'], 
                        color=colors[i], alpha=0.8, linewidth=2, label=label)
        
        ax1.set_title('X-Axis Angle Comparison - All Files (30s Recording)', fontsize=16, fontweight='bold')
        ax1.set_ylabel('X-Angle (degrees)', fontsize=14)
        ax1.set_xlabel('Time (seconds)', fontsize=14)
        ax1.legend(fontsize=10, loc='upper right')
        ax1.grid(True, alpha=0.3)
        
        # Plot X-angular velocity comparison
        for i, (df, filename) in enumerate(self.all_data):
            if df is not None and len(df) > 0:
                label = os.path.splitext(filename)[0]
                ax2.plot(df['time_seconds'], df['x_angular_velocity_smooth'], 
                        color=colors[i], alpha=0.8, linewidth=2, label=label)
        
        ax2.set_title('X-Axis Angular Velocity Comparison - All Files (30s Recording)', fontsize=16, fontweight='bold')
        ax2.set_ylabel('X-Angular Velocity (degrees/s)', fontsize=14)
        ax2.set_xlabel('Time (seconds)', fontsize=14)
        ax2.legend(fontsize=10, loc='upper right')
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save comparison plot
        comparison_path = os.path.join(self.output_folder, "all_files_comparison.png")
        plt.savefig(comparison_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"Comparison plot saved: {comparison_path}")
    
    def analyze_all_files(self):
        """
        Analyze all CSV files in folder
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
                # Create individual plot
                self.create_individual_plot(df_processed, filename)
                
                # Save data for comparison
                self.all_data.append((df_processed, filename))
            
            print("-" * 40)
        
        # Create comparison plot
        if len(self.all_data) > 1:
            print("Creating comparison plot...")
            self.create_comparison_plot()
        
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
        
        with open(report_path, 'w') as f:
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