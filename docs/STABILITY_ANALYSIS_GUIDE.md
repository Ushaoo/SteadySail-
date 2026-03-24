# Motor Control Stability Analysis Guide

## Overview

This analysis system focuses on **concrete physical metrics** to evaluate the stability and effectiveness of the motor control system for roll angle reduction. Rather than abstract scores, it measures actual behavior: how much the system oscillates, how fast it reacts, and how well it maintains stability.

## Key Metrics Explained

### Roll Angle Metrics

- **Mean Roll (mean_roll)**: Average roll angle during the test
  - **Ideal**: Close to 0° (system is balanced on average)
  - **Lower is Better**: Indicates better stabilization

- **Roll Std Dev (roll_std)**: Standard deviation of roll angle
  - **Measures**: How much the angle fluctuates around the mean
  - **Lower is Better**: Smaller fluctuations = more stable system
  - **For this system**: Values around 2.5-4.2° are expected

- **Roll Range (roll_range)**: Maximum difference between highest and lowest roll angle
  - **Measures**: Total spread of oscillation during the test
  - **Lower is Better**: Less likely to tip over
  - **Target**: Stay under 20° range

- **RMS Roll (rms_roll)**: Root mean square of roll angle
  - **Measures**: Overall magnitude of roll movement
  - **Lower is Better**: Indicates stable operation

### Angular Velocity Metrics

- **Angular Velocity Std (omega_std)**: Standard deviation of roll rate
  - **Measures**: Consistency of rotational motion
  - **Lower is Better**: Less erratic motion
  - **For this system**: 3-10°/s is typical

- **Max Angular Velocity (omega_max)**: Maximum rotational speed achieved
  - **Measures**: Peak responsiveness/oscillation severity
  - **Lower is Better**: Calmer, more controlled motion
  - **Critical**: Directly related to fall prevention

### Angular Acceleration Metrics

- **Acceleration Std (accel_std)**: Standard deviation of angular acceleration
  - **Measures**: Consistency of control changes
  - **Lower is Better**: Smoother control transitions
  - **For this system**: 50-100°/s² is typical

- **Max Acceleration (accel_max)**: Peak angular acceleration
  - **Measures**: Aggressiveness of control corrections
  - **Lower is Better**: Less jarring movements

### Frequency Domain Analysis

- **Dominant Frequency (dominant_freq)**: Most common oscillation frequency
  - **Measures**: What frequency the system naturally oscillates at
  - **Lower is Better**: Lower frequency = slower, more controllable oscillations
  - **Best Case**: < 0.3 Hz (slow, gentle movements)
  - **Typical**: 0.4-0.6 Hz

- **Frequency Energy Ratios**: How energy is distributed across frequency bands
  - **Low Freq (<0.5 Hz)**: Slow, natural oscillations
  - **Mid Freq (0.5-2 Hz)**: Normal control response
  - **High Freq (>2 Hz)**: Noise or aggressive control
  - **Healthy Pattern**: More energy in low/mid frequencies, less in high frequencies

### PWM Output Analysis

- **PWM Sync Error (pwm_sync_error)**: Difference between left and right motor PWM signals
  - **Measures**: Motor synchronization quality
  - **Lower is Better**: Motors are perfectly matched
  - **Typical**: 150-200 us error is acceptable

## Single Test Analysis

When you run `system_stability_analyzer.py`, it generates:

1. **Analysis Charts**: 8-panel visualization for each test showing:
   - Roll angle time series with mean and std dev bands
   - Angular velocity and acceleration profiles
   - Distribution histograms
   - Control signal breakdown (feedforward vs feedback)
   - Frequency spectrum analysis

2. **Stability Report** (`stability_report.txt`): Complete metrics for each test

3. **JSON Data** (`analysis_data.json`): Machine-readable results for further analysis

## Comparative Analysis

When you run `stability_comparator.py`, it generates:

1. **Comparison Charts**: 9-panel visualization showing:
   - Side-by-side metric comparison across all tests
   - Which test performs best/worst in each metric
   - Normalized heatmap for pattern recognition
   - Stability index showing overall relative performance

2. **Comparison Report** (`comparison_report.txt`): Statistical analysis showing:
   - Mean, standard deviation, min/max for each metric
   - Which test is best/worst for each dimension
   - Improvement potential between best and worst tests

## Interpreting Results

### Best Test Indicators
✓ Roll Std < 2.5°  
✓ Roll Range < 19°  
✓ Max Angular Velocity < 20°/s  
✓ Dominant Frequency < 0.2 Hz  
✓ Low energy in high frequencies  

### Areas of Concern
✗ Roll Std > 4.0°  
✗ Roll Range > 21°  
✗ Max Angular Velocity > 35°/s  
✗ Dominant Frequency > 0.6 Hz  
✗ High energy in high frequencies (noise/instability)  

## System Behavior Analysis

### What These Metrics Tell You

1. **Roll Std + Roll Range** → System stability
   - Small values = system stays balanced
   - Large values = system oscillates excessively

2. **Angular Velocity Max** → Response aggressiveness
   - Small = controlled, gentle corrections
   - Large = sharp, potentially jerky movements

3. **Dominant Frequency** → Natural oscillation pattern
   - Very low (<0.2 Hz) = slow, stable motion
   - Moderate (0.3-0.6 Hz) = normal control response
   - High (>1 Hz) = unstable or noisy control

4. **Angular Acceleration Std** → Control smoothness
   - Small = smooth, consistent control
   - Large = abrupt, inconsistent corrections

## Example Interpretation

Test: `feedforward_dual_imu_20251213_181510`

- Roll Std: 2.535° (Best among tests - very stable!)
- Roll Range: 19.292° (Narrowest containment)
- Max Angular Velocity: 17.207°/s (Smoothest motion)
- Dominant Frequency: 0.1286 Hz (Very low - excellent!)
- Angular Accel Max: 385.135°/s² (Gentlest corrections)

**Conclusion**: This test shows the system at its best - stable, controlled, with slow gentle oscillations that are easy to manage. This is ideal for preventing falls.

## Running the Analysis

```bash
# Single test analysis
/home/pi/Desktop/motor_env/bin/python /home/pi/Desktop/system_stability_analyzer.py

# Comparative analysis (requires single test analysis first)
/home/pi/Desktop/motor_env/bin/python /home/pi/Desktop/stability_comparator.py
```

## Output Files

```
analysis_results/
  ├── figures/
  │   ├── feedforward_dual_imu_*_analysis.png  (Detailed 8-panel charts)
  │   └── ...
  ├── stability_report.txt                      (Complete metrics for all tests)
  └── analysis_data.json                        (Machine-readable results)

comparison_results/
  ├── comparison_analysis.png                   (9-panel comparison visualization)
  └── comparison_report.txt                     (Statistical comparison)
```

## Next Steps

1. **Review the comparison report** to identify which configuration performs best
2. **Examine the dominant frequency** - lower is more stable
3. **Check roll range and velocity** - indicators of safety/control
4. **Look at frequency domain ratios** - should be weighted toward low frequencies
5. **Use metrics to tune control parameters** - focus on reducing dominant frequency and max velocity

---

This analysis framework focuses on **what actually happens** in the physical system, allowing you to make data-driven improvements to motor control for maximum stability and safety.
