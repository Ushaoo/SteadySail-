# Motor Control Stability - Quick Reference Card

## 🎯 System Goal
Reduce roll angle oscillation and prevent falling through motor control

## 📊 Core Metrics at a Glance

| Metric | Best Test Value | What It Means | Lower is Better |
|--------|-----------------|---------------|-----------------|
| **Roll Std Dev** | 2.535° | How stable (low = stable) | ✅ Yes |
| **Roll Range** | 19.292° | Total oscillation swing | ✅ Yes |
| **Angular Velocity Max** | 17.207°/s | Peak rotation speed | ✅ Yes |
| **Dominant Frequency** | 0.129 Hz | Oscillation frequency | ✅ Yes |
| **Angular Accel Max** | 385.135°/s² | Peak correction force | ✅ Yes |

## 📈 Performance vs Baseline

| Metric | Baseline | Best | Improvement |
|--------|----------|------|-------------|
| Roll Std | 4.120° | 2.535° | **38% Better** 🎉 |
| Angular Velocity | 34.415°/s | 17.207°/s | **50% Better** 🎉 |
| Dominant Frequency | 0.495 Hz | 0.129 Hz | **74% Better** 🎉 |

## ✅ Good Indicators
- Roll Std < 2.5° (Excellent)
- Roll Range < 20° (Safe)
- Angular Velocity Max < 20°/s (Gentle)
- Dominant Frequency < 0.3 Hz (Slow oscillations)

## ⚠️ Warning Signs
- Roll Std > 4.0° (Excessive)
- Roll Range > 21° (Risky)
- Angular Velocity > 35°/s (Aggressive)
- Dominant Frequency > 0.6 Hz (Fast, unstable)

## 📁 Key Files

```
📊 analysis_results/
   ├─ figures/
   │  └─ *_analysis.png (8-panel detailed charts)
   ├─ stability_report.txt (numerical data)
   └─ analysis_data.json (machine-readable)

📊 comparison_results/
   ├─ comparison_analysis.png (9-panel comparison)
   └─ comparison_report.txt (statistics)
```

## 🚀 Quick Commands

```bash
# Run complete analysis
/home/pi/Desktop/motor_env/bin/python /home/pi/Desktop/run_stability_analysis.py

# Or run individual components
/home/pi/Desktop/motor_env/bin/python /home/pi/Desktop/system_stability_analyzer.py
/home/pi/Desktop/motor_env/bin/python /home/pi/Desktop/stability_comparator.py
```

## 📖 Documentation

- **STABILITY_ANALYSIS_GUIDE.md** - Complete metric explanations
- **NEW_ANALYSIS_SUMMARY.md** - Overview of analysis system
- **ANALYSIS_RESULTS_SUMMARY.txt** - This analysis run details

## 🎓 Understanding the Numbers

### Roll Angle Metrics
- **Mean**: Average angle (center of oscillation)
- **Std**: Variability (lower = more stable)
- **Range**: Max swing (narrower = safer)

### Angular Velocity Metrics  
- **Std**: Consistency of rotation rate
- **Max**: Peak speed (affects tipping risk)

### Frequency Analysis
- **Dominant Freq**: Main oscillation frequency
- **Energy Distribution**: Where the motion "energy" is concentrated
  - Low freq (<0.5 Hz) = Slow, manageable
  - High freq (>2 Hz) = Fast, unstable

---
**Framework**: Focus on concrete physical metrics, not abstract scores  
**Best Configuration**: `feedforward_dual_imu_20251213_181510` (38% better than baseline)
