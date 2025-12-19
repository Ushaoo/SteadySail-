import os
import csv
import matplotlib
matplotlib.use('Agg')  # 使用非GUI后端
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import imageio
import gc
import sys
from pathlib import Path

def get_csv_files():
    """获取当前目录中所有的CSV文件"""
    current_dir = Path('.')
    csv_files = list(current_dir.glob('*.csv'))
    return sorted([f for f in csv_files if f.is_file()])

def load_csv_data(csv_file):
    """从CSV文件加载时间戳和roll角度数据"""
    timestamps = []
    roll_angles = []
    
    try:
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    timestamps.append(float(row['timestamp']))
                    roll_angles.append(float(row['roll_deg']))
                except (ValueError, KeyError):
                    continue
    except Exception as e:
        print(f"读取文件 {csv_file} 时出错: {e}")
        return None, None
    
    return np.array(timestamps), np.array(roll_angles)

def animation_exists(csv_file, animation_type):
    """检查动画文件是否已存在"""
    if animation_type == 'curve':
        output_file = csv_file.stem + '_roll_curve.mp4'
    else:  # level
        output_file = csv_file.stem + '_roll_level.mp4'
    return Path(output_file).exists()

def create_curve_animation(csv_file, timestamps, roll_angles):
    """生成5秒滑动窗口的曲线动画"""
    output_filename = csv_file.stem + '_roll_curve.mp4'
    
    print(f"  生成曲线动画: {output_filename}")
    
    frames = []
    y_min = roll_angles.min() - 5
    y_max = roll_angles.max() + 5
    window_size = 5  # 5秒窗口
    
    # 生成每一帧
    for idx in range(len(timestamps)):
        # 获取当前时间
        current_time = timestamps[idx]
        window_start = max(0, current_time - window_size)
        
        # 找到窗口范围内的数据点
        mask = (timestamps >= window_start) & (timestamps <= current_time)
        window_times = timestamps[mask]
        window_angles = roll_angles[mask]
        
        fig, ax = plt.subplots(figsize=(10, 5), dpi=80)
        
        # 设置轴的范围（固定窗口宽度）
        ax.set_xlim(window_start, current_time)
        ax.set_ylim(y_min, y_max)
        ax.set_xlabel('Time (s)', fontsize=10)
        ax.set_ylabel('Roll Angle (deg)', fontsize=10)
        ax.grid(True, alpha=0.3)
        
        # 绘制窗口内的曲线
        if len(window_times) > 0:
            ax.plot(window_times, window_angles, lw=2.5, color='#2E86AB')
        
        # 移除上下左右的边框
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        
        # 转换图形为图像
        fig.canvas.draw()
        image = np.frombuffer(fig.canvas.buffer_rgba(), dtype='uint8')
        w, h = fig.canvas.get_width_height()
        image = image.reshape((h, w, 4))
        image = image[:, :, :3]  # 去掉alpha通道
        frames.append(image)
        
        plt.close(fig)
        
        # 定期清理内存
        if (idx + 1) % 50 == 0:
            gc.collect()
        
        # 每100帧打印进度
        if (idx + 1) % 100 == 0:
            print(f"    进度: {idx + 1}/{len(timestamps)}")
            sys.stdout.flush()
    
    # 保存为视频文件
    try:
        imageio.mimsave(output_filename, frames, fps=30, codec='libx264', quality=5)
        print(f"    ✓ 曲线动画已保存\n")
        sys.stdout.flush()
    except Exception as e:
        print(f"    ✗ 保存曲线动画出错: {e}\n")
        sys.stdout.flush()
    
    # 清理frames内存
    frames.clear()
    gc.collect()

def create_level_animation(csv_file, timestamps, roll_angles):
    """生成水平线旋转动画，展示浆板倾斜程度"""
    output_filename = csv_file.stem + '_roll_level.mp4'
    
    print(f"  生成水平线动画: {output_filename}")
    
    frames = []
    
    # 生成每一帧
    for idx in range(len(timestamps)):
        # 获取当前roll角度
        current_roll = roll_angles[idx]
        
        fig, ax = plt.subplots(figsize=(8, 6), dpi=80)
        
        # 设置背景色
        ax.set_facecolor('white')
        
        # 绘制外圆框
        circle_frame = patches.Circle((0, 0), 1, fill=False, edgecolor='#333333', linewidth=2)
        ax.add_patch(circle_frame)
        
        # 绘制参考线（固定的虚线）
        ax.plot([-1.2, 1.2], [0, 0], 'k--', linewidth=1, alpha=0.2, label='水平参考线')
        ax.plot([0, 0], [-1.2, 1.2], 'k--', linewidth=1, alpha=0.2)
        
        # 绘制旋转的水平线（表示浆板）
        # 线的长度和厚度
        line_length = 0.9
        # 计算旋转后的线端点
        angle_rad = np.radians(current_roll)
        x_end = line_length * np.cos(angle_rad)
        y_end = line_length * np.sin(angle_rad)
        ax.plot([-x_end, x_end], [-y_end, y_end], color='#2E86AB', linewidth=4, 
                label='浆板倾斜线', zorder=10)
        
        # 绘制中心点
        ax.plot(0, 0, 'o', color='#2E86AB', markersize=8, zorder=11)
        
        # 绘制刻度标记（度数）
        for angle in np.arange(0, 360, 15):
            if angle % 30 == 0:  # 主刻度
                radius_outer = 1.05
                radius_inner = 0.95
                linewidth = 1.5
            else:  # 副刻度
                radius_outer = 1.03
                radius_inner = 0.97
                linewidth = 0.8
            
            angle_rad = np.radians(angle)
            x_in = radius_inner * np.cos(angle_rad)
            y_in = radius_inner * np.sin(angle_rad)
            x_out = radius_outer * np.cos(angle_rad)
            y_out = radius_outer * np.sin(angle_rad)
            ax.plot([x_in, x_out], [y_in, y_out], 'k-', linewidth=linewidth, alpha=0.5)
        
        # 显示当前roll角度
        ax.text(0, -1.5, f'Roll: {current_roll:.1f}°', 
               ha='center', va='top', fontsize=14, fontweight='bold', color='#2E86AB')
        
        # 设置轴属性
        ax.set_xlim(-1.7, 1.7)
        ax.set_ylim(-1.7, 1.7)
        ax.set_aspect('equal')
        ax.axis('off')
        
        # 转换图形为图像
        fig.canvas.draw()
        image = np.frombuffer(fig.canvas.buffer_rgba(), dtype='uint8')
        w, h = fig.canvas.get_width_height()
        image = image.reshape((h, w, 4))
        image = image[:, :, :3]  # 去掉alpha通道
        frames.append(image)
        
        plt.close(fig)
        
        # 定期清理内存
        if (idx + 1) % 50 == 0:
            gc.collect()
        
        # 每100帧打印进度
        if (idx + 1) % 100 == 0:
            print(f"    进度: {idx + 1}/{len(timestamps)}")
            sys.stdout.flush()
    
    # 保存为视频文件
    try:
        imageio.mimsave(output_filename, frames, fps=30, codec='libx264', quality=5)
        print(f"    ✓ 水平线动画已保存\n")
        sys.stdout.flush()
    except Exception as e:
        print(f"    ✗ 保存水平线动画出错: {e}\n")
        sys.stdout.flush()
    
    # 清理frames内存
    frames.clear()
    gc.collect()

def main():
    """主函数 - 单次运行，生成一个视频后停止"""
    csv_files = get_csv_files()
    
    if not csv_files:
        print("当前目录中未找到CSV文件")
        return
    
    print(f"找到 {len(csv_files)} 个CSV文件\n")
    
    for csv_file in csv_files:
        print(f"处理: {csv_file.name}")
        
        # 加载数据
        timestamps, roll_angles = load_csv_data(csv_file)
        
        if timestamps is None or len(timestamps) == 0:
            print(f"  ✗ 无法加载数据\n")
            continue
        
        print(f"  总帧数: {len(timestamps)}")
        
        # 检查并生成曲线动画
        if animation_exists(csv_file, 'curve'):
            print(f"  ⓘ 曲线动画已存在，跳过")
        else:
            create_curve_animation(csv_file, timestamps, roll_angles)
            # 生成完一个视频就停止
            del timestamps, roll_angles
            gc.collect()
            return
        
        # 检查并生成水平线动画
        if animation_exists(csv_file, 'level'):
            print(f"  ⓘ 水平线动画已存在，跳过")
        else:
            create_level_animation(csv_file, timestamps, roll_angles)
            # 生成完一个视频就停止
            del timestamps, roll_angles
            gc.collect()
            return
        
        # 如果两个动画都已存在，打印已完成并继续下一个
        print(f"  ✓ 该文件所有动画已完成\n")
        
        # 清理数据和垃圾
        del timestamps, roll_angles
        gc.collect()
    
    print("所有CSV文件已检查完毕！")
    print("如需生成更多视频，请重新运行脚本。")

if __name__ == '__main__':
    main()
