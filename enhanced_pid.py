#!/usr/bin/env python3
# enhanced_pid.py
# 在文件顶部添加滤波器类
class LowPassFilter:
    """一阶低通滤波器"""
    def __init__(self, alpha=0.5):
        self.alpha = alpha  # 滤波系数，0-1之间，越小滤波越强
        self.prev_value = 0
        self.initialized = False
    
    def update(self, value):
        if not self.initialized:
            self.prev_value = value
            self.initialized = True
            return value
        
        filtered = self.alpha * value + (1 - self.alpha) * self.prev_value
        self.prev_value = filtered
        return filtered
    
    def reset(self):
        self.initialized = False

class AdaptiveLowPassFilter:
    """自适应低通滤波器，基于角度动态调整滤波强度"""
    def __init__(self, base_alpha=0.25):
        self.base_alpha = base_alpha
        self.filter = LowPassFilter(base_alpha)
        self.prev_angle = 0
        
    def update(self, value, current_angle):
        # 根据角度绝对值调整滤波强度
        angle_abs = abs(current_angle)
        
        # 动态计算alpha：角度越大，alpha越大（滤波越弱）
        if angle_abs < 5:  # 小角度，强滤波
            alpha = 0.1  # τ≈0.09s
        elif angle_abs < 15:  # 中等角度，中等滤波
            alpha = 0.25  # τ≈0.03s
        elif angle_abs < 25:  # 较大角度，弱滤波
            alpha = 0.5  # τ≈0.01s
        else:  # 大角度（接近倾覆），几乎不过滤
            alpha = 0.9  # τ≈0.001s
        
        # 更新滤波器参数
        self.filter.alpha = alpha
        self.prev_angle = current_angle
        
        return self.filter.update(value)
    
    def reset(self):
        self.filter.reset()

class EnhancedPIDController:
    """
    增强的PID控制器
    支持运行时参数更新和性能监控
    """
    
    def __init__(self, kp, ki, kd, target_value=0, name="PID"):
        self.kp = kp
        self.ki = ki  
        self.kd = kd
        self.target = target_value
        self.name = name
        
        # 状态变量
        self.previous_error = 0
        self.integral = 0
        self.last_output = 0
        
        # 性能监控
        self.update_count = 0
        self.max_output = 0
        self.min_output = 0
        
        # 积分限制（抗饱和）
        self.integral_limit = None
        
        # 输出限制
        self.output_limit = None
    
    def update_parameters(self, kp, ki, kd, target=None):
        """动态更新PID参数"""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        if target is not None:
            self.target = target
    
    def set_integral_limit(self, limit):
        """设置积分项限制"""
        self.integral_limit = limit
    
    def set_output_limit(self, limit):
        """设置输出限制"""
        self.output_limit = limit
    
    def gyro_update(self, current_value, dt):
        """陀螺仪PID更新"""
        return self._update(current_value, dt)
    
    def acc_update(self, current_value, dt):
        """加速度计PID更新"""
        return self._update(current_value, dt)
        
    def angacc_update(self, current_value, dt):
        """角加速度PID更新"""
        return self._update(current_value, dt)
    
    def _update(self, current_value, dt):
        """通用的PID更新算法"""
        # 滤波处理
        if self.enable_filter:
            # 对测量值进行自适应滤波
            filtered_value = self.value_filter.update(current_value, self.current_angle)
            current_value = filtered_value
        # 计算误差
        error = self.target - current_value
        
        # 比例项
        proportional = self.kp * error
        
        # 积分项
        self.integral += error * dt
        
        # 积分抗饱和
        if self.integral_limit is not None:
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        
        integral = self.ki * self.integral
        
        # 微分项
        derivative = 0
        if dt > 0:
            raw_derivative = (error - self.previous_error) / dt
            # 对微分项进行滤波
            if self.enable_filter:
                derivative = self.derivative_filter.update(raw_derivative)
            else:
                derivative = raw_derivative
            derivative = self.kd * derivative
        
        # 计算PID输出
        output = proportional + integral + derivative
        
        # 输出限制
        if self.output_limit is not None:
            output = max(min(output, self.output_limit), -self.output_limit)
        
        # 更新状态
        self.previous_error = error
        self.last_output = output
        self.update_count += 1
        
        # 更新性能统计
        if output > self.max_output:
            self.max_output = output
        if output < self.min_output:
            self.min_output = output
        
        return output
    
    def set_target(self, target):
        """设置目标值"""
        self.target = target
        
    def reset(self):
        """重置控制器状态"""
        self.previous_error = 0
        self.integral = 0
        self.last_output = 0
    
    def get_stats(self):
        """获取控制器统计信息"""
        return {
            'name': self.name,
            'kp': self.kp,
            'ki': self.ki,
            'kd': self.kd,
            'target': self.target,
            'update_count': self.update_count,
            'max_output': self.max_output,
            'min_output': self.min_output,
            'last_output': self.last_output
        }
    def set_current_angle(self, angle):
        """设置当前角度，用于自适应滤波"""
        self.current_angle = angle
    
    # 新增方法：更新滤波器参数
    def update_filter_params(self, value_alpha=None, derivative_alpha=None):
        """更新滤波器参数"""
        if value_alpha is not None:
            self.value_filter.base_alpha = value_alpha
        if derivative_alpha is not None and hasattr(self, 'derivative_filter'):
            self.derivative_filter.alpha = derivative_alpha