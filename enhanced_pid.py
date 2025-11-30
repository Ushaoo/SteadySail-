#!/usr/bin/env python3
"""
增强的PID控制器，支持动态参数更新
"""

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
            derivative = self.kd * (error - self.previous_error) / dt
        
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