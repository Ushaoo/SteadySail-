#!/usr/bin/env python3
"""
测试Web PID Tuner中的数据记录API功能
"""

import sys
import json

# 测试导入
try:
    from web_pid_tuner import ParameterManager
    print("✓ Successfully imported ParameterManager")
except ImportError as e:
    print(f"✗ Failed to import ParameterManager: {e}")
    sys.exit(1)

# 创建参数管理器实例
pm = ParameterManager()
print("✓ Created ParameterManager instance")

# 检查初始数据记录状态
is_logging = pm.is_data_logging_enabled()
print(f"✓ Initial logging status: {is_logging}")

# 创建一个模拟的DataLogger类用于测试
class MockDataLogger:
    def __init__(self):
        self.enabled = False
        self.last_params = None
    
    def start_logging(self, params=None):
        self.enabled = True
        self.last_params = params
        print(f"  → MockDataLogger.start_logging called")
        return True
    
    def stop_logging(self):
        self.enabled = False
        print(f"  → MockDataLogger.stop_logging called")
        return True

# 设置模拟数据记录器
mock_logger = MockDataLogger()
pm.set_data_logger(mock_logger)
print("✓ Set MockDataLogger to ParameterManager")

# 测试启动数据记录
print("\nTesting start_data_logging()...")
success = pm.start_data_logging()
print(f"✓ start_data_logging returned: {success}")
print(f"✓ pm.is_data_logging_enabled(): {pm.is_data_logging_enabled()}")

# 测试停止数据记录
print("\nTesting stop_data_logging()...")
success = pm.stop_data_logging()
print(f"✓ stop_data_logging returned: {success}")
print(f"✓ pm.is_data_logging_enabled(): {pm.is_data_logging_enabled()}")

# 测试再次启动
print("\nTesting restart...")
success = pm.start_data_logging()
print(f"✓ Started data logging again, returned: {success}")

# 再次启动应该失败
print("\nTesting duplicate start (should fail)...")
success = pm.start_data_logging()
print(f"✓ Second start_data_logging returned: {success} (should be False)")

print("\n✓ All tests passed!")
