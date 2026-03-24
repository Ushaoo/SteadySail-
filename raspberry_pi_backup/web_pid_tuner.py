#!/usr/bin/env python3
"""
独立的Web PID参数调节器模块
可以在主程序中导入并使用
"""

import json
import time
import threading
from collections import deque
from datetime import datetime

# Flask for web interface
try:
    from flask import Flask, render_template, request, jsonify
    HAS_FLASK = True
except ImportError:
    HAS_FLASK = False
    print("Flask not available - web interface disabled")

class ParameterManager:
    """管理PID参数和预设配置"""
    
    def __init__(self):
        self.lock = threading.Lock()
        self.parameter_history = deque(maxlen=1000)  # 参数变化历史
        self.presets = self.load_presets()
        
        # 默认参数 - 统一PID
        self.current_params = {
            # 统一PID参数
            'pid': {'kp': 20.0, 'ki': 1.0, 'kd': 0.0},
            # 前馈/反馈参数
            'feedforward_param': 0.28,
            'feedback_param': 0.5,
            # 物理参数
            'mass': 80.0,
            'width': 0.6,
            # 死区参数
            'angle_deadzone': 1.0,
            'angle_deadzone_soft': 3.0,
            'omega_deadzone_soft': 6.0,
            # 电机控制参数
            'motor_left_invert': False,  # 左电机是否反转
            'motor_right_invert': False,  # 右电机是否反转
            'thrust_scale': 0.55  # 力矩到PWM的缩放系数
        }
        
        # 数据记录状态管理
        self.data_logging_enabled = False
        self.data_logger = None  # 引用，由主程序设置
        
        # 回调函数，当参数变化时调用
        self.on_parameter_change = None
    
    def load_presets(self):
        """从文件加载预设配置"""
        try:
            with open('pid_presets.json', 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            # 默认预设
            return {
                'default': {
                    'pid': {'kp': 20.0, 'ki': 1.0, 'kd': 0.0},
                    'feedforward_param': 0.28,
                    'feedback_param': 0.5,
                    'motor_left_invert': False,
                    'motor_right_invert': False
                },
                'aggressive': {
                    'pid': {'kp': 30.0, 'ki': 1.5, 'kd': 0.1},
                    'feedforward_param': 0.35,
                    'feedback_param': 0.6,
                    'motor_left_invert': False,
                    'motor_right_invert': False
                },
                'smooth': {
                    'pid': {'kp': 15.0, 'ki': 0.8, 'kd': 0.05},
                    'feedforward_param': 0.25,
                    'feedback_param': 0.45,
                    'motor_left_invert': False,
                    'motor_right_invert': False
                }
            }
    
    def save_presets(self):
        """保存预设配置到文件"""
        with open('pid_presets.json', 'w') as f:
            json.dump(self.presets, f, indent=2)
    
    def update_parameter(self, param_type, param_name, value):
        """更新参数并记录历史"""
        with self.lock:
            # 转换数据类型 - 处理boolean类型参数
            if param_type in ['motor_left_invert', 'motor_right_invert']:
                # 处理boolean参数
                if isinstance(value, str):
                    value = value.lower() == 'true'
                else:
                    value = bool(value)
            elif param_type == 'mode':
                value = value.lower() == 'true' if isinstance(value, str) else bool(value)
            else:
                value = float(value)
            
            # 获取旧值
            if isinstance(self.current_params.get(param_type), dict):
                old_value = self.current_params[param_type].get(param_name, None)
            else:
                old_value = self.current_params.get(param_type, None)
            
            # 更新参数
            if isinstance(self.current_params.get(param_type), dict):
                if param_type not in self.current_params:
                    self.current_params[param_type] = {}
                self.current_params[param_type][param_name] = value
            else:
                # 顶级参数直接更新
                self.current_params[param_type] = value
            
            # 记录参数变化
            history_entry = {
                'timestamp': time.time(),
                'param_type': param_type,
                'param_name': param_name,
                'old_value': old_value,
                'new_value': value
            }
            self.parameter_history.append(history_entry)
            
            # 调用回调函数
            if self.on_parameter_change:
                self.on_parameter_change(param_type, param_name, old_value, value)
            
            return True
    
    def get_current_params(self):
        """获取当前参数（线程安全）"""
        with self.lock:
            return self.current_params.copy()
    
    def save_preset(self, name, params):
        """保存当前参数为预设"""
        self.presets[name] = params
        self.save_presets()
    
    def delete_preset(self, name):
        """删除预设"""
        if name in self.presets:
            del self.presets[name]
            self.save_presets()
    
    def get_preset(self, name):
        """获取预设参数"""
        return self.presets.get(name)
    
    def get_parameter_history(self, limit=100):
        """获取参数变化历史"""
        return list(self.parameter_history)[-limit:]
    
    def set_parameter_change_callback(self, callback):
        """设置参数变化回调函数"""
        self.on_parameter_change = callback
    
    def set_data_logger(self, logger):
        """由主程序设置DataLogger引用"""
        self.data_logger = logger
    
    def start_data_logging(self):
        """启动数据记录"""
        if self.data_logger is None:
            print("Error: data_logger not set")
            return False
        
        with self.lock:
            if self.data_logging_enabled:
                print("Warning: data logging already enabled")
                return False
            
            try:
                # 获取当前参数供记录使用（不需要再次获得lock）
                current_params = self.current_params.copy()
                params = {
                    'pid_kp': current_params['pid']['kp'],
                    'pid_ki': current_params['pid']['ki'],
                    'pid_kd': current_params['pid']['kd'],
                    'feedforward_param': current_params.get('feedforward_param', 0.28),
                    'feedback_param': current_params.get('feedback_param', 0.5),
                    'mass': current_params.get('mass', 80.0),
                    'width': current_params.get('width', 0.6),
                    'angle_deadzone': current_params['angle_deadzone'],
                    'angle_deadzone_soft': current_params['angle_deadzone_soft'],
                    'omega_deadzone_soft': current_params['omega_deadzone_soft'],
                    'motor_left_invert': current_params.get('motor_left_invert', False),
                    'motor_right_invert': current_params.get('motor_right_invert', False),
                    'thrust_scale': current_params.get('thrust_scale', 0.55),
                }
                self.data_logger.start_logging(params=params)
                self.data_logging_enabled = True
                print("Data logging started successfully")
                return True
            except Exception as e:
                print(f"启动数据记录失败: {e}")
                import traceback
                traceback.print_exc()
                return False
    
    def stop_data_logging(self):
        """停止数据记录"""
        if self.data_logger is None:
            print("Error: data_logger not set")
            return False
        
        with self.lock:
            if not self.data_logging_enabled:
                print("Warning: data logging not enabled")
                return False
            
            try:
                self.data_logger.stop_logging()
                self.data_logging_enabled = False
                print("Data logging stopped successfully")
                return True
            except Exception as e:
                print(f"停止数据记录失败: {e}")
                import traceback
                traceback.print_exc()
                return False
    
    def is_data_logging_enabled(self):
        """获取数据记录状态"""
        with self.lock:
            return self.data_logging_enabled
    
    def get_control_params(self):
        """获取用于控制系统的参数（与feedforward_dual_imu.py中的ParameterSync兼容）"""
        params = self.get_current_params()
        return {
            'pid_kp': params['pid']['kp'],
            'pid_ki': params['pid']['ki'],
            'pid_kd': params['pid']['kd'],
            'feedforward_param': params.get('feedforward_param', 0.28),
            'feedback_param': params.get('feedback_param', 0.5),
            'mass': params.get('mass', 80.0),
            'width': params.get('width', 0.6),
            'angle_deadzone': params['angle_deadzone'],
            'angle_deadzone_soft': params['angle_deadzone_soft'],
            'omega_deadzone_soft': params['omega_deadzone_soft'],
            'motor_left_invert': params.get('motor_left_invert', False),
            'motor_right_invert': params.get('motor_right_invert', False),
            'thrust_scale': params.get('thrust_scale', 0.55),
        }

class WebPIDTuner:
    """
    Web PID参数调节器
    提供Web界面实时调整PID参数
    """
    
    def __init__(self, param_manager, port=5000, host='0.0.0.0'):
        if not HAS_FLASK:
            raise RuntimeError("Flask is required for web interface")
            
        self.param_manager = param_manager
        self.port = port
        self.host = host
        self.app = Flask(__name__)
        self.thread = None
        self.running = False
        
        self.setup_routes()
        self.create_web_template()
    
    def setup_routes(self):
        """设置Flask路由"""
        
        @self.app.route('/')
        def index():
            return self.get_web_interface()
        
        @self.app.route('/api/parameters', methods=['GET'])
        def get_parameters():
            """获取当前所有参数"""
            params = self.param_manager.get_current_params()
            presets = self.param_manager.presets
            return jsonify({
                'current': params,
                'presets': presets
            })
        
        @self.app.route('/api/parameters/<param_type>/<param_name>', methods=['POST'])
        def update_parameter(param_type, param_name):
            """更新单个参数"""
            value = request.json.get('value')
            if value is None:
                return jsonify({'error': 'No value provided'}), 400
            
            success = self.param_manager.update_parameter(param_type, param_name, value)
            if success:
                return jsonify({'success': True})
            else:
                return jsonify({'error': 'Invalid parameter'}), 400
        
        @self.app.route('/api/presets', methods=['POST'])
        def save_preset():
            """保存预设"""
            data = request.json
            name = data.get('name')
            params = data.get('parameters')
            
            if not name or not params:
                return jsonify({'error': 'Name and parameters required'}), 400
            
            self.param_manager.save_preset(name, params)
            return jsonify({'success': True})
        
        @self.app.route('/api/presets/<name>', methods=['DELETE'])
        def delete_preset(name):
            """删除预设"""
            self.param_manager.delete_preset(name)
            return jsonify({'success': True})
        
        @self.app.route('/api/presets/<name>/load', methods=['POST'])
        def load_preset(name):
            """加载预设"""
            preset = self.param_manager.get_preset(name)
            if not preset:
                return jsonify({'error': 'Preset not found'}), 404
            
            # 应用预设参数
            for param_type, params in preset.items():
                for param_name, value in params.items():
                    self.param_manager.update_parameter(param_type, param_name, value)
            
            return jsonify({'success': True})
        
        @self.app.route('/api/history', methods=['GET'])
        def get_history():
            """获取参数变化历史"""
            limit = request.args.get('limit', 100, type=int)
            history = self.param_manager.get_parameter_history(limit)
            return jsonify(history)
        
        @self.app.route('/api/system/status', methods=['GET'])
        def get_system_status():
            """获取系统状态"""
            status = {
                'running': self.running,
                'timestamp': time.time(),
                'web_interface': 'active',
                'data_logging': self.param_manager.is_data_logging_enabled()
            }
            return jsonify(status)
        
        @self.app.route('/api/data-logging/start', methods=['POST'])
        def start_logging():
            """启动数据记录"""
            if self.param_manager.start_data_logging():
                return jsonify({'success': True, 'message': '数据记录已启动'})
            else:
                return jsonify({'error': '数据记录启动失败或已运行'}), 400
        
        @self.app.route('/api/data-logging/stop', methods=['POST'])
        def stop_logging():
            """停止数据记录"""
            if self.param_manager.stop_data_logging():
                return jsonify({'success': True, 'message': '数据记录已停止'})
            else:
                return jsonify({'error': '数据记录停止失败'}), 400
        
        @self.app.route('/api/data-logging/status', methods=['GET'])
        def logging_status():
            """获取数据记录状态"""
            return jsonify({
                'enabled': self.param_manager.is_data_logging_enabled(),
                'timestamp': time.time()
            })
    
    def get_web_interface(self):
        """返回Web界面HTML"""
        return """
<!DOCTYPE html>
<html>
<head>
    <title>Motor Control PID Tuner</title>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f5f5f5; }
        .container { max-width: 1200px; margin: 0 auto; }
        .card { background: white; padding: 20px; margin: 10px 0; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .param-group { display: flex; flex-wrap: wrap; gap: 20px; margin: 10px 0; }
        .param-item { flex: 1; min-width: 200px; }
        label { display: block; margin: 5px 0; font-weight: bold; }
        input[type="range"] { width: 100%; }
        .value-display { display: inline-block; width: 60px; text-align: right; }
        .preset-section { display: flex; gap: 10px; align-items: center; }
        button { padding: 8px 16px; margin: 5px; border: none; border-radius: 4px; cursor: pointer; }
        .btn-primary { background: #007bff; color: white; }
        .btn-success { background: #28a745; color: white; }
        .btn-danger { background: #dc3545; color: white; }
        .history-item { padding: 5px; border-bottom: 1px solid #eee; font-size: 12px; }
        .status-indicator { width: 10px; height: 10px; border-radius: 50%; display: inline-block; margin-right: 5px; }
        .status-online { background: #28a745; }
        .status-offline { background: #dc3545; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Motor Control PID Tuner</h1>
        
        <div class="card">
            <h2>System Status</h2>
            <div id="system-status">
                <span class="status-indicator status-online"></span>
                <span>Web Interface Active</span>
            </div>
            <div style="margin-top: 15px;">
                <p><strong>Data Logging Status:</strong> <span id="logging-status-text" style="color: #dc3545;">Disabled</span></p>
                <div style="display: flex; gap: 10px;">
                    <button class="btn-success" onclick="startDataLogging()">Start Recording</button>
                    <button class="btn-danger" onclick="stopDataLogging()">Stop Recording</button>
                </div>
            </div>
        </div>

        <div class="card">
            <h2>PID Parameters (Unified)</h2>
            <div class="param-group">
                <div class="param-item">
                    <label>Kp: <span id="pid-kp-value" class="value-display">20.0</span></label>
                    <input type="range" id="pid-kp" min="0" max="100" step="0.1" value="20.0">
                </div>
                <div class="param-item">
                    <label>Ki: <span id="pid-ki-value" class="value-display">1.0</span></label>
                    <input type="range" id="pid-ki" min="0" max="10" step="0.1" value="1.0">
                </div>
                <div class="param-item">
                    <label>Kd: <span id="pid-kd-value" class="value-display">0.0</span></label>
                    <input type="range" id="pid-kd" min="0" max="1" step="0.01" value="0.0">
                </div>
            </div>
        </div>

        <div class="card">
            <h2>Feedforward & Feedback Control</h2>
            <div class="param-group">
                <div class="param-item">
                    <label>Feedforward Param: <span id="feedforward-param-value" class="value-display">0.28</span></label>
                    <input type="range" id="feedforward-param" min="0" max="1" step="0.01" value="0.28">
                </div>
                <div class="param-item">
                    <label>Feedback Param: <span id="feedback-param-value" class="value-display">0.50</span></label>
                    <input type="range" id="feedback-param" min="0" max="1" step="0.01" value="0.50">
                </div>
            </div>
        </div>

        <div class="card">
            <h2>Physical Parameters</h2>
            <div class="param-group">
                <div class="param-item">
                    <label>Mass: <span id="mass-value" class="value-display">80.0</span> kg</label>
                    <input type="range" id="mass" min="50" max="150" step="1" value="80.0">
                </div>
                <div class="param-item">
                    <label>Width: <span id="width-value" class="value-display">0.6</span> m</label>
                    <input type="range" id="width" min="0.3" max="1.0" step="0.05" value="0.6">
                </div>
            </div>
        </div>

        <div class="card">
            <h2>Motor Control</h2>
            <div class="param-group">
                <div class="param-item">
                    <label>Thrust Scale: <span id="thrust-scale-value" class="value-display">0.55</span></label>
                    <input type="range" id="thrust-scale" min="0" max="2" step="0.01" value="0.55">
                </div>
            </div>
            <div class="param-group">
                <div class="param-item">
                    <label style="display: flex; align-items: center; gap: 10px;">
                        <input type="checkbox" id="motor-left-invert" style="width: auto; cursor: pointer;">
                        <span>Left Motor Reverse (Normal: PWM = pulse_left, Reverse: PWM = 3000 - pulse_left)</span>
                    </label>
                </div>
                <div class="param-item">
                    <label style="display: flex; align-items: center; gap: 10px;">
                        <input type="checkbox" id="motor-right-invert" style="width: auto; cursor: pointer;">
                        <span>Right Motor Reverse (Normal: PWM = pulse_right, Reverse: PWM = 3000 - pulse_right)</span>
                    </label>
                </div>
            </div>
        </div>

        <div class="card">
            <h2>Deadzone Parameters</h2>
            <div class="param-group">
                <div class="param-item">
                    <label>Angle Deadzone: <span id="angle-deadzone-value" class="value-display">1.0</span>°</label>
                    <input type="range" id="angle-deadzone" min="0" max="5" step="0.1" value="1.0">
                </div>
                <div class="param-item">
                    <label>Angle Deadzone Soft: <span id="angle-deadzone-soft-value" class="value-display">3.0</span>°</label>
                    <input type="range" id="angle-deadzone-soft" min="0" max="10" step="0.1" value="3.0">
                </div>
                <div class="param-item">
                    <label>Omega Deadzone: <span id="omega-deadzone-soft-value" class="value-display">6.0</span>°/s</label>
                    <input type="range" id="omega-deadzone-soft" min="0" max="20" step="0.5" value="6.0">
                </div>
            </div>
        </div>

        <div class="card">
            <h2>Preset Management</h2>
            <div class="preset-section">
                <select id="preset-select">
                    <option value="">Select Preset</option>
                </select>
                <button class="btn-primary" onclick="loadPreset()">Load Preset</button>
                <button class="btn-success" onclick="savePreset()">Save Current as Preset</button>
                <button class="btn-danger" onclick="deletePreset()">Delete Preset</button>
            </div>
            <div>
                <input type="text" id="preset-name" placeholder="New preset name">
            </div>
        </div>

        <div class="card">
            <h2>Parameter History</h2>
            <div id="parameter-history" style="max-height: 200px; overflow-y: auto;">
                <!-- History will be populated here -->
            </div>
        </div>
    </div>

    <script>
        let currentParams = {};
        
        // 初始化页面
        async function initialize() {
            await loadParameters();
            setupEventListeners();
            startStatusUpdates();
        }
        
        // 加载当前参数
        async function loadParameters() {
            try {
                const response = await fetch('/api/parameters');
                const data = await response.json();
                currentParams = data.current;
                
                // 更新滑块和显示值
                updateSliders();
                updatePresets(data.presets);
                
            } catch (error) {
                console.error('Failed to load parameters:', error);
            }
        }
        
        // 更新滑块位置
        function updateSliders() {
            // Unified PID
            setSliderValue('pid-kp', currentParams.pid.kp);
            setSliderValue('pid-ki', currentParams.pid.ki);
            setSliderValue('pid-kd', currentParams.pid.kd);
            
            // Physical parameters
            setSliderValue('mass', currentParams.mass);
            setSliderValue('width', currentParams.width);
            
            // Deadzone parameters
            setSliderValue('angle-deadzone', currentParams.angle_deadzone);
            setSliderValue('angle-deadzone-soft', currentParams.angle_deadzone_soft);
            setSliderValue('omega-deadzone-soft', currentParams.omega_deadzone_soft);
            
            // Feedforward & Feedback parameters
            setSliderValue('feedforward-param', currentParams.feedforward_param);
            setSliderValue('feedback-param', currentParams.feedback_param);
            
            // Motor control parameters
            setSliderValue('thrust-scale', currentParams.thrust_scale);
            setCheckboxValue('motor-left-invert', currentParams.motor_left_invert);
            setCheckboxValue('motor-right-invert', currentParams.motor_right_invert);
        }
        
        function setCheckboxValue(checkboxId, value) {
            const checkbox = document.getElementById(checkboxId);
            if (checkbox) {
                checkbox.checked = value;
            }
        }
        
        function setSliderValue(sliderId, value) {
            const slider = document.getElementById(sliderId);
            const valueDisplay = document.getElementById(sliderId + '-value');
            if (slider && valueDisplay) {
                slider.value = value;
                valueDisplay.textContent = value.toFixed(3);
            }
        }
        
        // 更新预设列表
        function updatePresets(presets) {
            const select = document.getElementById('preset-select');
            select.innerHTML = '<option value="">Select Preset</option>';
            
            for (const [name, params] of Object.entries(presets)) {
                const option = document.createElement('option');
                option.value = name;
                option.textContent = name;
                select.appendChild(option);
            }
        }
        
        // 设置事件监听器
        function setupEventListeners() {
            // 电机控制复选框
            document.querySelectorAll('input[type="checkbox"]').forEach(checkbox => {
                checkbox.addEventListener('change', function() {
                    updateParameter(this.id, this.checked);
                });
            });
            
            // PID参数滑块
            document.querySelectorAll('input[type="range"]').forEach(slider => {
                slider.addEventListener('input', function() {
                    const valueDisplay = document.getElementById(this.id + '-value');
                    valueDisplay.textContent = parseFloat(this.value).toFixed(3);
                    updateParameter(this.id, this.value);
                });
            });
            
            // 模式选择
            document.querySelectorAll('input[name="mode"]').forEach(radio => {
                radio.addEventListener('change', function() {
                    updateParameter('mode', this.value);
                });
            });
        }
        
        // 更新参数到服务器
        async function updateParameter(paramId, value) {
            let paramType, paramName;
            
            // 映射参数ID到类型和名称
            const paramMap = {
                'pid-kp': ['pid', 'kp'],
                'pid-ki': ['pid', 'ki'],
                'pid-kd': ['pid', 'kd'],
                'mass': ['mass', 'value'],
                'width': ['width', 'value'],
                'angle-deadzone': ['angle_deadzone', 'value'],
                'angle-deadzone-soft': ['angle_deadzone_soft', 'value'],
                'omega-deadzone-soft': ['omega_deadzone_soft', 'value'],
                'feedforward-param': ['feedforward_param', 'value'],
                'feedback-param': ['feedback_param', 'value'],
                'thrust-scale': ['thrust_scale', 'value'],
                'motor-left-invert': ['motor_left_invert', 'value'],
                'motor-right-invert': ['motor_right_invert', 'value']
            };
            
            if (paramMap[paramId]) {
                [paramType, paramName] = paramMap[paramId];
            } else {
                console.error('Unknown parameter:', paramId);
                return;
            }
            
            try {
                const response = await fetch(`/api/parameters/${paramType}/${paramName}`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ value: value })
                });
                
                if (!response.ok) {
                    console.error('Failed to update parameter');
                }
                
                // 更新历史记录
                await updateHistory();
                
            } catch (error) {
                console.error('Error updating parameter:', error);
            }
        }
        
        // 加载预设
        async function loadPreset() {
            const select = document.getElementById('preset-select');
            const presetName = select.value;
            
            if (!presetName) return;
            
            try {
                const response = await fetch(`/api/presets/${presetName}/load`, {
                    method: 'POST'
                });
                
                if (response.ok) {
                    await loadParameters(); // 重新加载参数更新界面
                    alert(`Preset "${presetName}" loaded successfully`);
                }
            } catch (error) {
                console.error('Error loading preset:', error);
            }
        }
        
        // 保存预设
        async function savePreset() {
            const nameInput = document.getElementById('preset-name');
            const presetName = nameInput.value.trim();
            
            if (!presetName) {
                alert('Please enter a preset name');
                return;
            }
            
            // 只保存PID参数
            const presetParams = {
                pid: currentParams.pid,
                feedforward_param: currentParams.feedforward_param,
                feedback_param: currentParams.feedback_param
            };
            
            try {
                const response = await fetch('/api/presets', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        name: presetName,
                        parameters: presetParams
                    })
                });
                
                if (response.ok) {
                    await loadParameters(); // 刷新预设列表
                    nameInput.value = '';
                    alert(`Preset "${presetName}" saved successfully`);
                }
            } catch (error) {
                console.error('Error saving preset:', error);
            }
        }
        
        // 删除预设
        async function deletePreset() {
            const select = document.getElementById('preset-select');
            const presetName = select.value;
            
            if (!presetName) return;
            
            if (!confirm(`Are you sure you want to delete preset "${presetName}"?`)) {
                return;
            }
            
            try {
                const response = await fetch(`/api/presets/${presetName}`, {
                    method: 'DELETE'
                });
                
                if (response.ok) {
                    await loadParameters(); // 刷新预设列表
                    alert(`Preset "${presetName}" deleted successfully`);
                }
            } catch (error) {
                console.error('Error deleting preset:', error);
            }
        }
        
        // 更新参数历史
        async function updateHistory() {
            try {
                const response = await fetch('/api/history?limit=10');
                const history = await response.json();
                
                const historyContainer = document.getElementById('parameter-history');
                historyContainer.innerHTML = '';
                
                history.slice().reverse().forEach(entry => {
                    const item = document.createElement('div');
                    item.className = 'history-item';
                    const date = new Date(entry.timestamp * 1000).toLocaleTimeString();
                    item.textContent = `${date} - ${entry.param_type}.${entry.param_name}: ${entry.old_value} → ${entry.new_value}`;
                    historyContainer.appendChild(item);
                });
                
            } catch (error) {
                console.error('Error loading history:', error);
            }
        }
        
        // 启动数据记录
        async function startDataLogging() {
            try {
                const response = await fetch('/api/data-logging/start', {
                    method: 'POST'
                });
                
                if (response.ok) {
                    const data = await response.json();
                    console.log(data.message);
                    updateLoggingStatus();
                } else {
                    const error = await response.json();
                    alert(`Error: ${error.error}`);
                }
            } catch (error) {
                console.error('Error starting data logging:', error);
                alert('Failed to start data logging');
            }
        }
        
        // 停止数据记录
        async function stopDataLogging() {
            try {
                const response = await fetch('/api/data-logging/stop', {
                    method: 'POST'
                });
                
                if (response.ok) {
                    const data = await response.json();
                    console.log(data.message);
                    updateLoggingStatus();
                } else {
                    const error = await response.json();
                    alert(`Error: ${error.error}`);
                }
            } catch (error) {
                console.error('Error stopping data logging:', error);
                alert('Failed to stop data logging');
            }
        }
        
        // 更新数据记录状态显示
        async function updateLoggingStatus() {
            try {
                const response = await fetch('/api/data-logging/status');
                const status = await response.json();
                
                const statusText = document.getElementById('logging-status-text');
                if (statusText) {
                    if (status.enabled) {
                        statusText.textContent = 'Recording...';
                        statusText.style.color = '#28a745';
                    } else {
                        statusText.textContent = 'Disabled';
                        statusText.style.color = '#dc3545';
                    }
                }
            } catch (error) {
                console.error('Error updating logging status:', error);
            }
        }
        
        // 开始状态更新
        function startStatusUpdates() {
            // 初始化时更新一次
            updateLoggingStatus();
            
            setInterval(async () => {
                try {
                    const response = await fetch('/api/system/status');
                    const status = await response.json();
                    // 更新数据记录状态
                    const statusText = document.getElementById('logging-status-text');
                    if (statusText) {
                        if (status.data_logging) {
                            statusText.textContent = 'Recording...';
                            statusText.style.color = '#28a745';
                        } else {
                            statusText.textContent = 'Disabled';
                            statusText.style.color = '#dc3545';
                        }
                    }
                } catch (error) {
                    console.error('Error updating status:', error);
                }
            }, 5000); // 每5秒更新一次
        }
        
        // 初始化页面
        document.addEventListener('DOMContentLoaded', initialize);
    </script>
</body>
</html>
"""
    
    def create_web_template(self):
        """创建Web界面模板文件（如果需要的话）"""
        # 这个版本我们直接在代码中返回HTML，不需要外部模板文件
        pass
    
    def run_server(self):
        """运行Flask服务器"""
        self.running = True
        print(f"Starting Web PID Tuner on http://{self.host}:{self.port}")
        self.app.run(host=self.host, port=self.port, threaded=True, debug=False)
    
    def start(self):
        """在后台线程中启动Web服务器"""
        if not HAS_FLASK:
            print("Warning: Flask not available, web interface disabled")
            return False
            
        self.thread = threading.Thread(target=self.run_server, daemon=True)
        self.thread.start()
        return True
    
    def stop(self):
        """停止Web服务器"""
        self.running = False
        # Flask没有内置的停止方法，通常通过信号停止
        # 这里主要标记状态，实际停止需要外部干预
    
    def is_running(self):
        """检查Web服务器是否在运行"""
        return self.running and self.thread and self.thread.is_alive()

# 简化的使用示例
if __name__ == "__main__":
    # 独立测试Web PID调节器
    param_manager = ParameterManager()
    tuner = WebPIDTuner(param_manager, port=5000)
    
    print("Starting standalone Web PID Tuner...")
    print("Access the web interface at: http://localhost:5000")
    
    try:
        tuner.run_server()
    except KeyboardInterrupt:
        print("\nWeb PID Tuner stopped.")