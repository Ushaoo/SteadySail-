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
        
        # 默认参数
        self.current_params = {
            'gyro': {'kp': 294.0, 'ki': 0.08, 'kd': 0.0, 'target': 0.0},
            'acc': {'kp': 29.0, 'ki': 0.000, 'kd': 0.008, 'target': 0.0},
            'angacc': {'kp': 0.0, 'ki': 0.000, 'kd': 0.008, 'target': 0.0},
            'mode': True,
            'angle_low': 20,
            'angle_high': 30,
            'angle_capsized': 35
        }
        
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
                    'gyro': {'kp': 294.0, 'ki': 0.08, 'kd': 0.0},
                    'acc': {'kp': 29.0, 'ki': 0.000, 'kd': 0.008},
                    'angacc': {'kp': 0.0, 'ki': 0.000, 'kd': 0.008}
                },
                'aggressive': {
                    'gyro': {'kp': 400.0, 'ki': 0.1, 'kd': 0.0},
                    'acc': {'kp': 40.0, 'ki': 0.001, 'kd': 0.01},
                    'angacc': {'kp': 350.0, 'ki': 0.001, 'kd': 0.01}
                },
                'smooth': {
                    'gyro': {'kp': 200.0, 'ki': 0.05, 'kd': 0.0},
                    'acc': {'kp': 20.0, 'ki': 0.000, 'kd': 0.005},
                    'angacc': {'kp': 250.0, 'ki': 0.000, 'kd': 0.005}
                }
            }
    
    def save_presets(self):
        """保存预设配置到文件"""
        with open('pid_presets.json', 'w') as f:
            json.dump(self.presets, f, indent=2)
    
    def update_parameter(self, param_type, param_name, value):
        """更新参数并记录历史"""
        with self.lock:
            # 转换数据类型
            if param_type == 'mode':
                value = value.lower() == 'true' if isinstance(value, str) else bool(value)
            else:
                value = float(value)
                
            old_value = self.current_params[param_type][param_name] if param_name != 'value' else self.current_params[param_type]
            
            # 更新参数
            if param_name == 'value':
                self.current_params[param_type] = value
            else:
                self.current_params[param_type][param_name] = value
            
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
                'web_interface': 'active'
            }
            return jsonify(status)
    
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
        </div>

        <div class="card">
            <h2>Control Mode</h2>
            <div>
                <label>
                    <input type="radio" name="mode" value="true" checked> Gyro+Accel PID
                </label>
                <label>
                    <input type="radio" name="mode" value="false"> Angular Acceleration PID
                </label>
            </div>
        </div>

        <div class="card">
            <h2>Gyro PID Parameters</h2>
            <div class="param-group">
                <div class="param-item">
                    <label>Kp: <span id="gyro-kp-value" class="value-display">294.0</span></label>
                    <input type="range" id="gyro-kp" min="0" max="1000" step="0.1" value="294.0">
                </div>
                <div class="param-item">
                    <label>Ki: <span id="gyro-ki-value" class="value-display">0.08</span></label>
                    <input type="range" id="gyro-ki" min="0" max="1" step="0.001" value="0.08">
                </div>
                <div class="param-item">
                    <label>Kd: <span id="gyro-kd-value" class="value-display">0.0</span></label>
                    <input type="range" id="gyro-kd" min="0" max="10" step="0.001" value="0.0">
                </div>
            </div>
        </div>

        <div class="card">
            <h2>Acceleration PID Parameters</h2>
            <div class="param-group">
                <div class="param-item">
                    <label>Kp: <span id="acc-kp-value" class="value-display">29.0</span></label>
                    <input type="range" id="acc-kp" min="0" max="100" step="0.1" value="29.0">
                </div>
                <div class="param-item">
                    <label>Ki: <span id="acc-ki-value" class="value-display">0.000</span></label>
                    <input type="range" id="acc-ki" min="0" max="0.01" step="0.0001" value="0.000">
                </div>
                <div class="param-item">
                    <label>Kd: <span id="acc-kd-value" class="value-display">0.008</span></label>
                    <input type="range" id="acc-kd" min="0" max="0.1" step="0.001" value="0.008">
                </div>
            </div>
        </div>

        <div class="card">
            <h2>Angle Thresholds</h2>
            <div class="param-group">
                <div class="param-item">
                    <label>Low Angle: <span id="angle-low-value" class="value-display">5</span>°</label>
                    <input type="range" id="angle-low" min="1" max="20" step="0.5" value="5">
                </div>
                <div class="param-item">
                    <label>High Angle: <span id="angle-high-value" class="value-display">10</span>°</label>
                    <input type="range" id="angle-high" min="5" max="30" step="0.5" value="10">
                </div>
                <div class="param-item">
                    <label>Capsized Angle: <span id="angle-capsized-value" class="value-display">35</span>°</label>
                    <input type="range" id="angle-capsized" min="20" max="60" step="1" value="35">
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
            // Gyro PID
            setSliderValue('gyro-kp', currentParams.gyro.kp);
            setSliderValue('gyro-ki', currentParams.gyro.ki);
            setSliderValue('gyro-kd', currentParams.gyro.kd);
            
            // Acc PID
            setSliderValue('acc-kp', currentParams.acc.kp);
            setSliderValue('acc-ki', currentParams.acc.ki);
            setSliderValue('acc-kd', currentParams.acc.kd);
            
            // Angle thresholds
            setSliderValue('angle-low', currentParams.angle_low);
            setSliderValue('angle-high', currentParams.angle_high);
            setSliderValue('angle-capsized', currentParams.angle_capsized);
            
            // Mode
            const modeValue = currentParams.mode.toString();
            document.querySelector(`input[name="mode"][value="${modeValue}"]`).checked = true;
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
                'gyro-kp': ['gyro', 'kp'],
                'gyro-ki': ['gyro', 'ki'],
                'gyro-kd': ['gyro', 'kd'],
                'acc-kp': ['acc', 'kp'],
                'acc-ki': ['acc', 'ki'],
                'acc-kd': ['acc', 'kd'],
                'angle-low': ['angle_low', 'value'],
                'angle-high': ['angle_high', 'value'],
                'angle-capsized': ['angle_capsized', 'value'],
                'mode': ['mode', 'value']
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
            
            // 只保存PID参数，不保存角度阈值和模式
            const presetParams = {
                gyro: currentParams.gyro,
                acc: currentParams.acc,
                angacc: currentParams.angacc
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
        
        // 开始状态更新
        function startStatusUpdates() {
            setInterval(async () => {
                try {
                    const response = await fetch('/api/system/status');
                    const status = await response.json();
                    // 可以在这里更新状态指示器
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