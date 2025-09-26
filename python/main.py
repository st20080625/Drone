import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial
import time
import math
import re

class QuaternionVisualizer:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        # 初期化フラグ
        self.initialized = False
        
        # データ格納
        self.current_q = [1.0, 0.0, 0.0, 0.0]  # w, x, y, z
        self.target_q = [1.0, 0.0, 0.0, 0.0]   # w, x, y, z
        
        # データ履歴
        self.max_points = 100
        self.time_data = []
        self.current_euler_data = [[], [], []]
        self.target_euler_data = [[], [], []]
        self.error_data = [[], [], []]
        
        self.start_time = time.time()
        
        # シリアル接続
        self.ser = None
        self.serial_port = port
        self.serial_baudrate = baudrate
        self.serial_buffer = ""  # バッファ追加
        
        print("Initializing plots...")
        try:
            self.setup_plots()
            self.initialized = True
            print("Plots initialized successfully")
        except Exception as e:
            print(f"Error initializing plots: {e}")
    
    def connect_serial(self):
        """シリアル接続を試行"""
        if self.ser is not None:
            return True
            
        try:
            print(f"Attempting to connect to {self.serial_port}...")
            self.ser = serial.Serial(self.serial_port, self.serial_baudrate, timeout=0.01)
            self.ser.reset_input_buffer()
            print(f"Connected to {self.serial_port}")
            return True
        except Exception as e:
            print(f"Serial connection failed: {e}")
            return False
    
    def quaternion_to_euler(self, q):
        """クォータニオンをオイラー角に変換（安全版）"""
        try:
            w, x, y, z = q
            
            # NaNチェック
            if not all(math.isfinite(val) for val in [w, x, y, z]):
                return 0.0, 0.0, 0.0
            
            # 正規化
            norm = math.sqrt(w*w + x*x + y*y + z*z)
            if norm < 1e-6:
                return 0.0, 0.0, 0.0
            
            w, x, y, z = w/norm, x/norm, y/norm, z/norm
            
            # Roll (x-axis rotation)
            roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * 180 / math.pi
            
            # Pitch (y-axis rotation)
            sin_pitch = 2 * (w * y - z * x)
            sin_pitch = max(-1.0, min(1.0, sin_pitch))
            pitch = math.asin(sin_pitch) * 180 / math.pi
            
            # Yaw (z-axis rotation)
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * 180 / math.pi
            
            return roll, pitch, yaw
        except Exception as e:
            print(f"Euler conversion error: {e}")
            return 0.0, 0.0, 0.0
    
    def setup_plots(self):
        """プロット設定（軽量版）"""
        plt.rcParams['figure.max_open_warning'] = 0
        plt.ioff()
        
        self.fig, self.axes = plt.subplots(2, 3, figsize=(15, 10))
        self.fig.suptitle('Drone Attitude Visualizer', fontsize=14)
        
        titles = [
            'Current Attitude', 'Target Attitude', 'Euler Angles Comparison',
            'Attitude Error', 'Quaternion Values', 'Real-time Info'
        ]
        
        for i, ax in enumerate(self.axes.flat):
            ax.set_title(titles[i])
            if i < 4:
                ax.grid(True, alpha=0.3)
                ax.set_xlabel('Time [s]')
                ax.set_ylabel('Angle [deg]' if i < 4 else 'Value')
        
        self.axes[1, 2].axis('off')
        plt.tight_layout()
    
    def read_serial_data(self):
        """シリアルデータを読み取り（修正版）"""
        # シリアル接続試行
        if not self.connect_serial():
            # シミュレーションデータ
            t = time.time() - self.start_time
            self.current_q = [
                math.cos(t * 0.1), 
                0.1 * math.sin(t * 0.2), 
                0.05 * math.cos(t * 0.15), 
                0.02 * math.sin(t * 0.05)
            ]
            self.target_q = [
                math.cos(t * 0.05), 
                0.2 * math.sin(t * 0.1), 
                0.1 * math.cos(t * 0.08), 
                0.05 * math.sin(t * 0.03)
            ]
            return True
        
        try:
            # データ読み取り
            if self.ser.in_waiting > 0:
                # バイト単位で読み取りバッファに追加
                data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                self.serial_buffer += data
                
                # パターンマッチングで8つの数値を抽出
                pattern = r'(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)'
                matches = re.findall(pattern, self.serial_buffer)
                
                if matches:
                    # 最新のマッチを取得
                    latest_match = matches[-1]
                    
                    try:
                        # 8つの数値を取得
                        values = [float(x) for x in latest_match]
                        
                        self.current_q = values[:4]   # r, i, j, k
                        self.target_q = values[4:]    # w, x, y, z
                        
                        # デバッグ出力（最初の数回のみ）
                        if len(self.time_data) < 5:
                            print(f"Data received: Current={self.current_q}, Target={self.target_q}")
                        
                        return True
                        
                    except ValueError as e:
                        print(f"Value conversion error: {e}")
                
                # バッファが大きくなりすぎたらクリア
                if len(self.serial_buffer) > 1000:
                    self.serial_buffer = self.serial_buffer[-500:]
            
            return False
            
        except Exception as e:
            print(f"Serial read error: {e}")
            # エラー時はシリアル接続をリセット
            try:
                self.ser.close()
            except:
                pass
            self.ser = None
            return False
    
    def update_plots(self, frame):
        """プロット更新"""
        try:
            # データ読み取り
            if not self.read_serial_data():
                return
            
            # 時間データ更新
            current_time = time.time() - self.start_time
            self.time_data.append(current_time)
            
            # オイラー角計算
            current_euler = self.quaternion_to_euler(self.current_q)
            target_euler = self.quaternion_to_euler(self.target_q)
            error_euler = [t - c for c, t in zip(current_euler, target_euler)]
            
            # データ格納
            for i in range(3):
                self.current_euler_data[i].append(current_euler[i])
                self.target_euler_data[i].append(target_euler[i])
                self.error_data[i].append(error_euler[i])
            
            # データ長制限
            if len(self.time_data) > self.max_points:
                self.time_data = self.time_data[-self.max_points:]
                for i in range(3):
                    self.current_euler_data[i] = self.current_euler_data[i][-self.max_points:]
                    self.target_euler_data[i] = self.target_euler_data[i][-self.max_points:]
                    self.error_data[i] = self.error_data[i][-self.max_points:]
            
            # プロット更新
            if len(self.time_data) > 1:
                # オイラー角比較
                ax = self.axes[0, 2]
                ax.clear()
                ax.set_title('Euler Angles Comparison')
                ax.grid(True, alpha=0.3)
                
                labels = ['Roll', 'Pitch', 'Yaw']
                colors = ['red', 'green', 'blue']
                
                for i in range(3):
                    ax.plot(self.time_data, self.current_euler_data[i], 
                            color=colors[i], label=f'{labels[i]} Current', linewidth=1)
                    ax.plot(self.time_data, self.target_euler_data[i], 
                            color=colors[i], label=f'{labels[i]} Target', linewidth=1, linestyle='--')
                
                ax.legend(fontsize=8)
                ax.set_xlabel('Time [s]')
                ax.set_ylabel('Angle [deg]')
                
                # 誤差グラフ
                ax = self.axes[1, 0]
                ax.clear()
                ax.set_title('Attitude Error')
                ax.grid(True, alpha=0.3)
                
                for i, (label, color) in enumerate(zip(labels, colors)):
                    ax.plot(self.time_data, self.error_data[i], 
                            color=color, label=f'{label} Error', linewidth=1)
                
                ax.legend(fontsize=8)
                ax.set_xlabel('Time [s]')
                ax.set_ylabel('Error [deg]')
            
            # 情報パネル更新
            ax = self.axes[1, 2]
            ax.clear()
            ax.axis('off')
            ax.set_title('Real-time Info')
            
            info_text = f"""CURRENT (r,i,j,k):
{self.current_q[0]:6.3f}, {self.current_q[1]:6.3f}
{self.current_q[2]:6.3f}, {self.current_q[3]:6.3f}

TARGET (w,x,y,z):
{self.target_q[0]:6.3f}, {self.target_q[1]:6.3f}
{self.target_q[2]:6.3f}, {self.target_q[3]:6.3f}

EULER (Current):
Roll:  {current_euler[0]:6.1f}°
Pitch: {current_euler[1]:6.1f}°
Yaw:   {current_euler[2]:6.1f}°

EULER (Target):
Roll:  {target_euler[0]:6.1f}°
Pitch: {target_euler[1]:6.1f}°
Yaw:   {target_euler[2]:6.1f}°

Data Points: {len(self.time_data)}"""
            
            ax.text(0.05, 0.95, info_text, transform=ax.transAxes,
                   fontsize=9, verticalalignment='top', fontfamily='monospace')
            
        except Exception as e:
            print(f"Update error: {e}")
    
    def run(self):
        """ビジュアライザー実行"""
        if not self.initialized:
            print("Visualizer not properly initialized")
            return
        
        print("Starting animation...")
        try:
            ani = FuncAnimation(self.fig, self.update_plots, 
                              interval=100, cache_frame_data=False, blit=False)
            plt.show()
        except KeyboardInterrupt:
            print("Interrupted by user")
        except Exception as e:
            print(f"Animation error: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """クリーンアップ"""
        print("Cleaning up...")
        try:
            if self.ser:
                self.ser.close()
            plt.close('all')
        except Exception as e:
            print(f"Cleanup error: {e}")

if __name__ == "__main__":
    PORT = '/dev/ttyUSB0'
    
    print("Starting Drone Attitude Visualizer...")
    
    try:
        visualizer = QuaternionVisualizer(port=PORT, baudrate=115200)
        visualizer.run()
    except Exception as e:
        print(f"Fatal error: {e}")
    finally:
        print("Visualizer stopped.")