import serial
import time

def simple_console_reader():
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        print("Connected to /dev/ttyUSB0")
        ser.reset_input_buffer()  # バッファクリア
    except Exception as e:
        print(f"Serial connection failed: {e}")
        return
    
    print("Reading data from ESP32...")
    print("Press Ctrl+C to stop")
    print("-" * 50)
    
    try:
        while True:
            if ser.in_waiting > 0:
                # 生データを読み取り
                raw_line = ser.readline()
                print(f"Raw bytes: {raw_line}")
                
                # デコード
                try:
                    line = raw_line.decode('utf-8').strip()
                    print(f"Decoded: '{line}'")
                    
                    if line and ',' in line:
                        data = line.split(',')
                        print(f"Split data ({len(data)} parts): {data}")
                        
                        if len(data) >= 8:
                            try:
                                # 数値変換
                                current_q = [float(data[0]), float(data[1]), float(data[2]), float(data[3])]
                                target_q = [float(data[4]), float(data[5]), float(data[6]), float(data[7])]
                                
                                print(f"Current quaternion: w={current_q[0]:.3f}, x={current_q[1]:.3f}, y={current_q[2]:.3f}, z={current_q[3]:.3f}")
                                print(f"Target quaternion:  w={target_q[0]:.3f}, x={target_q[1]:.3f}, y={target_q[2]:.3f}, z={target_q[3]:.3f}")
                                
                            except ValueError as e:
                                print(f"Value conversion error: {e}")
                        else:
                            print(f"Insufficient data: expected 8, got {len(data)}")
                    else:
                        print("No comma found in data or empty line")
                        
                except UnicodeDecodeError as e:
                    print(f"Decode error: {e}")
                
                print("-" * 50)
            else:
                # データがない場合は短い待機
                time.sleep(0.05)
                
    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        try:
            ser.close()
            print("Serial connection closed")
        except:
            pass

if __name__ == "__main__":
    simple_console_reader()