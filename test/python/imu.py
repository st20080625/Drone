import socket
import json
import math

# クォータニオン → オイラー角変換関数
def quaternion_to_euler(w, x, y, z):
    # Roll (x軸回転)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y軸回転)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # ±90°
    else:
        pitch = math.asin(sinp)

    # Yaw (z軸回転)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw  # 単位: ラジアン

# UDP設定
UDP_IP = "0.0.0.0"
UDP_PORT = 8089
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"UDP受信中... ポート: {UDP_PORT}（Ctrl+Cで終了）\n")

try:
    while True:
        data, addr = sock.recvfrom(1024)
        decoded = data.decode('utf-8').strip()

        try:
            # JSONとして読み込み
            parsed = json.loads(decoded)

            q = parsed["Quaternion"]
            w, x, y, z = q["r"], q["i"], q["j"], q["k"]

            roll, pitch, yaw = quaternion_to_euler(w, x, y, z)

            # ラジアン → 度に変換して出力
            roll_deg  = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg   = math.degrees(yaw)

            print(f"roll: {roll_deg:+.2f}°, pitch: {pitch_deg:+.2f}°, yaw: {yaw_deg:+.2f}°")

        except Exception as e:
            print(f"解析エラー: {e}\n受信データ: {decoded}")

except KeyboardInterrupt:
    print("\n終了しました。")
