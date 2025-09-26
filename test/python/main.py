import socket
import json
import math
import pygame
import time
import numpy as np

# --- UDP受信設定 ---
UDP_IP = "0.0.0.0"  # 自分のIP
UDP_PORT = 8089

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.01)  # ノンブロッキング

# --- コントローラ初期化（pygame） ---
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# --- 状態変数 ---
offset_roll = 0
offset_pitch = 0
offset_yaw = 0
yaw = 0

kq = 4
kw = 2
kq_yaw = 2
kw_yaw = 1

offset_motor_speed = 30
calibration_done = False
is_motor_running = False

prev_circle = 0

# --- モーター出力 ---
motor_speed = 0
motor_values = [0, 0, 0, 0]

# --- クォータニオン操作用 ---
def quaternion_conjugate(q):
    return [q[0], -q[1], -q[2], -q[3]]

def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return [
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ]

def quaternion_to_euler(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - z * x))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll * 180 / math.pi, pitch * 180 / math.pi, yaw * 180 / math.pi

# --- クォータニオンから目標姿勢計算 ---
def calc_target(up, down, left, right, yaw_input):
    global yaw, offset_roll, offset_pitch, offset_yaw

    pitch_euler = up * 40 + down * -40 - offset_pitch
    roll_euler = left * -40 + right * 40 - offset_roll
    yaw += yaw_input
    yaw_euler = yaw + offset_yaw

    def euler_to_quaternion(roll, pitch, yaw):
        cr = math.cos(math.radians(roll) / 2)
        sr = math.sin(math.radians(roll) / 2)
        cp = math.cos(math.radians(pitch) / 2)
        sp = math.sin(math.radians(pitch) / 2)
        cy = math.cos(math.radians(yaw) / 2)
        sy = math.sin(math.radians(yaw) / 2)

        return [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]

    q_roll = euler_to_quaternion(roll_euler, 0, 0)
    q_pitch = euler_to_quaternion(0, pitch_euler, 0)
    q_yaw = euler_to_quaternion(0, 0, yaw_euler)

    q_target = quaternion_multiply(q_yaw, quaternion_multiply(q_pitch, q_roll))
    return q_target

# --- センサーのキャリブレーション ---
def calibrate(quat_list):
    rolls, pitches, yaws = [], [], []
    for q in quat_list:
        r, p, y = quaternion_to_euler(q)
        rolls.append(r)
        pitches.append(p)
        yaws.append(y)
    return np.mean(rolls), np.mean(pitches), np.mean(yaws)

# --- メインループ ---
print("開始")
last_time = time.time()
quat_buffer = []

while True:
    now = time.time()
    dt = now - last_time
    last_time = now

    # --- ゲームパッド入力処理 ---
    pygame.event.pump()
    Lx = joystick.get_axis(0)
    Ly = joystick.get_axis(1)
    Rx = joystick.get_axis(3)
    Ry = joystick.get_axis(4)

    Square = joystick.get_button(3)
    Circle = joystick.get_button(1)
    R1 = joystick.get_button(5)
    L1 = joystick.get_button(4)
    up = joystick.get_hat(0)[1] == 1
    down = joystick.get_hat(0)[1] == -1
    left = joystick.get_hat(0)[0] == -1
    right = joystick.get_hat(0)[0] == 1

    # --- キャリブレーショントリガー ---
    if Square and not calibration_done and len(quat_buffer) >= 50:
        offset_roll, offset_pitch, offset_yaw = calibrate(quat_buffer)
        calibration_done = True
        is_motor_running = False
        print(f"Calibration complete: roll={offset_roll}, pitch={offset_pitch}, yaw={offset_yaw}")
        quat_buffer = []

    # --- モータON/OFF トグル ---
    if Circle and not prev_circle:
        is_motor_running = not is_motor_running
    prev_circle = Circle

    try:
        data, addr = sock.recvfrom(1024)
        json_data = json.loads(data)

        r = json_data["Quaternion"]["r"]
        i = json_data["Quaternion"]["i"]
        j = json_data["Quaternion"]["j"]
        k = json_data["Quaternion"]["k"]
        wx = json_data["Gyroscope"]["wx"]
        wy = json_data["Gyroscope"]["wy"]
        wz = json_data["Gyroscope"]["wz"]

        q = [r, i, j, k]
        if not calibration_done:
            quat_buffer.append(q)
            continue

        q_target = calc_target(up, down, left, right, (L1 - R1) * 2)
        q_error = quaternion_multiply(q_target, quaternion_conjugate(q))

        w = max(min(q_error[0], 1.0), -1.0)
        angle = 2 * math.acos(w)
        s = math.sqrt(1 - w * w) if 1 - w * w > 0 else 1e-6
        axis = [q_error[1]/s, q_error[2]/s, q_error[3]/s] if s > 1e-6 else [0, 0, 0]

        Mx = -kq * axis[0] * angle - kw * wx
        My = -kq * axis[1] * angle - kw * wy
        Mz = kq_yaw * axis[2] * angle - kw_yaw * wz

        mapped_speed = (-Ly * 50) + 20
        motor_speed = offset_motor_speed + int(mapped_speed)

        motor_values[0] = motor_speed + Mx + My + Mz
        motor_values[1] = motor_speed - Mx + My - Mz
        motor_values[2] = motor_speed - Mx - My + Mz
        motor_values[3] = motor_speed + Mx - My - Mz

        motor_values = [max(0, min(100, int(m))) for m in motor_values]

        if is_motor_running:
            print(f"Motors: {motor_values}")
        else:
            print("Motors OFF")

    except socket.timeout:
        continue

