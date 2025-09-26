import socket
import json
import math
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np

# --- UDP受信設定 ---
UDP_IP = "0.0.0.0"
UDP_PORT = 8089
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# --- pygame & joystick 初期化 ---
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# --- 回転行列に変換 ---
def quaternion_to_matrix(q):
    w, x, y, z = q
    # 正規化
    n = math.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/n, x/n, y/n, z/n

    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w), 0],
        [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w), 0],
        [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y), 0],
        [0,                   0,                 0,             1]
    ], dtype=np.float32)

# --- 立方体の描画 ---
def draw_cube():
    glBegin(GL_QUADS)
    glColor3f(1,0,0)  # 前面（赤）
    glVertex3f(-1, -1, 1)
    glVertex3f(1, -1, 1)
    glVertex3f(1, 1, 1)
    glVertex3f(-1, 1, 1)

    glColor3f(0,1,0)  # 背面（緑）
    glVertex3f(-1, -1, -1)
    glVertex3f(-1, 1, -1)
    glVertex3f(1, 1, -1)
    glVertex3f(1, -1, -1)

    glColor3f(0,0,1)  # 左面（青）
    glVertex3f(-1, -1, -1)
    glVertex3f(-1, -1, 1)
    glVertex3f(-1, 1, 1)
    glVertex3f(-1, 1, -1)

    glColor3f(1,1,0)  # 右面（黄）
    glVertex3f(1, -1, -1)
    glVertex3f(1, 1, -1)
    glVertex3f(1, 1, 1)
    glVertex3f(1, -1, 1)

    glColor3f(0,1,1)  # 上面（水色）
    glVertex3f(-1, 1, -1)
    glVertex3f(-1, 1, 1)
    glVertex3f(1, 1, 1)
    glVertex3f(1, 1, -1)

    glColor3f(1,0,1)  # 底面（紫）
    glVertex3f(-1, -1, -1)
    glVertex3f(1, -1, -1)
    glVertex3f(1, -1, 1)
    glVertex3f(-1, -1, 1)
    glEnd()

# --- クォータニオン操作 ---
def quaternion_conjugate(q):
    return [q[0], -q[1], -q[2], -q[3]]

def quaternion_multiply(q1, q2):
    w1,x1,y1,z1 = q1
    w2,x2,y2,z2 = q2
    return [
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ]

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
        cr * cp * sy - sr * sp * cy
    ]

def calc_target(up, down, left, right,Rx, Ry, yaw_input, offset_roll, offset_pitch, offset_yaw, yaw):
    #roll_euler = up * -40 + down * +40 - offset_roll
    #pitch_euler = left * -40 + right * 40 - offset_pitch
    roll_euler = -Ry * -40 - offset_roll
    pitch_euler = Rx * 40 - offset_pitch
    yaw += yaw_input
    yaw_euler = yaw + offset_yaw

    q_roll = euler_to_quaternion(roll_euler, 0, 0)
    q_pitch = euler_to_quaternion(0, pitch_euler, 0)
    q_yaw = euler_to_quaternion(0, 0, yaw_euler)

    q_target = quaternion_multiply(q_yaw, quaternion_multiply(q_pitch, q_roll))
    return q_target, yaw

# --- OpenGL初期設定 ---
def init_gl(width, height):
    glViewport(0, 0, width, height)
    glEnable(GL_DEPTH_TEST)
    glClearColor(0.1, 0.1, 0.1, 1)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, width / height, 0.1, 50.0)
    glMatrixMode(GL_MODELVIEW)

# --- 座標変換行列（センサ座標→OpenGL座標） ---
axis_transform = np.array([
    [1, 0, 0, 0],
    [0, 0, 1, 0],
    [0, 1, 0, 0],
    [0, 0, 0, 1]
], dtype=np.float32)

# --- メイン ---
def main():
    width, height = 800, 400
    screen = pygame.display.set_mode((width, height), DOUBLEBUF | OPENGL)
    pygame.display.set_caption("Sensor & Target Quaternion Visualization")
    init_gl(width, height)

    offset_roll = 0
    offset_pitch = 0
    offset_yaw = 0
    yaw = 0
    quat_buffer = []
    calibration_done = False
    is_motor_running = False
    prev_circle = 0

    clock = pygame.time.Clock()

    sensor_q = [1, 0, 0, 0]
    target_q = [1, 0, 0, 0]

    while True:
        # --- pygameイベント処理 ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return

        pygame.event.pump()
        # コントローラー割当
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

        # キャリブレーション処理
        if Square and not calibration_done and len(quat_buffer) >= 50:
            rolls = []
            pitches = []
            yaws = []
            for q in quat_buffer:
                w,x,y,z = q
                roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
                pitch = math.asin(2*(w*y - z*x))
                yaw_angle = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
                rolls.append(math.degrees(roll))
                pitches.append(math.degrees(pitch))
                yaws.append(math.degrees(yaw_angle))
            offset_roll = sum(rolls)/len(rolls)
            offset_pitch = sum(pitches)/len(pitches)
            offset_yaw = sum(yaws)/len(yaws)
            calibration_done = True
            quat_buffer.clear()
            print(f"Calibration done: roll={offset_roll:.2f}, pitch={offset_pitch:.2f}, yaw={offset_yaw:.2f}")

        # モーターON/OFFトグル
        if Circle and not prev_circle:
            is_motor_running = not is_motor_running
        prev_circle = Circle

        # UDP受信
        try:
            data, addr = sock.recvfrom(1024)
            json_data = json.loads(data.decode())
            q = json_data["Quaternion"]
            sensor_q = [q["r"], q["i"], q["j"], q["k"]]
            if not calibration_done:
                quat_buffer.append(sensor_q)
                if len(quat_buffer) > 200:
                    quat_buffer.pop(0)
        except socket.timeout:
            pass
        except Exception as e:
            print("UDP receive error:", e)

        yaw_input = 0
        if R1:
            yaw_input -= 1.0  # R1ボタンでyawを増やす
        if L1:
            yaw_input += 1.0  # L1ボタンでyawを減らす
        
        # もしDパッドの左右が押されていたら上書きしたいなら以下を調整
        if left:
            yaw_input = -abs(yaw_input)
        elif right:
            yaw_input = abs(yaw_input)

                
        # ターゲット姿勢計算
        target_q, yaw = calc_target(up, down, left, right, Rx, Ry, yaw_input, offset_roll, offset_pitch, offset_yaw, yaw)


        # 描画処理
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        sensor_matrix = quaternion_to_matrix(sensor_q)
        sensor_matrix = axis_transform @ sensor_matrix

        target_matrix = quaternion_to_matrix(target_q)
        target_matrix = axis_transform @ target_matrix

        # センサ姿勢（左）
        glLoadIdentity()
        glTranslatef(-2.5, 0, -7)
        glMultMatrixf(sensor_matrix.T)
        draw_cube()

        # 目標姿勢（右）
        glLoadIdentity()
        glTranslatef(2.5, 0, -7)
        glMultMatrixf(target_matrix.T)
        draw_cube()

        pygame.display.flip()
        clock.tick(60)

if __name__ == "__main__":
    main()
