import pygame
import time

# 初期化
pygame.init()
pygame.joystick.init()

# ジョイスティック（コントローラ）が接続されているか確認
if pygame.joystick.get_count() == 0:
    print("コントローラが接続されていません。")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"コントローラ名: {joystick.get_name()}")
print(f"ボタン数: {joystick.get_numbuttons()}, 軸数: {joystick.get_numaxes()}, ハット数: {joystick.get_numhats()}")

print("\nCtrl+Cで終了\n")

try:
    while True:
        pygame.event.pump()  # 入力更新

        # ボタンの状態
        print("[Buttons]", end=' ')
        for i in range(joystick.get_numbuttons()):
            val = joystick.get_button(i)
            print(f"{i}:{val}", end=' ')
        
        # 軸（スティック）の状態
        print("\n[Axes]   ", end=' ')
        for i in range(joystick.get_numaxes()):
            val = joystick.get_axis(i)
            print(f"{i}:{val:.2f}", end=' ')

        # ハット（十字キー）
        print("\n[Hats]   ", end=' ')
        for i in range(joystick.get_numhats()):
            val = joystick.get_hat(i)
            print(f"{i}:{val}", end=' ')

        print("\n" + "-"*50)
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n終了しました。")
    pygame.quit()

