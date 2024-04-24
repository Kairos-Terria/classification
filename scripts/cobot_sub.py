import rospy
from your_custom_msg_package.msg import Block  # 'your_custom_msg_package'와 'Block'을 적절히 바꿔야 합니다.
import cv2
import torch
from numpy import random
import numpy as np
from pymycobot.mycobot import MyCobot
import time


def callback(block_data):
    global block
    center_x, center_y = 320, 480
    nearest_block = None
    min_distance = float('inf')

    # 각 블록의 존재 여부 및 위치 확인
    blocks = [
        ('red', block_data.rx, block_data.ry) if block_data.r else None,
        ('yellow', block_data.yx, block_data.yy) if block_data.y else None,
        ('blue', block_data.bx, block_data.by) if block_data.b else None,
        ('purple', block_data.px, block_data.py) if block_data.p else None
    ]

    # (320, 480)에 가장 가까운 블록 찾기
    for block in blocks:
        if block is not None:
            color, x, y = block
            distance = ((x - center_x) ** 2 + (y - center_y) ** 2) ** 0.5
            if distance < min_distance:
                min_distance = distance
                nearest_block = (x, y, color)

    # 가장 가까운 블록 정보를 전역 변수에 저장
    block = nearest_block

def main_loop(mc):
    rospy.init_node('block_subscriber', anonymous=True)
    rospy.Subscriber('block_topic', Block, callback)  # 'block_topic'은 실제 토픽 이름으로 바꿔야 합니다.
    rospy.wait_for_message('block_topic', Block)  # 최소한 하나의 메시지를 기다립니다.

    width, height = 640, 480
    mc.sync_send_coords([329, -55, 227, -178, -4, -83], 20, 1)
    time.sleep(2)
    init_coords = list(mc.get_coords())
    print(mc.get_coords())

    mc.set_gripper_calibration()
    mc.set_gripper_mode(0)
    mc.init_eletric_gripper()

    while not rospy.is_shutdown():
        if not block or None in block:
            continue  # block 정보가 없거나 완전하지 않은 경우 루프 계속

        x_center, y_center = block[0], block[1]
        print('Block detected:', block[0:2])

        if x_center < width * 0.48:
            init_coords[1] += 1  # 왼쪽 이동
        elif x_center > width * 0.52:
            init_coords[1] -= 1  # 오른쪽 이동
        elif width * 0.48 <= x_center <= width * 0.52:
            if y_center < height * 0.90:
                init_coords[0] += 1  # 왼쪽 이동
            elif y_center > height * 0.95:
                init_coords[0] -= 1  # 오른쪽 이동

        mc.send_coord(1, init_coords[0], 20)
        mc.send_coord(2, init_coords[1], 20)
        #mc.send_coords(tuple(init_coords), 20, 1)
        #print(mc.get_coords())
        time.sleep(0.1)

        if width * 0.48 <= x_center <= width * 0.52 and height * 0.90 <= y_center <= height * 0.95:
            init_coords[0] = 5
            mc.send_coord(1, init_coords[0], 20)
        
            init_coords[2] = 300
            mc.send_coord(3, init_coords[2], 20)
            time.sleep(2)
            init_coords[3] = -175
            mc.send_coord(4, init_coords[3], 20)

            mc.set_eletric_gripper(1)
            mc.set_gripper_value(0, 20, 1)
            time.sleep(2)
            if block[2] == "red" or block[2] == "blue":
                mc.sync_send_coords([207, -234, 304, -164, -8, -123], 20, 1)  # Assuming this resets the position
                mc.set_eletric_gripper(0)
                mc.set_gripper_value(100, 20, 1)
            else:
                mc.sync_send_coords([251, 197, 243, 177, -5, -35], 20, 1)  # Assuming this resets the position
                mc.set_eletric_gripper(0)
                mc.set_gripper_value(100, 20, 1)

            mc.sync_send_coords([239, -60, 331, -176, 0, -83], 20, 1)

            print("Task completed with:", block[2])
            break

        # 이하 로직은 초기 제공된 코드와 같습니다.
        # 여기에 조건문과 로직을 추가하여 로봇팔 조정
        # ...

if __name__ == "__main__":
    mc = MyCobot("/dev/ttyACM0", 115200)
    while True:
        main_loop(mc)
