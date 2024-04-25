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

class MoveCobot:
    def __init__(self):

        mc.sync_send_coords([329, -55, 227, -178, -4, -83], 20, 1)
        mc.set_gripper_calibration()
        mc.set_gripper_mode(0)
        mc.init_eletric_gripper()

        self.init_coords = list(mc.get_coords())

        self.width = 640
        self.w_min = self.width*0.48
        
        self.height = 480

    def move_to_x_center(self, x_center):
        if x_center < self.w_min:
            self.init_coords[1] += 1
        elif x_center > self.w_max:
            self.init_coords[1] -= 1
        else:
            return True

        return False

    def move_to_y_center(self, y_center):
        if y_center < self.h_min:
            self.init_coords[0] += 1
        elif y_center > self.h_max:
            self.init_coords[0] -= 1
        else:
            return True

        return False
    
    def z_calbration(self):
        self.init_coords[2] = list(mc.get_coords())[2]



    def grab_block(self, x_center, y_center):
        while self.move_to_x_center(x_center):
            mc.send_coord(1, self.init_coords[0], 20)

        while self.move_to_y_center(y_center):
            mc.send_coord(2, self.init_coords[1], 20)

        if w_min <= x_center <= w_max and h_min <= y_center <= h_max:
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


    def go_starting_point(self):
        mc.sync_send_coords([329, -55, 227, -178, -4, -83], 20, 1)
        self.init_coords = list(mc.get_coords())
        


    def main_loop(self, mc):
        rospy.Subscriber('block_topic', Block, callback)  # 'block_topic'은 실제 토픽 이름으로 바꿔야 합니다.
        rospy.wait_for_message('block_topic', Block)  # 최소한 하나의 메시지를 기다립니다.

        w_min, w_max = self.width*0.48, self.width*0.52
        h_min, h_max = self.height*0.90, self.height*0.95

        while not rospy.is_shutdown():
            if not block or None in block:
                continue  # block 정보가 없거나 완전하지 않은 경우 루프 계속

            x_center, y_center = block[0], block[1]
            print('Block detected:', block[0:2])

            while not self.move_to_x_center(x_center):
                mc.send_coord(1, self.init_coords[0], 20)
                
            while not self.move_to_y_center(y_center):
                mc.send_coord(2, self.init_coords[1], 20)

            time.sleep(0.1)

            if w_min <= x_center <= w_max and h_min <= y_center <= h_max:
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


if __name__ == "__main__":
    rospy.init_node('block_subscriber', anonymous=True)

    mc = MyCobot("/dev/ttyACM0", 115200)
    m = MoveCobot()

    while True:
        m.main_loop(mc)
