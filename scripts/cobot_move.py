import cv2
import torch
from numpy import random
import numpy as np
from pymycobot.mycobot import MyCobot
import time

def main_loop(mc):
    width, height = 640, 480
    mc.sync_send_coords([329, -55, 227, -178, -4, -83], 20, 1)
    time.sleep(2)
    init_coords = list(mc.get_coords())
    print(mc.get_coords())

    mc.set_gripper_calibration()
    mc.set_gripper_mode(0)
    mc.init_eletric_gripper()

    while True:
        block = []
        if not block:
            continue  # You may want to handle cases when block is empty differently.

        x_center, y_center = block[0], block[1]

        if x_center < width * 0.48:
            init_coords[1] += 1  # 왼쪽 이동
        elif x_center > width * 0.52:
            init_coords[1] -= 1  # 오른쪽 이동
        elif width * 0.48 <= x_center <= width * 0.52:
            if y_center < height * 0.96:
                init_coords[0] += 1  # 앞으로 이동
            elif y_center > height * 0.99:
                init_coords[0] -= 1  # 뒤로 이동

        mc.send_coords(tuple(init_coords), 20, 1)
        print(mc.get_coords())
        time.sleep(0.1)

        if width * 0.48 <= x_center <= width * 0.52 and height * 0.96 <= y_center <= height * 0.99:
            print("check1: ", mc.get_coords())
            init_coords[2] = 170
            mc.send_coord(2, init_coords[2], 20)
            time.sleep(2)
            init_coords[3] = -175
            mc.send_coord(3, init_coords[3], 20)

            print("check2: ", mc.get_coords())
            time.sleep(1)

            mc.set_eletric_gripper(1)
            mc.set_gripper_value(0, 20, 1)

            time.sleep(2)
            if block[2] == "red" or block[2] == "blue":
                mc.sync_send_coords()  # Assuming this resets the position
                mc.set_eletric_gripper(0)
                mc.set_gripper_value(100, 20, 1)
            else:
                mc.sync_send_coords()  # Assuming this resets the position
                mc.set_eletric_gripper(0)
                mc.set_gripper_value(100, 20, 1)
            
            return

 

if __name__ == "__main__":
    mc = MyCobot("/dev/ttyACM0", 115200)
    while True:
        main_loop(mc)
