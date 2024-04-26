#!/usr/bin/env python

import rospy
from classification.msg import Block, ImageInfo

import cv2
import torch
from numpy import random
import numpy as np
from pymycobot.mycobot import MyCobot
import time
import threading

mc = MyCobot("/dev/ttyACM0", 115200)

class MoveCobot:
    def __init__(self):

        self.speed = 60

        self.init_done = False
        self.init_coords = self.init_mycobot()

        self.coords = {'zero': [0, 0, 0, 0, 0, 0],
                       'init': [0, -120, 130, -90, 90, 0]}

        self.blocks = list()

        self.width, self.w_min, self.w_max = None, None, None
        self.height, self.h_min, self.h_max = None, None, None
        self.step = None

        self.block_sub = rospy.Subscriber('/block/color_xy', Block, self.block_callback)
        self.img_info_sub = rospy.Subscriber('/image/image_wh_s', ImageInfo, self.img_info_callback)

    def block_callback(self, block_data):
        self.blocks = list()
        if block_data.r: self.blocks.append(('red', block_data.rx, block_data.ry))
        if block_data.y: self.blocks.append(('yellow', block_data.yx, block_data.yy))
        if block_data.b: self.blocks.append(('blue', block_data.bx, block_data.by))
        if block_data.p: self.blocks.append(('purple', block_data.px, block_data.py))

    def img_info_callback(self, data):
        self.width = data.width
        self.w_min = self.width*0.48
        self.w_max = self.width*0.52
        
        self.height = data.height
        self.h_min = self.height*0.90 
        self.h_max = self.height*0.95

        self.step = data.step

    def init_mycobot(self):
        mc.sync_send_angles(self.coords['zero'], self.speed)
        mc.sync_send_angles(self.coords['init'], self.speed)

        mc.set_gripper_calibration()
        mc.set_gripper_mode(0)
        mc.init_eletric_gripper()
        time.sleep(1)

        print('initialize..')
        self.init_done = True
        
        return list(mc.get_coords())
        
    def move_to_x_center(self, x_center):

        if x_center < self.w_min:
            self.init_coords[1] += self.step
        elif x_center > self.w_max:
            self.init_coords[1] -= self.step
        else:
            print('x done')
            return True

        mc.send_coords(self.init_coords, self.speed, 1)

        return False

    def move_to_y_center(self, y_center):
        if y_center < self.h_min:
            self.init_coords[0] += self.step
        elif y_center > self.h_max:
            self.init_coords[0] -= self.step
        else:
            print('y done')
            return True

        mc.send_coords(self.init_coords, self.speed, 1)

        return False
    

    def grab_block(self, color, x_center, y_center):
        print('start grab')
        self.init_coords[0] += 5
        mc.send_coords(self.init_coords, self.speed, 1)
        time.sleep(2)

        mc.set_eletric_gripper(0)
        mc.set_gripper_value(0, self.speed, 1)
        time.sleep(2)

        if color in ["red", "blue"]:
            mc.sync_send_coords([207, -234, 304, -164, -8, -123], self.speed, 1, timeout=3)
        else:
            mc.sync_send_coords([251, 197, 243, 177, -5, -35], self.speed, 1, timeout=3)

        mc.set_eletric_gripper(0)
        mc.set_gripper_value(100, self.speed, 1)
        time.sleep(2)

    def main_loop(self):

        while True:
            if self.blocks:
                color, x, y = self.blocks.pop()

                print(color, x, y)
                if self.move_to_x_center(x):
                    if self.move_to_y_center(y):
                        self.grab_block(color, x, y)

                        mc.sync_send_angles(self.coords['init'], self.speed)
                        self.init_coords = list(mc.get_coords())

            self.blocks = list()
                      
if __name__ == "__main__":
    rospy.init_node('block_subscriber', anonymous=False)

    try:
        m = MoveCobot()
        if m.init_done:
            t1 = threading.Thread(target=m.main_loop, daemon=True)
            t1.start()

        while not rospy.is_shutdown():
            rospy.spin()

        mc.set_eletric_gripper(0)
        mc.set_gripper_value(100, 20, 1)
        time.sleep(2)

    except rospy.ROSInterruptException:
        pass
