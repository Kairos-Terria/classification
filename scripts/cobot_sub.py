#!/usr/bin/env python

import rospy
from classification.msg import Block
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

        self.init_done = False
        self.init_coords = self.init_mycobot()

        self.blocks = list()

        self.width = 2592
        self.w_min = self.width*0.48
        self.w_max = self.width*0.52
        
        self.height = 1944
        self.h_min = self.height*0.90 
        self.h_max = self.height*0.95

        self.block_sub = rospy.Subscriber('/block/color_xy', Block, self.block_callback)

    def block_callback(self, block_data):
        self.blocks = list()
        if block_data.r: self.blocks.append(('red', block_data.rx, block_data.ry))
        if block_data.y: self.blocks.append(('yellow', block_data.yx, block_data.yy))
        if block_data.b: self.blocks.append(('blue', block_data.bx, block_data.by))
        if block_data.p: self.blocks.append(('purple', block_data.px, block_data.py))


    def init_mycobot(self):
        mc.sync_send_angles([0, 0, 0, 0, 0, 0], 20)
        mc.sync_send_angles([0, -120, 130, -90, 90, 0], 20)

        mc.set_gripper_calibration()
        mc.set_gripper_mode(0)
        mc.init_eletric_gripper()
        time.sleep(1)

        print('initialize..')
        self.init_done = True
        
        return list(mc.get_coords())
        
    def move_to_x_center(self, x_center):
        if x_center < self.w_min:
            self.init_coords[1] += 5
        elif x_center > self.w_max:
            self.init_coords[1] -= 5
        else:
            print('x done')
            return True

        mc.send_coords(self.init_coords, 20, 1)

        return False

    def move_to_y_center(self, y_center):
        if y_center < self.h_min:
            self.init_coords[0] += 5
        elif y_center > self.h_max:
            self.init_coords[0] -= 5
        else:
            print('y done')
            return True

        mc.send_coords(self.init_coords, 20, 1)

        return False
    

    def grab_block(self, x_center, y_center):
        print('start grab')
        self.init_coords[0] += 5
        mc.send_coords(self.init_coords, 20, 1)
        time.sleep(2)
    
        #self.init_coords[2] = 170
        #mc.send_coords(self.init_coords, 20, 1)
        #time.sleep(2)
        #self.init_coords[3] = -175
        #mc.send_coords(self.init_coords, 20, 1)

        mc.set_eletric_gripper(0)
        mc.set_gripper_value(0,20,1)
        time.sleep(2)

        mc.set_eletric_gripper(0)
        mc.set_gripper_value(100,20,1)
        time.sleep(2)

        """
        if block[2] == "red" or block[2] == "blue":
            mc.sync_send_coords([207, -234, 304, -164, -8, -123], 20, 1)  # Assuming this resets the position
            mc.set_eletric_gripper(0)
            mc.set_gripper_value(100, 20, 1)
        else:
            mc.sync_send_coords([251, 197, 243, 177, -5, -35], 20, 1)  # Assuming this resets the position
            mc.set_eletric_gripper(0)
            mc.set_gripper_value(100, 20, 1)

        """
    def main_loop(self):

        while True:
            if self.blocks:
                color, x, y = self.blocks.pop()

                print(color, x, y)
                if self.move_to_x_center(x):
                    if self.move_to_y_center(y):
                        self.grab_block(x, y)
                        break
            
            self.blocks = list()

        print('d')
                      

        """
        w_min, w_max = self.width*0.48, self.width*0.52
        h_min, h_max = self.height*0.90, self.height*0.95

        x_center, y_center = block[0], block[1]
        print('Block detected:', block[0:2])

        while self.move_to_x_center(x_center):
            mc.send_coord(1, self.init_coords[0], 20)

        while self.move_to_y_center(y_center):
            mc.send_coord(2, self.init_coords[1], 20)

        grab_block(self, x_center, y_center)

        mc.sync_send_coords([239, -60, 331, -176, 0, -83], 20, 1)

        print("Task completed with:", block[2])
        """

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
        mc.set_gripper_value(100,20,1)
        time.sleep(2)

    except rospy.ROSInterruptException:
        pass
