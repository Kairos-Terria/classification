#!/usr/bin/env python
import cv2
import numpy as np

import rospy
from classification.msg import Block

class ColorExtractor:
    def __init__(self):
        self.cap = cv2.VideoCapture(2)

        self.threshold = {'red': [np.array([170, 100, 100]), np.array([180, 255, 255])], #red
                          'yellow': [np.array([20, 100, 100]), np.array([30, 255, 255])], #yellow
                          'blue': [np.array([100, 100, 100]), np.array([130, 255, 255])], #blue
                          'purple': [np.array([125, 50, 50]), np.array([150, 255, 255])]} #purple

        self.threshold_len = len(self.threshold)

        self.draw = True
        self.color = [[np.random.randint(0, 255) for _ in range(3)] for _ in self.threshold]

        self.width = 640
        self.height = 480

        self.pub = rospy.Publisher('/color_xy', Block, queue_size=10)

    def get_color(self):

        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                break

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            mask = list()
            for k, v in self.threshold.items():
                mask.append(cv2.inRange(hsv, v[0], v[1]))

            contours = list()
            for i in range(self.threshold_len):
                contours.append(cv2.findContours(mask[i], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))

            msg = Block()
            temp = list()
            for i, (contour, _) in enumerate(contours):
                if len(contour) > 0:
                    max_contour = max(contour, key=cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(max_contour)
                    cx, cy = int(x+w/2), int(y+h/2)
                    if self.draw:
                        cv2.rectangle(frame, (x, y), (x + w, y + h), self.color[i], 2)

                    temp.append([True, cx, cy])
                else:
                    temp.append([False, 0, 0])

            msg.r, msg.rx, msg.ry = temp[0][0], temp[0][1], temp[0][2]
            msg.y, msg.yx, msg.yy = temp[1][0], temp[1][1], temp[1][2]
            msg.b, msg.bx, msg.by = temp[2][0], temp[2][1], temp[2][2]
            msg.p, msg.px, msg.py = temp[3][0], temp[3][1], temp[3][2]

            self.pub.publish(msg)

            cv2.namedWindow("window", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("window", width=self.width, height=self.height)
            cv2.imshow("window", frame)
            cv2.waitKey(1)

if __name__=='__main__':
    c = ColorExtractor()
    rospy.init_node('publish')

    while not rospy.is_shutdown():
        c.get_color()
