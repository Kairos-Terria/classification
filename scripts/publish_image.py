#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from classification.msg import ImageInfo
from cv_bridge import CvBridge

def publish_image():
    image_pub = rospy.Publisher('/image/image_raw', Image, queue_size=10)
    image_info_pub = rospy.Publisher('/image/image_wh_s', ImageInfo, queue_size=10)

    cap = cv2.VideoCapture("dev/video2")
    print("ok")
    ii = ImageInfo()
    ii.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ii.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    ii.step = 1 if ii.width < 1000 else 10

    while not rospy.is_shutdown() and cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        image_pub.publish(CvBridge().cv2_to_imgmsg(frame, "bgr8"))
        image_info_pub.publish(ii)
            
if __name__=='__main__':
    try:
        rospy.init_node('publish_image', anonymous=False)
        publish_image()
    except rospy.ROSInterruptException:
        pass

