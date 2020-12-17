#!/usr/bin/python
import rospy
import numpy as np
import cv2  # OpenCV module
import time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError



if __name__ == '__main__':
    rospy.init_node('cam_read', anonymous=True)
    pub = rospy.Publisher('img_input', Image,queue_size=10)
    rate = rospy.Rate(10) # 10hz

    bridge = CvBridge()
    cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        # Capture frame-by-frame
        ret, frame = cap.read()
        image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        pub.publish(image_message)
        rospy.loginfo('OK')
        rate.sleep()