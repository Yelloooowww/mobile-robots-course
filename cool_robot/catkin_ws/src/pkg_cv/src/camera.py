#!/usr/bin/python
import rospy
import numpy as np
import cv2  # OpenCV module
import time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String,Int32,Int32MultiArray




if __name__ == '__main__':
    rospy.init_node('cam_read', anonymous=True)
    pub_img = rospy.Publisher('img_detection', Image,queue_size=10)
    pub_info=rospy.Publisher('detection_info',Int32MultiArray,queue_size = 10)
    rate = rospy.Rate(10) # 10hz

    bridge = CvBridge()
    cap = cv2.VideoCapture(0)

    detection_time = 0
    detection_x, detection_y = 0, 0

    while not rospy.is_shutdown():
        # Capture frame-by-frame
        ret, imageFrame = cap.read()
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
        # define mask
        green_lower = np.array([40, 52, 72], np.uint8)
        green_upper = np.array([70,255,255], np.uint8)
        green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
        kernal = np.ones((5, 5), "uint8")

        # For green color
        green_mask = cv2.dilate(green_mask, kernal)
        res_green = cv2.bitwise_and(imageFrame, imageFrame,mask = green_mask)
        _, contours, hierarchy = cv2.findContours(green_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)


        max_area, max_x, max_y, max_w, max_h = 300, None, None, None, None
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if area >= max_area: max_x,max_y,max_w,max_h = cv2.boundingRect(contour)
            if(area > 300):
                x, y, w, h = cv2.boundingRect(contour)
                imageFrame = cv2.rectangle(imageFrame, (x, y),(x + w, y + h), (0, 255, 0), 2)
                cv2.putText(imageFrame, "Green Colour", (x, y),cv2.FONT_HERSHEY_SIMPLEX,1.0, (0, 255, 0))

                detection_time = detection_time+1
                detection_x = detection_x + (x+0.5*w)
                detection_y = detection_y + (y+0.5*h)

                if detection_time>= 5:
                    rospy.loginfo('-----------------detection-----------------')
                    avg_x, avg_y = detection_x/detection_time, detection_y/detection_time
                    imageFrame = cv2.circle(imageFrame, ( int(avg_x), int(avg_y) ), radius=5, color=(0, 0, 255), thickness=10 )

                    detection_info = Int32MultiArray()
                    detection_info.data = [imageFrame.shape[1],imageFrame.shape[0],avg_x,avg_y]
                    pub_info.publish(detection_info)

                    detection_time = 0
                    detection_x, detection_y = 0, 0



        image_message = bridge.cv2_to_imgmsg(imageFrame, encoding="bgr8")
        pub_img.publish(image_message)
        rospy.loginfo('camera ok')
        rate.sleep()
