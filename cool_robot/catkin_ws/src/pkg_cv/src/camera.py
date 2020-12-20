#!/usr/bin/python
import rospy
import numpy as np
import cv2  # OpenCV module
import time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String,Int32,Int32MultiArray
from pkg_cv.msg import IntArrayWithHeader,FloatArrayWithHeader




if __name__ == '__main__':
    rospy.init_node('cam_read', anonymous=True)
    pub_img = rospy.Publisher('img_detection', Image,queue_size=10)
    pub_info=rospy.Publisher('detection_info',Int32MultiArray,queue_size = 10)
    pub_info_header=rospy.Publisher('detection_info_header',IntArrayWithHeader,queue_size = 10)
    rate = rospy.Rate(10) # 10hz
    bridge = CvBridge()
    cap = cv2.VideoCapture(0)
    print('camera init done')


    while not rospy.is_shutdown():
        # Capture frame-by-frame
        ret, imageFrame = cap.read()
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

        # lower mask (0-10)
        lower_red = np.array([0,50,50], np.uint8)
        upper_red = np.array([10,255,255], np.uint8)
        mask0 = cv2.inRange(hsvFrame, lower_red, upper_red)

        # upper mask (170-180)
        lower_red = np.array([170,50,50], np.uint8)
        upper_red = np.array([180,255,255], np.uint8)
        mask1 = cv2.inRange(hsvFrame, lower_red, upper_red)

        # join my masks
        red_mask = mask0+mask1

        kernal = np.ones((5, 5), "uint8")

        # For green color
        red_mask = cv2.dilate(red_mask, kernal)
        res_green = cv2.bitwise_and(imageFrame, imageFrame,mask = red_mask)
        _, contours, hierarchy = cv2.findContours(red_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)


        max_area, max_x, max_y, max_w, max_h = 0, None, None, None, None
        detection_time = 0
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 300):
                x, y, w, h = cv2.boundingRect(contour)
                if area > max_area:
                    max_area,max_x,max_y,max_w,max_h = area, x, y, w, h
                imageFrame = cv2.rectangle(imageFrame, (x, y),(x + w, y + h), (0, 255, 0), 2)

                detection_time = detection_time+1

        if detection_time>= 1:
            rospy.loginfo('-----------------detection-----------------')
            imageFrame = cv2.rectangle(imageFrame, (max_x, max_y),(max_x + max_w, max_y + max_h), (0, 0, 255), 2)
            imageFrame = cv2.circle(imageFrame, ( int(max_x+0.5*max_w), int(max_y+0.5*max_h) ), radius=5, color=(0, 0, 255), thickness=10 )

            detection_info = Int32MultiArray()
            detection_info.data = [imageFrame.shape[1],imageFrame.shape[0],max_x+0.5*max_w ,max_y+0.5*max_h,max_area]
            pub_info.publish(detection_info)

            detection_info_header = IntArrayWithHeader()
            detection_info_header.data = [imageFrame.shape[1],imageFrame.shape[0],max_x+0.5*max_w ,max_y+0.5*max_h,max_area]
            detection_info_header.header.stamp = rospy.Time.now()
            pub_info_header.publish(detection_info_header)
        else :
            detection_info_header = IntArrayWithHeader()
            detection_info_header.data = []
            detection_info_header.header.stamp = rospy.Time.now()
            pub_info_header.publish(detection_info_header)






        image_message = bridge.cv2_to_imgmsg(imageFrame, encoding="bgr8")
        pub_img.publish(image_message)
        rospy.loginfo('camera ok')
        rate.sleep()
