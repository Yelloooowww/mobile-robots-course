#!/usr/bin/env python
import wiringpi
import time
import random
import rospy
import numpy as np
from std_msgs.msg import Int32,Float32,Float32MultiArray
from pkg_cv.msg import IntArrayWithHeader,FloatArrayWithHeader

#set GPIO Pins
GPIO_TRIGGER_0 = 12
GPIO_ECHO_0 = 30
GPIO_TRIGGER_1 = 13
GPIO_ECHO_1 = 21
GPIO_TRIGGER_2 = 14
GPIO_ECHO_2 = 22



def distance(TRIGGER_PIN,ECHO_PIN):
    GPIO_TRIGGER = TRIGGER_PIN
    GPIO_ECHO = ECHO_PIN
    # set Trigger to HIGH
    wiringpi.digitalWrite(GPIO_TRIGGER,1)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    wiringpi.digitalWrite(GPIO_TRIGGER,0)

    StartTime = time.time()
    StopTime = time.time()

    tmpA = 0
    tmpB = 0

    timeout_flag = False
    # save StartTime
    while wiringpi.digitalRead(GPIO_ECHO) == 0:
        StartTime = time.time()
        tmpA = tmpA+1
        if tmpA > 1000:
            timeout_flag
            break

    # save time of arrival
    while wiringpi.digitalRead(GPIO_ECHO) == 1:
        StopTime = time.time()
        tmpB = tmpB+1
        if tmpB > 1000:
            timeout_flag
            break

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance,timeout_flag

if __name__ == '__main__':
    rospy.init_node('ultrasonic_sensor', anonymous=True)
    pub_ultrasonic = rospy.Publisher('ultrasonic',Float32MultiArray,queue_size = 10)
    pub_ultrasonic_header = rospy.Publisher('ultrasonic_header',FloatArrayWithHeader,queue_size = 10)
    rate = rospy.Rate(10) # 10hz

    #set GPIO direction (IN / OUT)
    wiringpi.wiringPiSetup()
    wiringpi.pinMode(GPIO_TRIGGER_0, 1)#OUT
    wiringpi.pinMode(GPIO_ECHO_0, 0)#IN
    wiringpi.pinMode(GPIO_TRIGGER_1, 1)#OUT
    wiringpi.pinMode(GPIO_ECHO_1, 0)#IN
    wiringpi.pinMode(GPIO_TRIGGER_2, 1)#OUT
    wiringpi.pinMode(GPIO_ECHO_2, 0)#IN
    print('ultrasonic_sensor INIT')


    while not rospy.is_shutdown():
        # dist_0,dist_1,dist_2 = distance()
        dist_0,timeout_flag_0=distance(GPIO_TRIGGER_0 ,GPIO_ECHO_0)
        dist_1,timeout_flag_1=distance(GPIO_TRIGGER_1 ,GPIO_ECHO_1)
        dist_2,timeout_flag_2=distance(GPIO_TRIGGER_2 ,GPIO_ECHO_2)

        if (timeout_flag_0 == False) and (timeout_flag_1 == False) and (timeout_flag_2 == False):
            dis_msg = Float32MultiArray()
            dis_msg.data = [dist_0,dist_1,dist_2]
            pub_ultrasonic.publish(dis_msg)

            dis_msg_header = FloatArrayWithHeader()
            dis_msg_header.data = [dist_0,dist_1,dist_2]
            dis_msg_header.header.stamp = rospy.Time.now()
            pub_ultrasonic_header.publish(dis_msg_header)
            # print ("Measured Distance = %.2f,%.2f,%.2f" %dist_0 dist_1 dist_2)
            print("Measured Distance ={:^5f} {:^5f} {:^5f}".format(dist_0,dist_1,dist_2))

        rate.sleep()
