#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import String,Int32,Int32MultiArray,Float32MultiArray
import roslib
import struct
import time
import rospkg
from nav_msgs.msg import Odometry,OccupancyGrid
from PID import PID_control



class Control(object):
    def __init__(self):
        self.controller_tracking = PID_control("cool robot control",  P=25, I=0.0, D=0)
        self.pub_motor = rospy.Publisher('motor_control', Int32MultiArray ,queue_size=10)
        self.sub_info = rospy.Subscriber("detection_info", Int32MultiArray, self.cb_detection_info, queue_size=10)
        self.goal_index = None
        self.goal_area = None

        self.advance_speed = 80
        self.no_goal = 0

        self.timer = rospy.Timer(rospy.Duration(0.2), self.timer_control_loop)
        print("Tracking init done")


    def cb_detection_info(self,msg):
        self.goal_index = float( float(msg.data[2])/float(msg.data[0]) )-0.5
        self.goal_area = msg.data[4]

        self.control_loop()
        




    def control_loop(self):
        if self.goal_index != None:
            if self.goal_area > 80000:
                motor_array = Int32MultiArray()
                motor_array.data = [ 1, 1, 0, 0 ]
                self.pub_motor.publish(motor_array)
                rospy.loginfo('Goal Reach')
            else:
                self.controller_tracking.update(self.goal_index)
                u=self.controller_tracking.output
                motor_array = Int32MultiArray()
                motor_array.data = [ 1, 1, int(self.advance_speed-u), int(self.advance_speed+u) ]
                self.pub_motor.publish(motor_array)
                rospy.loginfo('See goal')
                print('goal_index=',self.goal_index,'u=',u,'motor=',motor_array.data)



        self.goal_index = None

        



    def timer_control_loop(self,event):
        if self.goal_index == None:
            self.no_goal = self.no_goal+1
            if self.no_goal>=5:
                rospy.loginfo(self.goal_index)
                motor_array = Int32MultiArray()
                motor_array.data = [ 1, 1, 0, 0 ]
                self.pub_motor.publish(motor_array)

                self.no_goal = 0






if __name__ == "__main__":
    rospy.init_node("Control")
    rot=Control()
    rospy.spin()
