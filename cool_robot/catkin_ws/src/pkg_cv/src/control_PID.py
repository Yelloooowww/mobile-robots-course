#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import String,Int32,Int32MultiArray
import roslib
import struct
import time
import rospkg
from nav_msgs.msg import Odometry,OccupancyGrid
from PID import PID_control



class Control(object):
    def __init__(self):
        self.controller=PID_control("cool robot control", P=130, I=0.0, D=15)
        self.pub_motor = rospy.Publisher('motor_control', Int32MultiArray ,queue_size=10)
        self.sub_info = rospy.Subscriber("detection_info", Int32MultiArray, self.cb_info, queue_size=10)
        self.goal_index = None
        self.timer = rospy.Timer(rospy.Duration(0.2), self.control_loop)

        self.advance_speed = 80
        self.no_goal = 0
        print("Control init done")


    def cb_info(self,msg):
        # print('goal_point=',msg.data[2],msg.data[3])
        self.goal_index = float( float(msg.data[2])/float(msg.data[0]) )-0.5


    def control_loop(self,event):

        if self.goal_index == None:
            self.no_goal = self.no_goal+1
            if self.no_goal>=5:
                rospy.loginfo(self.goal_index)
                motor_array = Int32MultiArray()
                motor_array.data = [ 1, 1, 0, 0 ]
                self.pub_motor.publish(motor_array)

                self.no_goal = 0


        if self.goal_index != None:
            self.controller.update(self.goal_index)
            u=self.controller.output
            motor_array = Int32MultiArray()
            motor_array.data = [ 1, 1, int(self.advance_speed-u), int(self.advance_speed+u) ]
            self.pub_motor.publish(motor_array)

            rospy.loginfo('See goal')
            print('goal_index=',self.goal_index,'u=',u,'motor=',motor_array.data)



        self.goal_index = None












if __name__ == "__main__":
    rospy.init_node("Control")
    rot=Control()
    rospy.spin()
