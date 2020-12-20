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
import message_filters
from pkg_cv.msg import IntArrayWithHeader,FloatArrayWithHeader



class Control(object):
    def __init__(self):
        self.controller_Obstacle = PID_control("obstacle control", P=2.2, I=0.0, D=0.05)
        self.controller_tracking = PID_control("tracking control", P=100, I=0.0, D=20)
        sub_ultrasonic = message_filters.Subscriber('/ultrasonic_header',FloatArrayWithHeader)
        sub_info = message_filters.Subscriber('/detection_info_header',IntArrayWithHeader)
        ts = message_filters.ApproximateTimeSynchronizer([sub_ultrasonic, sub_info], 10, slop=0.1)
		ts.registerCallback(self.callback)
        self.pub_motor = rospy.Publisher('motor_control', Int32MultiArray ,queue_size=10)

        self.ultrasonic_middle = None
        self.ultrasonic_left = None
        self.ultrasonic_right = None
        self.update_ultrasonic = False
        self.goal_index = None
        self.goal_area = 0
        self.advance_speed = 90
        self.force_tracking = False
        self.update = False
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timeout_control)
        print("init done")


    def callback(self,msg_ultrasonic, msg_info):
        self.update = True
        self.ultrasonic_middle = float(msg_ultrasonic.data[0])
        self.ultrasonic_left = float(msg_ultrasonic.data[1])
        self.ultrasonic_right = float(msg_ultrasonic.data[2])
        if len(msg_info.data) == 5:
            rospy.loginfo('See Goal')
            self.goal_index = float( float(msg_info.data[2])/float(msg_info.data[0]) )-0.5
            self.goal_area = msg.data[4]
        else :
            rospy.loginfo('No Goal')
            self.goal_index = None
            self.goal_area = 0
        self.motor_control()


    def motor_control(self):
        motor_array = Int32MultiArray()
        motor_array.data = [ 1, 1, 0, 0 ]
        self.force_tracking = False

        # turn right
        if (self.ultrasonic_left < 40 ) and (self.ultrasonic_right > 40 )  and (self.ultrasonic_middle < 40 ):
            self.controller_Obstacle.update(float(self.ultrasonic_left-40))
            u=self.controller_Obstacle.output

            motor_array.data = [ 1, 1, int(self.advance_speed+u), int(self.advance_speed-u) ]
            rospy.loginfo('~~~~~~See Obstacle(left&forward)~~~~~~')


        # turn right
        elif (self.ultrasonic_left < 40 ) and (self.ultrasonic_right > 40 )  and (self.ultrasonic_middle > 40 ):
            self.controller_Obstacle.update(float(self.ultrasonic_left-40))
            u=self.controller_Obstacle.output

            motor_array.data = [ 1, 1, int(self.advance_speed+u), int(self.advance_speed-u) ]
            rospy.loginfo('~~~~~~See Obstacle(left)~~~~~~')

        #turn left
        elif (self.ultrasonic_left > 40 ) and (self.ultrasonic_right < 40 ) and (self.ultrasonic_middle < 40 ) :
            self.controller_Obstacle.update(float(self.ultrasonic_right-40))
            u=self.controller_Obstacle.output

            motor_array.data = [ 1, 1, int(self.advance_speed-5*u), int(self.advance_speed+5*u) ]
            rospy.loginfo('~~~~~~See Obstacle(right&forward)~~~~~~')

        #turn left
        elif (self.ultrasonic_left > 40 ) and (self.ultrasonic_right < 40 ) and (self.ultrasonic_middle > 40 ) :
            self.controller_Obstacle.update(float(self.ultrasonic_right-40))
            u=self.controller_Obstacle.output

            motor_array.data = [ 1, 1, int(self.advance_speed-u), int(self.advance_speed+u) ]
            rospy.loginfo('~~~~~~See Obstacle(right)~~~~~~')

        # back
        elif (self.ultrasonic_left < 40 ) and (self.ultrasonic_right < 40 ) and (self.ultrasonic_middle < 40 ):
            if self.ultrasonic_left < self.ultrasonic_right:
                self.controller_Obstacle.update(float(self.ultrasonic_right-40))
            else:
                self.controller_Obstacle.update(float(self.ultrasonic_left-40))
            u=self.controller_Obstacle.output
            motor_array.data = [ 1, 1, int(-(self.advance_speed+10*u)), int(-(self.advance_speed-10*u)) ]
            rospy.loginfo('~~~~~~See Obstacle(all)~~~~~~')

        # advance
        elif (self.ultrasonic_left < 40 ) and (self.ultrasonic_right < 40 ) and (self.ultrasonic_middle > 40 ):
            if self.ultrasonic_left < self.ultrasonic_right:
                self.controller_Obstacle.update(float(self.ultrasonic_right-40))
            else:
                self.controller_Obstacle.update(float(self.ultrasonic_left-40))
            u=self.controller_Obstacle.output
            motor_array.data = [ 1, 1, int(self.advance_speed-u), int(self.advance_speed+u) ]
            rospy.loginfo('~~~~~~See Obstacle(right&left)~~~~~~')

        # back
        elif (self.ultrasonic_left > 40 ) and (self.ultrasonic_right > 40 ) and (self.ultrasonic_middle < 40 ):
            motor_array.data = [ 1, 1, int(-self.advance_speed), int(-self.advance_speed) ]
            rospy.loginfo('~~~~~~See Obstacle(forward only)~~~~~~')

        # tracking
        elif (self.ultrasonic_left > 40 ) and (self.ultrasonic_right > 40 ) and (self.ultrasonic_middle > 40 ):
            rospy.loginfo('~~~~~~No Obstacle~~~~~~')
            if self.goal_index != None:
                self.force_tracking = True
            else:
                motor_array.data = [ 1, 1, int(self.advance_speed), int(self.advance_speed) ]


        if self.goal_area > 20000:
                self.force_tracking = True
                print('force_tracking')

        if  self.force_tracking == True :
                if self.goal_index != None:
                    if self.goal_area > 100000:
                        motor_array.data = [ 1, 1, 0, 0 ]
                        rospy.loginfo('Goal Reach')
                    else :
                        self.controller_tracking.update(self.goal_index)
                        u=self.controller_tracking.output
                        motor_array.data = [ 1, 1, int(self.advance_speed-u), int(self.advance_speed+u) ]
                        rospy.loginfo('tracking')

        if motor_array.data[2]<0:
            motor_array.data[2] = abs(motor_array.data[2])
            motor_array.data[0] = 0
        if motor_array.data[3]<0:
            motor_array.data[3] = abs(motor_array.data[3])
            motor_array.data[1] = 0
        if motor_array.data[2]>=200:
            motor_array.data[2] = 200
        if motor_array.data[3]>200:
            motor_array.data[3] = 200

        print(motor_array.data)
        self.pub_motor.publish(motor_array)


    def timeout_control(self,event):
        if self.update == False :
            rospy.loginfo('!!! Timeout stop !!!')
            motor_array = Int32MultiArray()
            motor_array.data = [ 1, 1, 0, 0 ]
            self.pub_motor.publish(motor_array)

        self.update_ultrasonic = False




if __name__ == "__main__":
    rospy.init_node("Control")
    cool_robot=Control()
    rospy.spin()
