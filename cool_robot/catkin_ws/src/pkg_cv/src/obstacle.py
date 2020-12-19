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
        self.controller_Obstacle = PID_control("cool robot control", P=2.2, I=0.0, D=0.05)
        self.pub_motor = rospy.Publisher('motor_control', Int32MultiArray ,queue_size=10)
        self.sub_ultrasonic = rospy.Subscriber("ultrasonic", Float32MultiArray,self.cb_ultrasonic,queue_size = 10)
        self.ultrasonic_middle = None
        self.ultrasonic_left = None
        self.ultrasonic_right = None
        # self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

        self.advance_speed = 100
        print("Obstacle init done")

        
    def cb_ultrasonic(self,msg):
        self.ultrasonic_middle = float(msg.data[0])
        self.ultrasonic_left = float(msg.data[1])
        self.ultrasonic_right = float(msg.data[2])

        self.control_loop()




    def control_loop(self):

        motor_array = Int32MultiArray()
        motor_array.data = [ 1, 1, 0, 0 ]

        

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
            
            motor_array.data = [ 1, 1, int(self.advance_speed-u), int(self.advance_speed+u) ]
            rospy.loginfo('~~~~~~See Obstacle(right&forward)~~~~~~')
        
        #turn left
        elif (self.ultrasonic_left > 40 ) and (self.ultrasonic_right < 40 ) and (self.ultrasonic_middle > 40 ) :
            self.controller_Obstacle.update(float(self.ultrasonic_right-40))
            u=self.controller_Obstacle.output
            
            motor_array.data = [ 1, 1, int(self.advance_speed-u), int(self.advance_speed+u) ]
            rospy.loginfo('~~~~~~See Obstacle(right)~~~~~~')

        # back
        elif (self.ultrasonic_left < 40 ) and (self.ultrasonic_right < 40 ) and (self.ultrasonic_middle < 40 ):
            motor_array.data = [ 1, 1, int(-self.advance_speed), int(-self.advance_speed) ]
            rospy.loginfo('~~~~~~See Obstacle(all)~~~~~~')

        # advance
        elif (self.ultrasonic_left < 40 ) and (self.ultrasonic_right < 40 ) and (self.ultrasonic_middle > 40 ):
            motor_array.data = [ 1, 1, int(self.advance_speed), int(self.advance_speed) ]
            rospy.loginfo('~~~~~~See Obstacle(right&left)~~~~~~')

        # back
        elif (self.ultrasonic_left > 40 ) and (self.ultrasonic_right > 40 ) and (self.ultrasonic_middle < 40 ):
            motor_array.data = [ 1, 1, int(-self.advance_speed), int(-self.advance_speed) ]
            rospy.loginfo('~~~~~~See Obstacle(forward only)~~~~~~')

        # tracking
        elif (self.ultrasonic_left > 40 ) and (self.ultrasonic_right > 40 ) and (self.ultrasonic_middle > 40 ):
            motor_array.data = [ 1, 1, int(self.advance_speed), int(self.advance_speed) ]
            rospy.loginfo('~~~~~~No Obstacle~~~~~~')
            


        

        if motor_array.data[2]>255: motor_array.data[2] = 255
        if motor_array.data[3]>255: motor_array.data[3] = 255
        if motor_array.data[2]<0: 
            motor_array.data[2] = abs(motor_array.data[2])
            motor_array.data[0] = 0
        if motor_array.data[3]<0: 
            motor_array.data[3] = abs(motor_array.data[3])
            motor_array.data[1] = 0


        print(motor_array.data)
        self.pub_motor.publish(motor_array)












if __name__ == "__main__":
    rospy.init_node("Control")
    rot=Control()
    rospy.spin()
