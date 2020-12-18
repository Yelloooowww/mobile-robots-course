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
        self.controller = PID_control("cool robot control", P=100, I=0.0, D=20)
        self.controller_Obstacle = PID_control("cool robot control", P=2.2, I=0.0, D=0.05)
        self.pub_motor = rospy.Publisher('motor_control', Int32MultiArray ,queue_size=10)
        self.sub_info = rospy.Subscriber("detection_info", Int32MultiArray, self.cb_info, queue_size=10)
        self.sub_ultrasonic = rospy.Subscriber("ultrasonic", Float32MultiArray,self.cb_ultrasonic,queue_size = 10)
        self.goal_index = None
        self.goal_area = None
        self.ultrasonic_middle = None
        self.ultrasonic_left = None
        self.ultrasonic_right = None
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

        self.advance_speed = 100
        self.no_goal = 0

        self.state = "Obstacle"
        print("Control init done")


    def cb_info(self,msg):
        # print('goal_point=',msg.data[2],msg.data[3])
        self.goal_index = float( float(msg.data[2])/float(msg.data[0]) )-0.5
        

    def cb_ultrasonic(self,msg):
        self.ultrasonic_middle = float(msg.data[0])
        self.ultrasonic_left = float(msg.data[1])
        self.ultrasonic_right = float(msg.data[2])




    def control_loop(self,event):

        motor_array = Int32MultiArray()
        motor_array.data = [ 1, 1, 0, 0 ]

        

        # turn right
        if (self.ultrasonic_left < 40 ) and (self.ultrasonic_right > 40 )  and (self.ultrasonic_middle < 40 ):
            self.controller_Obstacle.update(float(self.ultrasonic_left-40))
            u=self.controller_Obstacle.output
            
            motor_array.data = [ 1, 1, int(self.advance_speed+u), int(self.advance_speed-u) ]
            # self.pub_motor.publish(motor_array)
            rospy.loginfo('~~~~~~See Obstacle(left&forward)~~~~~~')
        
        # turn right
        elif (self.ultrasonic_left < 40 ) and (self.ultrasonic_right > 40 )  and (self.ultrasonic_middle > 40 ):
            self.controller_Obstacle.update(float(self.ultrasonic_left-40))
            u=self.controller_Obstacle.output
            
            motor_array.data = [ 1, 1, int(self.advance_speed+u), int(self.advance_speed-u) ]
            # self.pub_motor.publish(motor_array)
            rospy.loginfo('~~~~~~See Obstacle(left)~~~~~~')
        
        #turn left
        elif (self.ultrasonic_left > 40 ) and (self.ultrasonic_right < 40 ) and (self.ultrasonic_middle < 40 ) :
            self.controller_Obstacle.update(float(self.ultrasonic_right-40))
            u=self.controller_Obstacle.output
            
            motor_array.data = [ 1, 1, int(self.advance_speed-u), int(self.advance_speed+u) ]
            # self.pub_motor.publish(motor_array)
            rospy.loginfo('~~~~~~See Obstacle(right&forward)~~~~~~')
        
        #turn left
        elif (self.ultrasonic_left > 40 ) and (self.ultrasonic_right < 40 ) and (self.ultrasonic_middle > 40 ) :
            self.controller_Obstacle.update(float(self.ultrasonic_right-40))
            u=self.controller_Obstacle.output
            
            motor_array.data = [ 1, 1, int(self.advance_speed-u), int(self.advance_speed+u) ]
            # self.pub_motor.publish(motor_array)
            rospy.loginfo('~~~~~~See Obstacle(right)~~~~~~')

        # back
        elif (self.ultrasonic_left < 40 ) and (self.ultrasonic_right < 40 ) and (self.ultrasonic_middle < 40 ):
            motor_array.data = [ 1, 1, int(-self.advance_speed), int(-self.advance_speed) ]
            # self.pub_motor.publish(motor_array)
            rospy.loginfo('~~~~~~See Obstacle(all)~~~~~~')

        # advance
        elif (self.ultrasonic_left < 40 ) and (self.ultrasonic_right < 40 ) and (self.ultrasonic_middle > 40 ):
            motor_array.data = [ 1, 1, int(self.advance_speed), int(self.advance_speed) ]
            # self.pub_motor.publish(motor_array)
            rospy.loginfo('~~~~~~See Obstacle(right&left)~~~~~~')

        # back
        elif (self.ultrasonic_left > 40 ) and (self.ultrasonic_right > 40 ) and (self.ultrasonic_middle < 40 ):
            motor_array.data = [ 1, 1, int(-self.advance_speed), int(-self.advance_speed) ]
            # self.pub_motor.publish(motor_array)
            rospy.loginfo('~~~~~~See Obstacle(forward only)~~~~~~')

        # tracking
        elif (self.ultrasonic_left > 40 ) and (self.ultrasonic_right > 40 ) and (self.ultrasonic_middle > 40 ):
            motor_array.data = [ 1, 1, int(self.advance_speed), int(self.advance_speed) ]
            # self.pub_motor.publish(motor_array)
            rospy.loginfo('~~~~~~No Obstacle~~~~~~')
            




        # if self.goal_index == None:
        #     self.no_goal = self.no_goal+1
        #     if self.no_goal>=5:
        #         rospy.loginfo(self.goal_index)
        #         # motor_array = Int32MultiArray()
        #         motor_array.data = [ 1, 1, 0, 0 ]
        #         # self.pub_motor.publish(motor_array)

        #         self.no_goal = 0


        # if self.goal_index != None:
        #     self.controller.update(self.goal_index)
        #     u=self.controller.output
        #     # motor_array = Int32MultiArray()
        #     motor_array.data = [ 1, 1, int(self.advance_speed-u), int(self.advance_speed+u) ]
        #     # self.pub_motor.publish(motor_array)

        #     rospy.loginfo('See goal')
        #     print('goal_index=',self.goal_index,'u=',u,'motor=',motor_array.data)



        # self.goal_index = None

        

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
