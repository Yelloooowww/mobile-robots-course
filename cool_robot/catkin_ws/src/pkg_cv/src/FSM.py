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
from pkg_cv.msg import IntArrayWithHeader,FloatArrayWithHeader



class Control(object):
    def __init__(self):
        self.controller_Obstacle = PID_control("cool robot control", P=2.2, I=0.0, D=0.05)
        self.controller_tracking = PID_control("cool robot control",  P=25, I=0.0, D=0)
        self.pub_motor = rospy.Publisher('motor_control', Int32MultiArray ,queue_size=10)
        self.sub_ultrasonic = rospy.Subscriber("ultrasonic_header", FloatArrayWithHeader,self.cb_ultrasonic,queue_size = 10)
        self.sub_info = rospy.Subscriber("detection_info_header", IntArrayWithHeader, self.cb_detection_info, queue_size=10)

        self.ultrasonic_middle = None
        self.ultrasonic_left = None
        self.ultrasonic_right = None
        self.timer = rospy.Timer(rospy.Duration(0.2), self.timer_control_loop)

        self.advance_speed = 100
        self.update_ultrasonic = False  
        self.update_camera = False      
        self.goal_index = None
        self.goal_area = None

        self.change_state_to_tracking = 0
        self.change_state_to_obstacle = 0
        self.state = "obstacle"

        print("FSM init done")

        
    def cb_ultrasonic(self,msg):
        self.update_ultrasonic = True
        self.ultrasonic_middle = float(msg.data[0])
        self.ultrasonic_left = float(msg.data[1])
        self.ultrasonic_right = float(msg.data[2])

        if self.state == "obstacle":
            self.obstacle_control_loop()



    def cb_detection_info(self,msg):
        self.update_camera = True
        if len(msg.data) == 5:
            self.goal_index = float( float(msg.data[2])/float(msg.data[0]) )-0.5
            self.goal_area = msg.data[4]
            if self.goal_area > 32000: 
                print('force tracking')
                self.state = "tracking"
        else :
            self.goal_index = None
            self.goal_area = 0

        if self.state == "tracking":
            self.tracking_control_loop()
        


    def tracking_control_loop(self):
        rospy.loginfo('State: tracking')
        self.advance_speed = 80
        motor_array = Int32MultiArray()
        if self.goal_index == None:
                rospy.loginfo('No Goal')
                motor_array.data = [ 1, 1, 0, 0 ]

                self.change_state_to_obstacle = self.change_state_to_obstacle+1
                if self.change_state_to_obstacle >50 :
                    self.state = "obstacle"
                    self.change_state_to_obstacle = 0
                    print("------ change_state_to_obstacle ------")
                    
        else:
            if self.goal_area > 80000:
                motor_array.data = [ 1, 1, 0, 0 ]
                rospy.loginfo('Goal Reach')
            else:
                self.controller_tracking.update(self.goal_index)
                u=self.controller_tracking.output
                motor_array.data = [ 1, 1, int(self.advance_speed-u), int(self.advance_speed+u) ]
                rospy.loginfo('tracking~~~')
        print('motor=',motor_array.data)
        self.pub_motor.publish(motor_array)


    def obstacle_control_loop(self):
        rospy.loginfo('State: obstacle')
        self.advance_speed = 100
        motor_array = Int32MultiArray()
        motor_array.data = [ 1, 1, 0, 0 ]

        

        # turn right
        if (self.ultrasonic_left < 40 ) and (self.ultrasonic_right > 40 )  and (self.ultrasonic_middle < 40 ):
            self.controller_Obstacle.update(float(self.ultrasonic_left-40))
            u=self.controller_Obstacle.output
            motor_array.data = [ 1, 1, int(self.advance_speed+u), int(self.advance_speed-u) ]
            # rospy.loginfo('~~~~~~See Obstacle(left&forward)~~~~~~')
        
        # turn right
        elif (self.ultrasonic_left < 40 ) and (self.ultrasonic_right > 40 )  and (self.ultrasonic_middle > 40 ):
            self.controller_Obstacle.update(float(self.ultrasonic_left-40))
            u=self.controller_Obstacle.output
            motor_array.data = [ 1, 1, int(self.advance_speed+u), int(self.advance_speed-u) ]
            # rospy.loginfo('~~~~~~See Obstacle(left)~~~~~~')
        
        #turn left
        elif (self.ultrasonic_left > 40 ) and (self.ultrasonic_right < 40 ) and (self.ultrasonic_middle < 40 ) :
            self.controller_Obstacle.update(float(self.ultrasonic_right-40))
            u=self.controller_Obstacle.output
            motor_array.data = [ 1, 1, int(self.advance_speed-u), int(self.advance_speed+u) ]
            # rospy.loginfo('~~~~~~See Obstacle(right&forward)~~~~~~')
        
        #turn left
        elif (self.ultrasonic_left > 40 ) and (self.ultrasonic_right < 40 ) and (self.ultrasonic_middle > 40 ) :
            self.controller_Obstacle.update(float(self.ultrasonic_right-40))
            u=self.controller_Obstacle.output
            motor_array.data = [ 1, 1, int(self.advance_speed-u), int(self.advance_speed+u) ]
            # rospy.loginfo('~~~~~~See Obstacle(right)~~~~~~')

        # back
        elif (self.ultrasonic_left < 40 ) and (self.ultrasonic_right < 40 ) and (self.ultrasonic_middle < 40 ):
            if self.ultrasonic_left < self.ultrasonic_right:
                self.controller_Obstacle.update(float(self.ultrasonic_right-40))
            else:
                self.controller_Obstacle.update(float(self.ultrasonic_left-40))
            u=self.controller_Obstacle.output
            motor_array.data = [ 1, 1, int(-(self.advance_speed+u)), int(-(self.advance_speed-u)) ]
            # rospy.loginfo('~~~~~~See Obstacle(all)~~~~~~')

        # advance
        elif (self.ultrasonic_left < 40 ) and (self.ultrasonic_right < 40 ) and (self.ultrasonic_middle > 40 ):
            if self.ultrasonic_left < self.ultrasonic_right:
                self.controller_Obstacle.update(float(self.ultrasonic_right-40))
            else:
                self.controller_Obstacle.update(float(self.ultrasonic_left-40))
            u=self.controller_Obstacle.output
            motor_array.data = [ 1, 1, int(self.advance_speed-u), int(self.advance_speed+u) ]
            # rospy.loginfo('~~~~~~See Obstacle(right&left)~~~~~~')

        # back
        elif (self.ultrasonic_left > 40 ) and (self.ultrasonic_right > 40 ) and (self.ultrasonic_middle < 40 ):
            motor_array.data = [ 1, 1, int(-self.advance_speed), int(-self.advance_speed) ]
            # rospy.loginfo('~~~~~~See Obstacle(forward only)~~~~~~')

        # tracking
        elif (self.ultrasonic_left > 40 ) and (self.ultrasonic_right > 40 ) and (self.ultrasonic_middle > 40 ):
            motor_array.data = [ 1, 1, int(self.advance_speed), int(self.advance_speed) ]
            # rospy.loginfo('~~~~~~No Obstacle~~~~~~')

            self.change_state_to_tracking = self.change_state_to_tracking+1
            if self.change_state_to_tracking >50 :
                self.state = "tracking"
                self.change_state_to_tracking = 0
                print("------ change_state_to_tracking ------")
        


        

        
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








    def timer_control_loop(self,event):
        if self.state == "obstacle":
            if self.update_ultrasonic == False:
                rospy.loginfo('!!! Ultrasonic Timeout stop !!!')
                motor_array = Int32MultiArray()
                motor_array.data = [ 1, 1, 0, 0 ]
                self.pub_motor.publish(motor_array)
            

        if self.state == "tracking":
            if self.update_camera == False:
                rospy.loginfo('!!! Camera Timeout stop !!!')
                motor_array = Int32MultiArray()
                motor_array.data = [ 1, 1, 0, 0 ]
                self.pub_motor.publish(motor_array)

        self.update_camera = False
        self.update_ultrasonic = False






if __name__ == "__main__":
    rospy.init_node("Control")
    rot=Control()
    rospy.spin()
