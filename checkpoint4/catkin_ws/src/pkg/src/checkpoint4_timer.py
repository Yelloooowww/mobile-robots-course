#!/usr/bin/env python
import wiringpi
import time
import random
import rospy
from std_msgs.msg import Int32




class Control(object):
    def __init__(self):
        wiringpi.wiringPiSetup()
        wiringpi.pinMode(7, 0)
        wiringpi.pinMode(8, 0)
        wiringpi.pinMode(9, 0)

        self.led=-1000
        self.last_led=-1000
        self.last_action=None
        self.now_action=None
        self.state="NoGoal"
        self.time=0

        self.sub_led = rospy.Subscriber("pub_led", Int32, self.cb_led, queue_size=1)
        self.sub_IR = rospy.Subscriber("pub_IR", Int32, self.cb_IR, queue_size=1)
        self.pub_int = rospy.Publisher("array", Int32, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

        self.get_IR=0
        self.IR_1=0
        self.IR=0
        self.sample_times=50

        print("init done")


    def cb_led(self,msg):
        self.led=msg.data

    def cb_IR(self,msg):
        # print('msg.data=',msg.dasta)
        if msg.data>512:
            self.IR_1 = self.IR_1+1

        self.get_IR=self.get_IR+1
        if self.get_IR>=self.sample_times:
            self.IR = self.IR_1/self.sample_times
            print(self.IR_1,self.IR)


            self.get_IR=0
            self.IR_0=0
            self.IR_1=0




    
    def motor_control(self):
        if self.now_action=="advance":
            self.pub_int.publish(1)
            # print('~~~pub advance~~~')
        if self.now_action=="right":
            self.pub_int.publish(2)
            # print('~~~pub right~~~')
        if self.now_action=="left":
            self.pub_int.publish(3)
            # print('~~~pub left~~~')
        if self.now_action=="back":
            self.pub_int.publish(4)
            # print('~~~pub back~~~')
        if self.now_action=="stop":
            self.pub_int.publish(5)
            # print('~~~pub stop~~~')




    def control_loop(self,event):
        #for bump sensor
        my_input7 = wiringpi.digitalRead(7)
        my_input8 = wiringpi.digitalRead(8)
        my_input9 = wiringpi.digitalRead(9)

        if my_input7==0 and my_input8==0 and my_input9==1: #hit
            self.now_action="back"
            self.next_action="left"
            self.state="Modify"
            self.time=20
            
        if my_input7==1 and my_input8==0 and my_input9==1: #hit
            self.now_action="back"
            self.next_action="left"
            self.state="Modify"
            self.time=20

        if my_input7==0 and my_input8==1 and my_input9==1: #hit
            self.now_action="back"
            self.next_action="right"
            self.state="Modify"
            self.time=20

        if my_input7==1 and my_input8==1 and my_input9==1: #free

            #for LED sensor
            if self.led<=475:
                # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!FindGoal!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1!!!!")
                self.state="FindGoal"
                self.time=10

            elif self.led<=550 and self.led>475 and self.state!="Modify":
                # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~NearGoal~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                if self.state!="NearGoal":
                    self.time=40
                    self.now_action="left"
                    self.next_action="right"
                self.state="NearGoal"    



 

        if my_input9==0 : #have catched the ball
            self.state="Finish"




        #update
        if self.state=="Modify":
            if self.time==14:
                self.now_action=self.next_action
                self.next_action=None
            if self.time<0:
                self.state="NoGoal"
                self.time=0
            
            

        if self.state=="NearGoal":
            if self.time==30:
                self.now_action=self.next_action
                self.next_action="left"
            if self.time==10:
                self.now_action=self.next_action
                self.next_action=None
            if self.time<0:
                self.state="NoGoal"
                self.time=0


        if self.state=="FindGoal":
            self.now_action="advance"
            if self.time<0:
                self.state="NoGoal"
                self.time=0

        if self.state=="NoGoal":
            self.now_action="advance"

        if self.state=="Finish":
            self.now_action="stop"



        
        self.time=self.time-1
        self.last_led=self.led
        self.last_action=self.now_action
        self.motor_control()    
        # print('sample_time=',self.get_IR,'IR=',self.IR)
        # print(self.led,my_input7,my_input8,my_input9,self.state,self.now_action,self.time)
        



if __name__ == "__main__":
    rospy.init_node("Control")
    control=Control()
    rospy.spin()
