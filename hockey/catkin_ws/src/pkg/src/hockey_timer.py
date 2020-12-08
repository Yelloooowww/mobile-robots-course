#!/usr/bin/env python
import wiringpi
import time
import random
import rospy
import numpy as np
from std_msgs.msg import Int32




class Control(object):
    def __init__(self):
        wiringpi.wiringPiSetup()
        wiringpi.pinMode(7, 0)
        wiringpi.pinMode(8, 0)
        wiringpi.pinMode(9, 0)

        self.excute_time=0

        self.led=-1000
        self.last_action=None
        self.now_action=None
        self.next_action=None
        self.next_next_action=None
        self.state="NoGoal"
        self.time=0

        self.sub_led = rospy.Subscriber("pub_led", Int32, self.cb_led, queue_size=1)
        self.sub_IR = rospy.Subscriber("pub_IR", Int32, self.cb_IR, queue_size=1)
        self.pub_int = rospy.Publisher("array", Int32, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)

        self.get_IR=0
        self.IR_1=0
        self.IR=0
        self.sample_times=50


        self.have_eat_ball=False
        self.loss_ball_time=0

        #get some param.
        self.led_low=rospy.get_param("/hockey_timer/led_low")
        self.led_high=rospy.get_param("/hockey_timer/led_high")
        self.led_eat=rospy.get_param("/hockey_timer/led_eat")
        self.IR_goal=rospy.get_param("/hockey_timer/IR_goal")
        self.IR_err=rospy.get_param("/hockey_timer/IR_err")

        self.start_search_time=rospy.get_param("/hockey_timer/start_search_time")
        self.modify_back_time=rospy.get_param("/hockey_timer/modify_back_time")
        self.modify_stop_time=rospy.get_param("/hockey_timer/modify_stop_time")
        self.modify_turn_time=rospy.get_param("/hockey_timer/modify_turn_time")
        self.modify_turn_time_back=rospy.get_param("/hockey_timer/modify_turn_time_back")
        self.find_mouth_advance_time=rospy.get_param("/hockey_timer/find_mouth_advance_time")


        print("init done")


    def cb_led(self,msg):
        self.led=msg.data

    def cb_IR(self,msg):
        if msg.data>512:
            self.IR_1 = self.IR_1+1

        self.get_IR=self.get_IR+1
        if self.get_IR>=self.sample_times:
            self.IR = float(float(self.IR_1)/float(self.sample_times))
            self.get_IR=0
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
            self.next_action="stop"
            self.next_next_action="left"
            self.state="Modify"
            self.modify_turn_time=self.modify_turn_time_back
            self.time=self.modify_back_time+self.modify_stop_time+self.modify_turn_time

        elif my_input7==1 and my_input8==0 and my_input9==1: #hit
            self.now_action="back"
            self.next_action="stop"
            self.next_next_action="left"
            self.state="Modify"
            self.time=self.modify_back_time+self.modify_stop_time+self.modify_turn_time

        elif my_input7==0 and my_input8==1 and my_input9==1: #hit
            self.now_action="back"
            self.next_action="stop"
            self.next_next_action="right"
            self.state="Modify"
            self.time=self.modify_back_time+self.modify_stop_time+self.modify_turn_time

        if my_input7==1 and my_input8==1 and my_input9==1: #free

            if not self.have_eat_ball:
            #for LED sensor
                if self.led<=self.led_low:
                    self.state="FindGoal"
                    self.time=10*2

                elif self.led<=self.led_high and self.led>self.led_low and self.state!="Modify" and self.excute_time>self.start_search_time:
                    if self.state!="NearGoal":
                        self.time=40*2
                        self.now_action="left"
                        self.next_action="right"
                    self.state="NearGoal"





        if my_input9==0 or self.led<=self.led_eat: #have catched the ball
            self.have_eat_ball=True
            if self.state!="Finish":
                self.time=30*10*2
                self.state="Finish"
                self.now_action="right"
                self.next_action="stop"

        if my_input9==1 and self.have_eat_ball==True:
            self.loss_ball_time=self.loss_ball_time+1
            print('loss_ball_time=',self.loss_ball_time)
            if self.loss_ball_time>30*2:
                print('------------loss the ball------------')
                self.have_eat_ball=False
                self.loss_ball_time=0
                self.state="NoGoal"






        #update
        if self.state=="Modify":
            if self.time==(self.modify_stop_time+self.modify_turn_time):
                self.now_action=self.next_action
                self.next_action=self.next_next_action
                self.next_next_action=None
            elif self.time==self.modify_turn_time:
                self.now_action=self.next_action
                self.next_action=None
                self.next_next_action=None
            elif self.time<0:
                if self.have_eat_ball:
                    self.state="Finish"
                    self.time=0
                else:
                    self.state="NoGoal"
                    self.time=0



        if self.state=="NearGoal":
            if self.time==30*2:
                self.now_action=self.next_action
                self.next_action="left"
            if self.time==10*2:
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
            if self.time>0:
                if (self.time%4)>1:
                    self.now_action="right"
                else:
                    self.now_action="stop"

                # if not self.IR==0:
                if self.IR<self.IR_goal+self.IR_err and self.IR>self.IR_goal-self.IR_err:
                    self.state=="Find_mouth"
                    self.now_action="advance"
                    self.time=self.find_mouth_advance_time

            if self.time==0:
                self.time=30*10*2
                self.state="Finish"
                self.now_action="right"





        if self.state=="Find_mouth":
            self.now_action="advance"
            if self.time<0:
                self.time=30*10*2
                self.state="Finish"
                self.now_action="right"





        
        

        if not self.now_action=="advance":
            self.motor_control()
        else:
            if not self.last_action==self.now_action:
                self.motor_control()


        self.excute_time=self.excute_time+1
        self.time=self.time-1
        self.last_action=self.now_action

        print(self.IR,self.led,self.state,self.now_action,self.time)




if __name__ == "__main__":
    rospy.init_node("Control")
    control=Control()
    rospy.spin()
