#!/usr/bin/env python

import wiringpi
import time
import random
import rospy
from std_msgs.msg import Int32



class Control(object):
    def __init__(self):

        self.sub_led = rospy.Subscriber("pub_led", Int32, self.cb_led, queue_size=1)
        wiringpi.wiringPiSetup()
        wiringpi.pinMode(7, 0) 
        wiringpi.pinMode(8, 0) 
        wiringpi.pinMode(9, 0) 

        self.pub_int = rospy.Publisher("array", Int32, queue_size=1)        

        self.pub_int.publish(5)
        time.sleep(1)
        self.led=-1000
        print("init done")

    def cb_led(self,msg):
        self.led=msg.data            


    def advance(self):
        self.pub_int.publish(1)
        print('~~~pub advance~~~')

    def right(self):
        self.pub_int.publish(2)
        print('~~~pub right~~~')

    def left(self):
        self.pub_int.publish(3)
        print('~~~pub left~~~')

    def back(self):
        self.pub_int.publish(4)
        print('~~~pub back~~~')

    def stop(self):
        self.pub_int.publish(5)
        print('~~~pub stop~~~')


if __name__ == "__main__":
    rospy.init_node("Control")
    control=Control()

    last=-1000
    last_action=None
    now_action=None

    while not rospy.is_shutdown():
        my_input7 = wiringpi.digitalRead(7)
        my_input8 = wiringpi.digitalRead(8)
        my_input9 = wiringpi.digitalRead(9)
        print(control.led,my_input7,my_input8,my_input9)

        if my_input9==1:
            if my_input7==0 and my_input8==0:
                now_action="back"
                if not now_action==last_action:
                    control.back()
                    time.sleep(1)
                    control.left()
                    time.sleep(0.5)
                else:
                    control.back()


            if my_input7==1 and my_input8==0:
                now_action="left"
                if not now_action==last_action:
                    control.back()
                    time.sleep(0.5)
                    control.left()
                    time.sleep(2)

            if my_input7==0 and my_input8==1:
                now_action="right"
                if not now_action==last_action:
                    control.back()
                    time.sleep(0.5)
                    control.right()
                    time.sleep(2)

            if my_input7==1 and my_input8==1:
                if last - control.led< -50:
                    tmp=random.random()
                    if tmp>0.5:
                        now_action="right"
                        if not now_action==last_action:
                            control.right()
                    else:
                        now_action="left"
                        if not now_action==last_action:
                            control.left()
                else:
                    now_action="advance"
                    control.advance()
        
        else :
            now_action="stop"
            if not now_action==last_action:
                control.stop()
        
        print(now_action)
        last=control.led
        last_action=now_action

        time.sleep(0.1)