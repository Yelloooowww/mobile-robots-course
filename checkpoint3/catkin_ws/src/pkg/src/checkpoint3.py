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

        self.sub_led = rospy.Subscriber("pub_led", Int32, self.cb_led, queue_size=1)
        self.pub_int = rospy.Publisher("array", Int32, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.2), self.control_loop)
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



    def control_loop(self,event):
        my_input7 = wiringpi.digitalRead(7)
        my_input8 = wiringpi.digitalRead(8)
        my_input9 = wiringpi.digitalRead(9)
        print(self.led,my_input7,my_input8,my_input9)

        if my_input9==1:
            if my_input7==0 and my_input8==0:
                self.now_action="back"
                if not self.now_action==self.last_action:
                    self.back()
                    time.sleep(1)
                    self.left()
                    time.sleep(0.5)
                else:
                    self.back()

            if my_input7==1 and my_input8==0:
                self.now_action="left"
                if not self.now_action==self.last_action:
                    self.back()
                    time.sleep(0.5)
                    self.left()
                    time.sleep(2)

            if my_input7==0 and my_input8==1:
                self.now_action="right"
                if not self.now_action==self.last_action:
                    self.back()
                    time.sleep(0.5)
                    self.right()
                    time.sleep(2)

            if my_input7==1 and my_input8==1:
                if self.last_led - self.led< -50:
                    tmp=random.random()
                    if tmp>0.5:
                        self.now_action="right"
                        if not self.now_action==self.last_action:
                            self.right()
                    else:
                        self.now_action="left"
                        if not self.now_action==self.last_action:
                            self.left()
                else:
                    self.now_action="advance"
                    self.advance()

        else :
            self.now_action="stop"
            if not self.now_action==self.last_action:
                self.stop()

        print(self.now_action)
        self.last_led=self.led
        self.last_action=self.now_action



if __name__ == "__main__":
    rospy.init_node("Control")
    control=Control()
    rospy.spin()
