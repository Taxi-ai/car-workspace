#!/usr/bin/env python

import rospy
import Adafruit_PCA9685
import time
from geometry_msgs.msg import Twist
from fsm_control.msg import servo_list

steer_channel = 0
throttle_channel = 1
max_left_steer = 460
center_steer=333
max_right_steer = 260
THROTTLE_FORWARD_PWM = 460      #pwm value for max forward throttle
THROTTLE_REVERSE_PWM = 270      #pwm value for max reverse throttle
THROTTLE_STOPPED_PWM = 370      #pwm value for no movement
steer_pulse = center_steer
throttle_pulse=THROTTLE_STOPPED_PWM
#pwm = Adafruit_PCA9685.PCA9685()

#pwm.set_pwm(channel, 0, pulse)
def set_steer_throttle():
    pwm.set_pwm(steer_channel, 0, steer_pulse)
    pwm.set_pwm(throttle_channel, 0, throttle_pulse)


def loop():
        rate = rospy.Rate(1) # 50Hz
        while not rospy.is_shutdown():
            #set_steer_throttle()
            print("steer_pulse")
            print(steer_pulse)
            print("throttle_pulse")
            print(throttle_pulse)
            
            
            
            rate.sleep()

def servo_updater_cb(msg):    
    global throttle_pulse
    global steer_pulse
    throttle_pulse = msg.linear.x
    steer_pulse = msg.angular.z
    rospy.loginfo("throttle : %d  steer : %d "%(throttle_pulse ,steer_pulse))

rospy.init_node('servo_setter')

rospy.Subscriber('/servo_cmd', Twist, servo_updater_cb)

loop()
