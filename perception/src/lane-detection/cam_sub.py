#!/usr/bin/env python3
import rospy
import cv2
import pickle

#from sensors.msg import Image 
from sensor_msgs.msg import Image


def imgCallback(data):
    rospy.loginfo("I heard image")
    rosImg = data.data 
    img = pickle.loads(rosImg)
    print("image : ", img.shape)

def listener():
    rospy.init_node('camSub')
    rospy.Subscriber('sensors/camera_topic',Image,imgCallback)

    rospy.spin()


listener()