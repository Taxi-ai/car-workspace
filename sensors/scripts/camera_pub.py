#!/usr/bin/env python3
import rospy
import cv2
import pickle

from sensor_msgs.msg import Image
from time import time

def camera_node():
    rospy.init_node('camera_node', anonymous=False)
    pub = rospy.Publisher('sensors/camera_topic', Image , queue_size=0)  
    rospy.loginfo("camera publisher started.")
    
    rate = rospy.Rate(10) # 10hz
    cam = cv2.VideoCapture(0)
    seq = 0
    while not rospy.is_shutdown():
        check, frame = cam.read()

        msg = Image()
        msg.header.stamp = rospy.get_rostime()
        msg.header.seq= seq
        msg.header.frame_id=str(seq)
        msg.data = pickle.dumps(frame,protocol=2)
        pub.publish(msg)
        seq +=1

        rate.sleep()
        
    
if __name__ == '__main__':
    try:
        camera_node()
    except rospy.ROSInterruptException:
        pass
