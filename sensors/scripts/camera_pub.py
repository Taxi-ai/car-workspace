#!/usr/bin/env python3
import rospy
import cv2
import pickle
import os
import rospkg

from sensor_msgs.msg import Image
from time import time

def camera_node():
    rospy.init_node('camera_node', anonymous=False)
    pub = rospy.Publisher('sensors/camera_topic', Image , queue_size=0)  
    rospy.loginfo("camera publisher started.")
    
    rate = rospy.Rate(10) # 10hz
    if test:
        rospack = rospkg.RosPack()
        test_video_path = rospack.get_path('sensors') + "/../test-videos/lane.avi"
        if not os.path.isfile(test_video_path):
                raise FileNotFoundError(test_video_path, " not Exist!")
        cam = cv2.VideoCapture(test_video_path) # for test
    else:
        cam = cv2.VideoCapture(0)
    
    seq = 0
    while not rospy.is_shutdown():
        check, frame = cam.read()
        msg = Image()
        msg.header.stamp = rospy.get_rostime()
        msg.header.seq= seq
        msg.header.frame_id=str(seq)
        msg.width = frame.shape[0]
        msg.height = frame.shape[1]
        msg.data = pickle.dumps(frame,protocol=2)
        pub.publish(msg)
        seq +=1
        rate.sleep()
        if test:
            cv2.imshow("original",frame)
            cv2.waitKey(0)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    # When everything done, release the capture
    cam.release()
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    test = True
    try:
        camera_node()
    except rospy.ROSInterruptException:
        pass
