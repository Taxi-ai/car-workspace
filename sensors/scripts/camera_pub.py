#!/usr/bin/env python3
import rospy
import cv2
import pickle

#from sensors.msg import Image 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# from  sensor_msgs import Image
from time import time
# import numpy as np
seq=0



def camera_capture(video):   
    # t1=time()
    check, frame = video.read()    
    # t2=time()
    # print(t2-t1)

    return frame

def camera_node():
    global seq
    pub = rospy.Publisher('camera_topic', Image , queue_size=0) 
    rospy.init_node('camera_node', anonymous=False)
    rate = rospy.Rate(8) # 10hz
    bridge = CvBridge()
    #msg = Image()
    #msg.header.stamp= rospy.Time.now()
    video = cv2.VideoCapture(0)
    # video.set(3,1024) # 1024/4 horizontal pixels
    # video.set(4,768) # 768/4vertical pixels
    #video.set(3,176) # 1024/4 horizontal pixels
    #video.set(4,144) # 768/4vertical pixels
    # video.set(3,240) # 1024/4 horizontal pixels
    # video.set(4,160) # 768/4vertical pixels
    count = 0
    # t_last = time()
    while not rospy.is_shutdown():
        # wt1=time()
        check, frame = video.read()
        # frame = cv2.resize(frame[20:80,:],(200,88))
        # frame = frame.astype(np.float32)
        # frame = np.multiply(frame, 1.0 / 255.0)
        #msg.header.stamp = rospy.get_rostime()
        #msg.header.stamp.nsecs=0
        #msg.header.seq= seq
        #msg.header.frame_id=str(seq)
        #seq +=1
        #msg.image = pickle.dumps(frame,protocol=2)
        msg = cv2_to_imgmsg(frame,"bgr8")
        # msg.image = pickle.dumps(frame)
        # rospy.loginfo("img published")
        pub.publish(msg)
        # wt2=time()
        # print(wt2-wt1)
        rate.sleep()
        # print(wt2-wt1)
        # cv2.imshow("asa",frame)
        # cv2.waitKey(10)
        # cv2.destroyAllWindows()
    #video.release()
    
if __name__ == '__main__':
    try:
        print("started")
        camera_node()
    except rospy.ROSInterruptException:
        pass
