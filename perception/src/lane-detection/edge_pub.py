#!/usr/bin/env python3
import rospy
import cv2
import pickle
import rospkg
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import String

from edge_detection import LaneDetection
from helper import perspective_tf
from line import Line

pub = None


def callback(data):
    # convert ros image to normal image
    ros_img = data.data
    img = pickle.loads(ros_img)

    lane_detector(img)
    if draw and lane_detector.threshold_img is not None:
        cv2.imshow("Binary Image",lane_detector.warped)
        lane_detector.drawLanes()
        cv2.imshow("lane lines",lane_detector.img)
        cv2.waitKey(1)
        

def talker():
    while not rospy.is_shutdown():
        #rospy.loginfo("published direction is " + str(lane_detector.turn_dir))
        pub.publish(lane_detector.turn_dir)
        
        rate.sleep()


if __name__ == "__main__":

    draw = True

    src = np.float32([[50, 300], [590, 300], [640, 480], [0, 480]])
    dst = np.float32([[0, 0], [640, 0], [640, 480], [0, 480]])

    M, Minv = perspective_tf(src, dst)

    # init lane lines obj and lane detection obj
    left_line = Line(5)
    right_line = Line(5)
    lane_detector = LaneDetection(M, Minv, left_line, right_line)

    # init node, cam sub and lane pub
    rospack = rospkg.RosPack()
    rospy.init_node('perception_lanes_node', anonymous=False)
    rate = rospy.Rate(10)  # 10hz

    rospy.Subscriber('sensors/camera_topic', Image, callback)
    pub = rospy.Publisher('perception/lanes_topic', String)

    rospy.loginfo("Lane Detection started..")
    talker()

    # closing all open windows
    cv2.destroyAllWindows()
    

    # TODO:
    # * calibrate camera ans save its Matrix then use it in commented code below.
    # * if we want to save visulization of output = finish save video code.
    # if cam_cal:
    #    if not os.path.isfile(cam_cal):
    #        raise FileNotFoundError(cam_cal, " Not Exist!")
    #
    #    with open(cam_cal, "rb") as p:
    #        calibration = pickle.load(p)
    #        mtx = calibration["mtx"]
    #        dist = calibration["dist"]
