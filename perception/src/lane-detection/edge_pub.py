#!/usr/bin/env python3
import rospy
import cv2
import pickle
import rospkg
import numpy as np

from line import Line
from sensor_msgs.msg import Image
from edge_detection import *
from helper import *
from perception.msg import Lanes

pub = None

def applyLaneDetection(frame, save_path=None, file_name="out"):
    
    # cv2 read frame as BGR, convert it to RGB
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # camera calibration
    if not (mtx is None or dist is None):
        frame = cv2.undistort(frame, mtx, dist, None, mtx)

    # get edges in image
    edges = apply_edge_detection(frame)

    #cv2.imshow("warped", frame)
    # transform image to bird view
    warped = warped_img(edges, M)
        

    # if line not detected, apply sliding window
    if not left_line.detected or not right_line.detected:
        leftx, lefty, rightx, righty = sliding_window(warped, 9, 200)

    # if already detected apply search around detected line
    else:
        leftx, lefty = search_around_poly(left_line, warped)
        rightx, righty = search_around_poly(right_line, warped)

    # will used for plotting line, find x fitted
    ploty = np.linspace(warped.shape[0] // 4, warped.shape[0] - 1, warped.shape[0])

    # check if at least 100 pixels detected as line
    if len(leftx) > 100 and len(rightx) > 100:

        # make detected flag true
        left_line.detected = True
        right_line.detected = True

        left_line.current_x = leftx
        left_line.current_y = lefty

        right_line.current_x = rightx
        right_line.current_y = righty

        left_line.fit_polynomial(ploty)
        right_line.fit_polynomial(ploty)

    else:
        print("Line not detected in this frame ")
        # we just draw line form previous frame

        # make detected flag true
        left_line.detected = False
        right_line.detected = False

    # update Lane line radius
    left_line.radius()
    right_line.radius()

    # avg radius of two lines, and plot it
    radius = (left_line.radius_of_curvature + right_line.radius_of_curvature) // 2
    if save:
        frame = write_text(frame, "Radius of Curvature = " + str(radius) + " M", pos=(20, 50))

    # calculate Alignment ( how much car away from center between Lane lines
    left_line.car_offset(frame.shape)  # distance from left line
    right_line.car_offset(frame.shape)  # distance from right line

    distance = round(left_line.line_base_pos - right_line.line_base_pos, 2)
    
    if save:
        # init out image which will draw lane line on it then weight it with original frame
        out_img = np.zeros_like(warped)
        if len(warped.shape) == 3 and warped.shape[2] == 3:
            pass
        else:
            out_img = np.dstack((out_img, out_img, out_img))

        #file_name += ".mp4"
        #out = cv2.VideoWriter(save_path + file_name, -1, 20, (int(cap.get(3)), int(cap.get(4))))

        dir = "Right"  # car far from left or right
        if distance < 0:  # car far away from right line not left line
            distance = -distance
            dir = "Left"
        frame = write_text(frame, "Vehicle is {}m {} of center".format(distance, dir), pos=(20, 80))

        # ** plot lane lines on image **
        # left_line.draw_line(out_img, ploty)
        # right_line.draw_line(out_img, ploty)

        # color pixel which belong to lane lines
        left_line.color_pixel(out_img, (255, 0, 0))
        right_line.color_pixel(out_img, (255, 100, 0))

        # fit green triangle in area between lane lines
        pts_left = np.array([np.transpose(np.vstack([left_line.bestx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_line.bestx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        # Draw the lane onto the warped blank image
        cv2.fillPoly(out_img, np.int_([pts]), (0, 255, 0))

        # return image to normal view from bird view
        out_img_undit = warped_img(out_img, Minv)

        # weight out_image_undit with original frame
        frame = cv2.addWeighted(out_img_undit, 0.5, frame, 1, 0)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        # fot test
        cv2.imshow("frame",frame)
        cv2.waitKey(1)

        # write video
        #out.write(frame)
    return radius, distance

def callback(data):
    # convert ros image to normal image
    ros_img = data.data
    img = pickle.loads(ros_img)
    print(img.shape)
    cv2.imshow("test",img)
    
    #radius, distance = applyLaneDetection(img)

    #msg = Lanes()
    #msg.header.seq= data.header.seq
    #msg.header.stamp = rospy.get_rostime()
    #msg.header.frame_id=str(data.header.seq)
    #msg.radius.data = float(radius)
    #msg.displacement.data = float(distance)
    #pub.publish(msg)

def listener():
    global pub
    rospy.init_node('perception_lanes_node', anonymous=False)
    pub = rospy.Publisher('perception/lanes_topic', Lanes )
    rospy.Subscriber('sensors/camera_topic',Image,callback)
    cv2.waitKey(1)
  
    #closing all open windows  
    cv2.destroyAllWindows()
    rospy.loginfo("Lane Detection started..")
    rospy.spin()


if __name__ == "__main__":

    # camera calibration parameters [ mtx , dist]
    mtx = None
    dist = None
    out = None
    save = True

    rospack = rospkg.RosPack()
    if save:
        save_path = rospack.get_path('perception') + "/src/lane-detection"
        file_name = "test_res"
        if not os.path.isdir(save_path):
            raise FileNotFoundError(save_path, " Not Exist!")
        file_name += ".mp4"
        #out = cv2.VideoWriter(save_path + file_name, -1, 20, (int(cap.get(3)), int(cap.get(4))))
    path = rospack.get_path('perception')+"/src/lane-detection/perspective_tf.pickle"
    """
    with open(path, "rb") as p:
        perspective_matrix = pickle.load(p)
        M = perspective_matrix["M"]
        Minv = perspective_matrix["Minv"]
    """
    src = np.float32([[150,350],[430,350],[640,480],[0,480]])
    dst = np.float32([[0,0],[640,0],[640,480],[0,480]])
    
    M,Minv = perspective_tf(src,dst)
    #TODO:
    # * calibrate camera ans save its Matrix then use it in commented code below.
    # * if we want to save visulization of output = finish save video code.


    #if cam_cal:
    #    if not os.path.isfile(cam_cal):
    #        raise FileNotFoundError(cam_cal, " Not Exist!")
    #
    #    with open(cam_cal, "rb") as p:
    #        calibration = pickle.load(p)
    #        mtx = calibration["mtx"]
    #        dist = calibration["dist"]

    left_line = Line(5)
    right_line = Line(5)
    listener()
    