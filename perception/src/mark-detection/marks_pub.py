#!/usr/bin/env python3
import rospy
import cv2
import pickle
import rospkg
import time
import numpy as np

from std_msgs.msg import String
from std_msgs.msg import Int16 as Int
from sensor_msgs.msg import Image

# ******** for test ***********
# Import the PCA9685 module.
import Adafruit_PCA9685


def perspective_tf(src, dst):
    """
    apply perspective transform on img using src and dist points
    :param src: list of src point
    :param dst: list of destination point which will transform src to it
    :return: M => transformed transform matrix, Minv => inverse transform matrix
    """
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)

    return M, Minv

def warped_img(img, M):
    """
    apply perspective transform on given img using given matrix
    :param img: image want to apply transform on it
    :param M: perspective TF matrix
    :return: transformed image
    """
    img_size = (img.shape[1], img.shape[0])
    warped = cv2.warpPerspective(img, M, img_size)
    return warped

def detectPoint(img):
  global passedPoint, PREVE_FRAM_ID, CURR_FRAM_ID
  CURR_FRAM_ID +=1
  img = warped_img(img,M)
  gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  gray = cv2.blur(gray,(5,5))
  gray = 255 - gray
  #cv2.imshow("warped",img)
  _ , threshold_img = cv2.threshold(gray,THRES_MIN,THRES_MAX,cv2.THRESH_BINARY)
  contours, h = cv2.findContours(threshold_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
  
  if contours is not None:
    for cnt in contours:
      area = cv2.arcLength(cnt,True)
      apprx = cv2.approxPolyDP(cnt,0.02 * area,True)
      # try to make sure that contour is our circle 
      if len(apprx) == POINT_NUM_THRESH and area < AREA_THRESH_MAX and area > AREA_THRESH_MIN:
        # to avoid count same point in every frame!
        if passedPoint == 0:
          passedPoint += 1
          
        elif CURR_FRAM_ID - PREVE_FRAM_ID > FRAM_THRES:
          passedPoint += 1
    
        PREVE_FRAM_ID = CURR_FRAM_ID
      """
      cv2.drawContours(img,[apprx],0,(0,255,0),3)
      cv2.imshow("window",img)
      cv2.waitKey(1)
      if cv2.waitKey(1) & 0xFF == ord('q'):
          break
      """
def callback(data):
    global PULS
    # convert ros image to normal image
    ros_img = data.data
    img = pickle.loads(ros_img)
    detectPoint(img)
    pub.publish(passedPoint)
    cmd = ""
    if passedPoint == path_points:
      cmd = "stop"
      PULS = 300
      pwm.set_pwm(1,0,PULS)
      PULS = 370
    elif passedPoint < path_points:
      cmd = "keep"
      PULS = 450
    else:
      rospy.loginfo("ERROR Passed points > path points!")
    pwm.set_pwm(1,0,PULS)
    st = "CMD : " + cmd
    rospy.loginfo(st)
    st = "PULS = " + str(PULS)
    rospy.loginfo(st)
    rospy.loginfo("passed Points %d",passedPoint)
    
if __name__ == "__main__":

  """ Init some params """
  TEST = True
  M = None 
  Minv = None
  passedPoint = 0
  THRES_MIN = 80
  THRES_MAX = 150
  AREA_THRESH_MAX = 900
  AREA_THRESH_MIN = 700
  FRAM_THRES = 10
  POINT_NUM_THRESH = 8 # approx conturs points 
  PREVE_FRAM_ID = 0
  CURR_FRAM_ID = 0
  path_points = 0
  
  # ********** for test *********
  pwm = Adafruit_PCA9685.PCA9685()
  PULS = 370
  pwm.set_pwm(1,0,PULS) # init car
  # *******************************
  
  rospy.init_node('perception_points_node', anonymous=False)
  pub = rospy.Publisher('perception/points_topic', Int )
  pub = rospy.Publisher('planning/cmd', String )
  rospy.Subscriber('sensors/camera_topic',Image,callback)
  rospy.loginfo("point-detection Detection started..")
  
  # to generate M and Minv --- > note point = (x,y) not (y,x) && img.shape return y,x (height , width)
  src = np.float32([[150,350],[430,350],[640,480],[0,480]])
  dst = np.float32([[0,0],[640,0],[640,480],[0,480]])
  
  if TEST:
    path_points = 3
  
  if M is None:
    M , Minv = perspective_tf(src, dst)
    
  rospy.spin()