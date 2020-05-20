#!/usr/bin/python3.5
import rospy
from std_msgs.msg import String, Float32
from sensors.msg import Imu
import time
import board
import busio
import adafruit_bno055
import math  


import serial
#uart = serial.Serial("/dev/serial0")
#sensor = adafruit_bno055.BNO055_UART(uart)

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055(i2c)  

calibaration= False
linearX=0

xPos = 0
yPos = 0
headingVel = 0;
BNO055_SAMPLERATE_DELAY_MS = 10; #how often to read data from the board

#velocity = accel*dt (dt in seconds)
#position = 0.5*accel*dt^2
ACCEL_VEL_TRANSITION =  (float)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
DEG_2_RAD = 0.01745329251; #trig functions require radians, BNO055 outputs degrees

def talker():
    global linearX, xPos, yPos
    pub = rospy.Publisher('imu_topic', Imu, queue_size=10)
    rospy.init_node('imu_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg=Imu()
    while not rospy.is_shutdown():
        time1= time.time()
       
        #msg.yaw=float("{0:.3f}".format(sensor.gyroscope[2])) #rad/s Rotation around Z-axis
        x_ori=float("{0:.3f}".format(sensor.euler[0])) #rad/s x orintation
        xPos = float(xPos + ACCEL_POS_TRANSITION * sensor.acceleration[0]) # x acc
        yPos = float(yPos + ACCEL_POS_TRANSITION * sensor.acceleration[1]) # y acc
        print("x ori" + str(x_ori))
        # velocity of sensor in the direction it's facing
        headingVel = (ACCEL_VEL_TRANSITION * sensor.acceleration[0] / math.cos(DEG_2_RAD * sensor.euler[0])) * 100
        #aX= sensor.linear_acceleration[0]
        #time2= time.time()
        #dT= time2-time1
        #linearX += dT * aX #V = v_node + a*t
        msg.speed=float("{0:.3f}".format(headingVel))
        msg.header.stamp = rospy.get_rostime()
        #output = (linearX, angularZ)
        msg.yaw = x_ori
        msg.pose_x = xPos
        msg.pose_y = yPos
        msg.speed = headingVel
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
