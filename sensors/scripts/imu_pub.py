#!/usr/bin/python3.5
import rospy
from std_msgs.msg import String, Float32
from sensors_pkg.msg import Imu
import time
import board
import busio
import adafruit_bno055


import serial
uart = serial.Serial("/dev/serial0")
sensor = adafruit_bno055.BNO055_UART(uart)

#i2c = busio.I2C(board.SCL, board.SDA)
#sensor = adafruit_bno055.BNO055(i2c)  

calibaration= False
linearX=0

def talker():
    global linearX
    pub = rospy.Publisher('imu_topic', Imu, queue_size=10)
    rospy.init_node('imu_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg=Imu()
    while not rospy.is_shutdown():
        time1= time.time()
       
        msg.angularZ=float("{0:.3f}".format(sensor.gyroscope[2])) #rad/s Rotation around Z-axis
        
        aX= sensor.linear_acceleration[0]
        time2= time.time()
        dT= time2-time1
        linearX += dT * aX #V = v_node + a*t
        msg.linearX=float("{0:.3f}".format(linearX))
        #output = (linearX, angularZ)

        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass