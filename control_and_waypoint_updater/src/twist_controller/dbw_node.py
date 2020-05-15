#!/usr/bin/env python
import numpy as np
import cutils
from sympy import integrate
import math
import rospy
from std_msgs.msg import Bool
##from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
import math

#from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.
You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.
One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.
We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.
We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.
Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.
'''
RATE_OF_CONTROL = 50
k = 0.1         #look forward gain
Lfc = 1.0       #look-ahead distance
L = 2.9


class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        

        self.twist_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=5)

        #self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',SteeringCmd, queue_size=1)
        #self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',ThrottleCmd, queue_size=1)
        #self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',BrakeCmd, queue_size=1)

        

        # TODO: Subscribe to all the topics you need to
        #rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/final_waypoints', TwistStamped, self.waypoint_cb)
        #rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
		#rospy.Subscriber('/current_pose', TwistStamped, self.velocity_cb)

       # self.current_vel = None
       # self.current_ang_vel = None
        self.dbw_enabled = True
       # self.linear_vel = None
        #self.angular_vel = None
        #self.throttle = 0
	#self.steering = 0
	#self.brake = 0

		self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = True
        self._set_throttle       = 0.1
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

	
        self.loop()

    def loop(self):
        rate = rospy.Rate(RATE_OF_CONTROL) # 50Hz
        while not rospy.is_shutdown():
        	if self._start_control_loop:
        		length = np.arange(0,100,1)
            	dx = [self._current_x - waypoints[icx][0] for icx in length]
            	dy = [self._current_y - waypoints[icy][1] for icy in length]
            	d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx,idy) in zip(dx,dy)]
            	ind = d.index(min(d))
            	if ind < 2:
                	tx = waypoints[ind][0]
                	ty = waypoints[ind][1]  
            	else:
                	tx = waypoints[-1][0]
                	ty = waypoints[-1][1]
                	# ind = len(self._current_x) - 1    

            	alpha_hat = math.atan2(ty - y,tx - x)
            	alpha = alpha_hat - yaw
            	Lf = k * v + Lfc
            	steer_output = math.atan2(2.0 * L * math.sin(alpha) / Lf,1.0)
            	print("steer_output = ",steer_output)

            	######################################################
            	# SET CONTROLS OUTPUT
            	######################################################
            	#self.set_throttle(throttle_output)  # in percent (0 to 1)
            	self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            	#self.set_brake(brake_output)        # in percent (0 to 1)        
          
           
            
			rate.sleep()

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = True

    def update_waypoints_cb(self, new_waypoints):
        self._waypoints = new_waypoints

    def velocity_cb(self, msg):
        self.current_vel = msg.twist.linear.x


    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer


    def publish(self, throttle, brake, steer):
	twist = Twist()
        twist.linear.x = throttle; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = steer
        self.twist_pub.publish(twist)
	'''
        twist = Twist()
        if not brake:

            twist.linear.x = throttle; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = steer
            self.twist_pub.publish(twist)
        else :
            twist.linear.x = brake; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = steer
            self.twist_pub.publish(twist)
'''
        '''
        twist = Twist()
        twist.linear.x = throttle; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        self.throttle_pub.publish(twist)
        twist = Twist()
        twist.linear.x = throttle; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = steer
        self.steer_pub.publish(twist)
        twist = Twist()
        twist.linear.x = brake; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = steer
        self.brake_pub.publish(twist)
        '''


if __name__ == '__main__':
    DBWNode()
