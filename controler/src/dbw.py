#!/usr/bin/env python

import rospy
from sensors.msg import Imu
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist,PoseArray
from geometry_msgs.msg import TwistStamped
import math
import cutils
import numpy as np



#v=0.12
way_list=[[-181.3353216786993, 80.53986286885691, 1.5],[-181.3868905774473, 56.897194181192845, 3.8642730118254875],[-181.3353216786993, 80.53986286885691, 1.5],[-181.3868905774473, 56.897194181192845, 3.8642730118254875],[-181.39365660987602, 39.819383248956, 5.572054365851686],[-181.3353216786993, 80.53986286885691, 1.5],[-181.3353216786993, 80.53986286885691, 1.5]]

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')
      
        self.twist_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)

        self.vars                = cutils.CUtils()
        self._current_x          = 0 #data.pose_x
        self._current_y          = 0 #data.pose_y
        self._current_yaw        = 0 #data.yaw
        self._current_speed      = 0 #data.speed
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = True
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = []#way_list 
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi
        

        # TODO: Create `TwistController` object
        
        # TODO: Subscribe to all the topics you need to
        #rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/final_waypoints', PoseArray, self.update_waypoints_cb)
        rospy.Subscriber('/imu_topic', Imu, self.current_pose_cb)
        #rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)

        # go into event loop
        self.loop()
    
    def current_pose_cb(self,imu_poses):

        self._current_x          =  imu_poses.pose_x
        self._current_y          = imu_poses.pose_y
        self._current_yaw        =  imu_poses.yaw
        self._current_speed      = imu_poses.speed
        self._current_timestamp  =imu_poses.header.timestamp



    def update_waypoints_cb(self, new_waypoints):
        for p in new_waypoints.poses:
        	print("x: " + str(p.position.x) + "y: " + str(p.position.y))
        	self._waypoints.append([p.position.x,p.position.y,0.12])
        self._waypoints[-1][2]= 0

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake
    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_controls(self):
        ######################################################
        # RETRIEVE car FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
       
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('t_previous', 0.0)
        self.vars.create_var('throttle_previous', 0.0)
        self.vars.create_var('int_val', 0.0)
        self.vars.create_var('last_error', 0.0)
        ## pid gains 
        kp = 1
        ki = 1
        kd = 0.01

        if self._start_control_loop:

            ## longitudinal controller
            throttle_output = 0
            brake_output    = 0

            # pid control
            st = t - self.vars.t_previous

            # error term
            delta_v = v_desired - v

            # I
            integral = self.vars.int_val + delta_v * st

            # D
            derivate = (delta_v - self.vars.last_error) /st

            rst = kp * delta_v + ki * integral + kd * derivate

            if rst > 0:
                throttle_output = np.tanh(rst)
                throttle_output = max(0.0, min(1.0, throttle_output))
                if throttle_output - self.vars.throttle_previous > 0.1:
                    throttle_output = self.vars.throttle_previous + 0.1
            else:
                throttle_output = 0
            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
           # Change the steer output with the lateral controller. 
            steer_output = 0

            # Use stanley controller for lateral control
            # 0. spectify stanley params
            k_e = 0.3
            k_v = 10

            # 1. calculate heading error
            #(Y_n -Y1) /(X_n-X_1)
            yaw_path = np.arctan2(waypoints[-1][1]-waypoints[0][1], waypoints[-1][0]-waypoints[0][0])
            yaw_diff = yaw_path - yaw 
            if yaw_diff > np.pi:
                yaw_diff -= 2 * np.pi
            if yaw_diff < - np.pi:
                yaw_diff += 2 * np.pi

            # 2. calculate crosstrack error
            current_xy = np.array([x, y])
            crosstrack_error = np.min(np.sum((current_xy - np.array(waypoints)[:, :2])**2, axis=1))

            yaw_cross_track = np.arctan2(y-waypoints[0][1], x-waypoints[0][0])
            yaw_path2ct = yaw_path - yaw_cross_track
            if yaw_path2ct > np.pi:
                yaw_path2ct -= 2 * np.pi
            if yaw_path2ct < - np.pi:
                yaw_path2ct += 2 * np.pi
            if yaw_path2ct > 0:
                crosstrack_error = abs(crosstrack_error)
            else:
                crosstrack_error = - abs(crosstrack_error)

            yaw_diff_crosstrack = np.arctan(k_e * crosstrack_error / (k_v + v))
            
            print(crosstrack_error, yaw_diff, yaw_diff_crosstrack)

            # 3. control low
            steer_expect = yaw_diff + yaw_diff_crosstrack
            if steer_expect > np.pi:
                steer_expect -= 2 * np.pi
            if steer_expect < - np.pi:
                steer_expect += 2 * np.pi
            steer_expect = min(1.22, steer_expect)
            steer_expect = max(-1.22, steer_expect)

            # 4. update
            steer_output = steer_expect
            rospy.loginfo("steer:%f ",steer_output)
            #throttle_output =self._waypoints[]
            rospy.loginfo("throttle_output:%f ",throttle_output)
            #steer_output = 0.05

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            #self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.t_previous = t
        self.vars.int_val = integral
        self.vars.throttle_previous = throttle_output
    def loop(self):
        rate = rospy.Rate(1) # 50Hz
        it=0
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            self.update_controls()
            throttle,steering,brake = self.get_commands()
            it=it+1
            
            rospy.loginfo("pub %d  steer:%f "%(it ,steering))
            
            self.publish(throttle, brake, steering)
            rate.sleep()

    

    

    def twist_cb(self, msg):
        self.twist_cmd = msg
        self.linear_velocity = self.twist_cmd.twist.linear.x
        self.angular_velocity = self.twist_cmd.twist.angular.z
        print "twist_cmd:", msg
        print ""

    def publish(self, throttle , brake, steer):
        twist = Twist()
        twist.linear.x = 0.25; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = steer
        self.twist_pub.publish(twist)
       


if __name__ == '__main__':
	DBWNode()
