#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane
import math
import tf

from yaw_controller import YawController
from twist_controller import Controller

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

# Globals
SET_SPEED = 10 # mph
MIN_SPEED = 0.0

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')
    	self.dbw_enabled = True  ## for simulator always True
        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)


        # Publish to Topics
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        # Member Variables
    	self.count = 0
        self.twist_cmd = None
        self.velocity = None
        self.final_wps = None
        
        
        # Controllers
        self.yaw_controller = YawController(wheel_base, steer_ratio * 8, MIN_SPEED, max_lat_accel, max_steer_angle)
        self.twist_controller = Controller(vehicle_mass, wheel_radius, decel_limit, accel_limit, max_steer_angle)

        # Subscribe to Topics
    	rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb, queue_size = 1) 
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb, queue_size = 1)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size = 1)
        rospy.Subscriber('current_velocity', TwistStamped, self.velocity_cb, queue_size = 1)
        rospy.Subscriber('/final_waypoints',Lane, self.final_wp, queue_size = 1)



        self.loop()

    def loop(self):
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            if (self.twist_cmd is not None) and (self.velocity is not None) and self.dbw_enabled and self.final_wps:

                # Knowns necassary for calculations

                # Known velocities
                target_velocity = self.twist_cmd.linear.x
                angular_velocity = self.twist_cmd.angular.z
                current_velocity = self.velocity.linear.x
                
                # Known yaw angles 
                #_, _, current_yaw = tf.transformations.euler_from_quaternion([self.pose.orientation.x, 
                #                                                              self.pose.orientation.y, 
                #                                                              self.pose.orientation.z, 
                #                                                              self.pose.orientation.w])

                #_, _, target_yaw = tf.transformations.euler_from_quaternion([self.final_wps[0].pose.pose.orientation.x, 
                #                                                             self.final_wps[0].pose.pose.orientation.y, 
                #                                                             self.final_wps[0].pose.pose.orientation.z, 
                #                                                             self.final_wps[0].pose.pose.orientation.w])                  

                cte = angular_velocity

                # Call Controllers to calculate and return variables to be controlled
                steering_yaw = self.yaw_controller.get_steering( target_velocity, angular_velocity, current_velocity)
                throttle, brake, steering_pid = self.twist_controller.control(current_velocity, target_velocity, cte, self.dbw_enabled)

                steering = steering_pid


                # Publish Control Variables
            	self.publish(throttle, brake, cte, target_velocity)

            rate.sleep()

    def publish(self, throttle, brake, steer, target_velocity):
        
        self.count += 1

        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

        rospy.loginfo('Throttle: %s Brake: %s Steer: %s, Target_v: %s, Count: %s', throttle, brake, steer, target_velocity, self.count)

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data

    def twist_cmd_cb(self, msg):
        self.twist_cmd = msg.twist

    def pose_cb(self, msg):
        self.pose = msg.pose

    def velocity_cb(self, msg): # geometry_msgs/TwistStamped
        self.velocity = msg.twist
    
    def final_wp(self, msg):
        self.final_wps = msg.waypoints

if __name__ == '__main__':
    DBWNode()
