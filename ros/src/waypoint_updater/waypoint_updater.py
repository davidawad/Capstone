#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
import tf
import math
import copy
import numpy as np
from scipy import interpolate
'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_BRAKE_DISTANCE = 60 # distance to start brake 

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.pose = None
        self.waypoints = None
        self.traffic_waypoint = -1
        self.current_velocity = None
        self.state = 0 

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.wp_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('current_velocity', TwistStamped, self.velocity_cb )

#	rospy.Subscriber('/obstacle_waypoint', Int32, obstacle_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.final_waypoints_index_pub = rospy.Publisher('final_index', Int32, queue_size=1)

        # TODO: Add other member variables you need below
        # self.published = False
        self.loop()

    def loop(self):
        rate = rospy.Rate(0.5) # 0.5Hz
        while not rospy.is_shutdown():

            if (self.pose is not None) and (self.waypoints is not None): # and not self.published:
                self.update_final_waypoints()
                self.publish_final_waypoints()

            # self.published = False
            rate.sleep()
        rospy.spin()

    def pose_cb(self, msg):
    	self.pose = msg
    	# rospy.loginfo('current_pos Received')

    def waypoints_cb(self, waypoints): # msg type Lane: Waypoints[] waypoints
        # TODO: Implement
    	rospy.loginfo('base_waypoints received - size:%s', len(waypoints.waypoints))
    	self.waypoints = waypoints.waypoints
        # only need once.  unregister it
        self.wp_sub.unregister()


    def traffic_cb(self, msg): ## msg type Int32
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_waypoint = msg.data
        if self.traffic_waypoint != -1:
            rospy.loginfo("traffic_waypoint: %s", msg)

        if ((self.state == 1) and (self.traffic_waypoint == -1)) or \
           ((self.state == 0) and (self.traffic_waypoint != -1)):
            if (self.pose is not None) and (self.waypoints is not None):
                self.update_final_waypoints()
                self.publish_final_waypoints()
                # self.published = True

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
    	self.obstacle_waypoint = msg

    def velocity_cb(self, msg): # geometry_msgs/TwistStamped
        self.current_velocity = msg.twist

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def wp_distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        len_waypoints = len(waypoints)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance_2d(self, a, b):
        return ((a.x-b.x)**2 + (a.y-b.y)**2)


    def closest_waypoint(self, position):
        closest_len = 1e6
        closest_index = 0
        for i in range(len(self.waypoints)):
            dist = self.distance_2d( position, self.waypoints[i].pose.pose.position)
            if dist < closest_len:
                closest_len = dist
                closest_index = i

        return closest_index

    def next_waypoint(self, position, theta):
    	index = self.closest_waypoint(position)
    	map_x = self.waypoints[index].pose.pose.position.x
    	map_y = self.waypoints[index].pose.pose.position.y

    	heading = math.atan2( map_y - position.y, map_x - position.x)
    	angle = math.fabs(theta-heading)
    	if angle > math.pi/4:
    	    index += 1

    	return index

    def get_current_yaw(self):
        orientation = [ 
            self.pose.pose.orientation.x, 
            self.pose.pose.orientation.y, 
            self.pose.pose.orientation.z, 
            self.pose.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(orientation)
        return euler[2]   # z direction

    def deceleration_waypoints(self, waypoints, stop_index, start_velocity):
        stop_distance = self.wp_distance(waypoints, 0, stop_index)
        # estimate distance per waypoint
        s0 = self.wp_distance(waypoints, 0, 1)
        if stop_distance > MAX_BRAKE_DISTANCE:
            x = np.array([  0, stop_distance-MAX_BRAKE_DISTANCE, stop_distance, stop_distance+s0, stop_distance+s0*2])
        else:
            x = np.array([-s0, 0, stop_distance, stop_distance+s0, stop_distance+s0*2])

        y = np.array([start_velocity, start_velocity, 0, 0, 0])
        tck = interpolate.splrep(x, y, s=0)
        xnew = 0
        for i in range(1, len(waypoints)):
            if ( i < stop_index):
                xnew += self.wp_distance(waypoints, i-1, i)
                ynew = interpolate.splev(xnew, tck, der=0)
                ynew = min(start_velocity, ynew)
                self.set_waypoint_velocity(waypoints, i, ynew)
            else:
                self.set_waypoint_velocity(waypoints, i, 0)

    def update_final_waypoints(self):

        theta = self.get_current_yaw()
        index = self.next_waypoint( self.pose.pose.position, theta )
        final_waypoints = []
        len1 = len(self.waypoints)
        for i in range(LOOKAHEAD_WPS):
            wp = (i+index) % len1
            waypoint = copy.deepcopy(self.waypoints[wp])
            final_waypoints.append(waypoint)

        if self.state ==0:
            if (self.traffic_waypoint != -1) and \
               (index < self.traffic_waypoint) and \
               ((self.traffic_waypoint - index) < LOOKAHEAD_WPS):

                start_velocity = self.get_waypoint_velocity(final_waypoints[0]) 
                self.deceleration_waypoints(final_waypoints, self.traffic_waypoint-index, start_velocity)
                self.decel_index = index
                self.decel_waypoints = copy.deepcopy(final_waypoints)
                self.state = 1

        elif self.state == 1:

            if (self.traffic_waypoint != -1):
                # reuse previous generate waypoints velocity
                for i in range(LOOKAHEAD_WPS):
                    offset_index = i + index - self.decel_index
                    if ( offset_index < LOOKAHEAD_WPS):
                        vel = self.get_waypoint_velocity(self.decel_waypoints[offset_index])
                        self.set_waypoint_velocity(final_waypoints, i, vel)
                    else:
                        self.set_waypoint_velocity(final_waypoints, i, 0.0)
            else:
                self.state = 0

        ## elif self.state == 2:

        self.final_waypoints = final_waypoints
        rospy.loginfo('final_waypoint index:%s', index)
        self.final_waypoints_index_pub.publish(Int32(index))

        
    def publish_final_waypoints(self):
    	msg = Lane()
        msg.header.stamp = rospy.Time(0)
    	msg.waypoints = self.final_waypoints
        # 
    	self.final_waypoints_pub.publish(msg)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
