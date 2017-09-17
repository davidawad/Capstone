#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
import tf
import math
import copy

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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.pose = None
        self.waypoints = None
        self.traffic_waypoint = None
        self.current_velocity = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.wp_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('current_velocity', TwistStamped, self.velocity_cb )

#	rospy.Subscriber('/obstacle_waypoint', Int32, obstacle_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.loop()

    def loop(self):
        rate = rospy.Rate(0.5) # 1Hz
        while not rospy.is_shutdown():

            if (self.pose is not None) and (self.waypoints is not None):
                self.update_final_waypoints()
                self.publish_final_waypoints()

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
        self.traffic_waypoint = msg

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
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance_2d(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)


    def closest_waypoint(self, position):
        closest_len = 100000
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

    # def get_frenet_s(self, position, theta):
    # 	next_wp = self.next_waypoint(position, theta)
    # 	prev_wp = next_wp-1

    # 	if prev_wp < 0:
    # 	    prev_wp += len(self.waypoints)

    # 	n_x = self.waypoints[next_wp].pose.pose.position.x-self.waypoints[prev_wp].pose.pose.position.x
    # 	n_y = self.waypoints[next_wp].pose.pose.position.y-self.waypoints[prev_wp].pose.pose.position.y
    # 	n_z = self.waypoints[next_wp].pose.pose.position.z-self.waypoints[prev_wp].pose.pose.position.z
    # 	x_x = position.x - self.waypoints[prev_wp].pose.pose.position.x
    # 	x_y = position.y - self.waypoints[prev_wp].pose.pose.position.y
    # 	x_z = position.z - self.waypoints[prev_wp].pose.pose.position.z
    # 	#  find the projection of x onto n
    #     proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y+n_z*n_z)

    # 	p_proj = Point()
    #     p_proj.x = proj_norm*n_x
    #     p_proj.y = proj_norm*n_y
    # 	p_proj.z = proj_norm.n_z
    # 	p0 = Point()
    # 	p0.x = 0
    # 	p0.y = 0
    # 	p0.z = 0

    #     frenet_s = self.wp_distance(0, prev_wp)
    #     frenet_s += self.dl(p0, p_proj)
    #     return frenet_s

    def get_current_yaw(self):
        orientation = [ 
            self.pose.pose.orientation.x, 
            self.pose.pose.orientation.y, 
            self.pose.pose.orientation.z, 
            self.pose.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(orientation)
        return euler[2]   # z direction

    def update_final_waypoints(self):

        theta = self.get_current_yaw()
        index = self.next_waypoint( self.pose.pose.position, theta )
        self.final_waypoints = []
        len1 = len(self.waypoints)
        for i in range(LOOKAHEAD_WPS):
            wp = (i+index) % len1
            waypoint = copy.deepcopy(self.waypoints[wp])
            self.final_waypoints.append(waypoint)


        rospy.loginfo('final_waypoint index:%s', index)

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
