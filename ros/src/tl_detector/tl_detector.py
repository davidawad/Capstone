#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import numpy as np
import os
from time import strftime
import math

STATE_COUNT_THRESHOLD = 3

def clamp(n, smallest, largest): 
    return max(smallest, min(n, largest))

def generate_training_data(dst_folder, img, state):
    if not os.path.isdir(dst_folder):
        os.makedirs(dst_folder)
    
    time = strftime("%Y%m%d_%H%M%S")
    res = cv2.resize(img, (64, 64) ,interpolation=cv2.INTER_CUBIC)
    filename = '{}/{}_state_{}.jpg'.format(dst_folder, time, state)
    cv2.imwrite(filename, res)

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = []
        self.camera_image = None
        self.lights = []

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.light_waypoints = []
        self.light_indexed = False

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_wp_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        self.light_sub = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        self.image_sub = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.stop_line_positions = self.config['stop_line_positions']

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)







        if self.config['debug_img']:
            self.debug_img_pub = rospy.Publisher('/debug_img', Image, 
                queue_size=1)

        self.light_waypoints = []
        self.light_indexed = False

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def build_light_waypoints(self):
        for light in self.lights:
            light_pos = light.pose.pose.position
            distances = []
            for i, wp in enumerate(self.waypoints):
                wp_pos = wp.pose.pose.position
                distance = (
                    (light_pos.x - wp_pos.x)**2
                    + (light_pos.y - wp_pos.y)**2
                    # + (light_pos.z - wp_pos.z)**2
                )
                distances.append((i, distance))
            distances.sort(key=lambda e: e[1])
            waypoint_indices = map(lambda e: e[0], distances)
            self.light_waypoints.append(waypoint_indices)
        self.light_indexed = True

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints
        self.base_wp_sub.unregister()

        if self.waypoints and self.lights and not self.light_indexed:
            self.build_light_waypoints()

    def traffic_cb(self, msg):
        self.lights = msg.lights

        if self.waypoints and self.lights and not self.light_indexed:
            self.build_light_waypoints()

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        if (self.waypoints is None):
            return

        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_light(self, pose):
        """
        Returns:
            int: index of the closest direct light

        """
        ego_pos = np.array([
            self.pose.pose.position.x,
            self.pose.pose.position.y
        ])

        rot_quat = np.array([
            self.pose.pose.orientation.x,
            self.pose.pose.orientation.y,
            self.pose.pose.orientation.z,
            self.pose.pose.orientation.w,
        ])

        rot = tf.transformations.quaternion_matrix(rot_quat)
        init_dir = np.array([1.0, 0., 0., 0.])
        ego_dir = np.matmul(rot, init_dir)[:2]
        # normalized
        ego_dir = ego_dir / np.linalg.norm(ego_dir)
        min_dist = 150
        candidates = []
        for i, light in enumerate(self.lights):
            light_pos = np.array([
                light.pose.pose.position.x,
                light.pose.pose.position.y])

            light_dir = light_pos - ego_pos
            # normalized
            light_dir = light_dir / np.linalg.norm(light_dir)
            # angle between light_dir & ego_dir
            cos_theta = np.dot(ego_dir, light_dir)
            distance = np.linalg.norm(ego_pos - light_pos)
            if distance < min_dist:
                candidates.append((i, cos_theta, distance))

        if candidates:
            # chose the smallest angle
            candidates.sort(key=lambda e: e[1])
            
            index, cos_theta, distance = candidates[0]
            
            if cos_theta > 0:
                return index
        else:
            return -1

    def project_to_image_plane(self, *points):
        """Project points from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        rot = None

        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        x = 0
        y = 0
        cx = image_width * 0.5
        cy = image_height * 0.5

        if trans is None or rot is None:
            return [(x, y) for _ in points ]

        camera_mat = self.listener.fromTranslationRotation(trans, rot)

        # if in simulator, use the tweak (not accurate)
        # since udacity can't provide the intrinsic parameters
        zc_offset = 0.
        if fx < 10.0:
            fx = 2574
            fy = 2744
            cx = image_width * 0.5 - 35.
            cy = image_height + 50
            zc_offset = -1.0

        result = []
        for p in points:
            # use homogenous coordinate
            point_in_world = np.array([p[0], p[1], p[2], 1.0])
            # point_in_world = np.array([p.x, p.y, p.z, 1.0])
            point_in_camera = np.matmul(camera_mat, point_in_world)
            xc, yc, zc, _ = point_in_camera
            zc += zc_offset

            x = int(- fx * yc / xc + cx)
            y = int(- fy * zc / xc + cy)
            result.append((x, y))
        
        return result

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        h, w, _ = cv_image.shape

        size = 1
        rot_quat = [
            self.pose.pose.orientation.x,
            self.pose.pose.orientation.y,
            self.pose.pose.orientation.z,
            self.pose.pose.orientation.w,
        ]
        rot = tf.transformations.quaternion_matrix(rot_quat)
        init_dir = np.array([1.0, 0., 0., 0.])
        ego_dir = np.matmul(rot, init_dir)[:3]
        ego_dir = ego_dir / np.linalg.norm(ego_dir)
        light_normal = -ego_dir
        up = np.array([0.,0.,1.])
        down = -up
        left = np.cross(light_normal, up)
        left = left / np.linalg.norm(left)
        right = -left
        light_center = [light.pose.pose.position.x, light.pose.pose.position.y, light.pose.pose.position.z]
        top_left = light_center + size * up + size * left
        bottom_right = light_center + size * down + size * right

        tl, br = self.project_to_image_plane(top_left, bottom_right)

        if tl[0] < w and tl[1] < h and br[0] > 0 and br[1] > 0:
            tl = (clamp(tl[0], 0, w), clamp(tl[1], 0, h))
            br = (clamp(br[0], 0, w), clamp(br[1], 0, h))
            
            if self.config['debug_img']:
                cv2.rectangle(cv_image, tl, br, (0, 255, 0), 2)
                debug_img = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                self.debug_img_pub.publish(debug_img)

            #TODO use light location to zoom in on traffic light in image
            roi = cv_image[tl[1]:br[1], tl[0]:br[0]]
            roi_h, roi_w, _ = roi.shape
            if roi_w > 0 and roi_h > 0:
                if self.config['generate_train']:
                    light_state = light.state

                    dst_folder = self.config['samples_folder']
                    generate_training_data(dst_folder, roi, light_state)
                    return light_state
                else:
                    test = cv2.resize(roi, (64, 64) ,interpolation=cv2.INTER_CUBIC)
                    # TODO classifier
                    #Get classification
                    tl_state = self.light_classifier.get_classification(test)
                    return tl_state

        return TrafficLight.UNKNOWN


    def get_light_wp(self, light_index):
        ego_pos = np.array([
            self.pose.pose.position.x,
            self.pose.pose.position.y
        ])

        rot_quat = np.array([
            self.pose.pose.orientation.x,
            self.pose.pose.orientation.y,
            self.pose.pose.orientation.z,
            self.pose.pose.orientation.w,
        ])

        rot = tf.transformations.quaternion_matrix(rot_quat)
        init_dir = np.array([1.0, 0., 0., 0.])
        ego_dir = np.matmul(rot, init_dir)[:2]
        # normalized
        ego_dir = ego_dir / np.linalg.norm(ego_dir)


        waypoint_indices = self.light_waypoints[light_index]
        wp_near_light = self.waypoints[waypoint_indices[0]].pose.pose.position

        # It's impossible the stop line coulde be further than 50 meters from the traffic light
        min_dist = 50
        target_stop_line = None
        for stop_line_pos in self.stop_line_positions:
            dist = math.sqrt(
                (wp_near_light.x - stop_line_pos[0]) ** 2
                + (wp_near_light.y - stop_line_pos[1]) ** 2
            )
            stop_line_nparray = np.array([stop_line_pos[0], stop_line_pos[1]])
            ego_to_stop = stop_line_nparray - ego_pos
            ego_to_stop = ego_to_stop / np.linalg.norm(ego_to_stop) # normalized
            cos_stop = np.dot(ego_dir, ego_to_stop)

            if cos_stop > 0 and dist < min_dist:
                min_dist = dist
                target_stop_line = stop_line_pos
        
        if target_stop_line is None:
            return -1

        min_dist = float('inf')
        way_point = -1
        # test the first 50 waypoints that are nearest to the light
        for i in range(50):
            wp_i = waypoint_indices[i]
            wp_pos = self.waypoints[wp_i].pose.pose.position
            dist = (
                (target_stop_line[0] - wp_pos.x) ** 2
                + (target_stop_line[1] - wp_pos.y) ** 2
            )
            
            wp_nparray = np.array([wp_pos.x, wp_pos.y])
            ego_to_wp = wp_nparray - ego_pos
            ego_to_wp = ego_to_wp / np.linalg.norm(ego_to_wp) # normalized
            cos_wp = np.dot(ego_dir, ego_to_wp)

            if cos_wp > 0. and dist < min_dist:
                min_dist = dist
                way_point = wp_i

        return way_point

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light_index = None

        if(self.pose):
            light_index = self.get_closest_light(self.pose.pose)

        if light_index > -1:
            if self.light_indexed:
                light = self.lights[light_index]
                light_wp = self.get_light_wp(light_index)
                state = self.get_light_state(light)
                return light_wp, state
            else:
                return -1, TrafficLight.UNKNOWN
        else:
            if light_index == -1:
                if self.config['generate_train']:
                    pass

            return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
