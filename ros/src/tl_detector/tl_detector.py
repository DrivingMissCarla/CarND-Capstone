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

import math
import numpy as np
from points_organizer import PointsOrganizer


STATE_COUNT_THRESHOLD = 3
MAX_DETECTION_DIST = 100.0 # Maximum distance allowed to detect/classify a traffic light

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=rospy.DEBUG)

        self.pose = None
        self.waypoints = None
        self.waypoints_organizer = None
        self.camera_image = None
        self.lights = []
        self.stop_line_organizer = None

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.state_count = 0

        self.has_image = False
        self.camera_image = None

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.stop_line_positions = self.config['stop_line_positions']
        self.stop_line_organizer = PointsOrganizer(
            [[stop_line[0], stop_line[1]] for stop_line in self.stop_line_positions])
        rospy.logdebug('{} stop lines loaded.'.format(len(self.stop_line_positions)))

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1,
                                buff_size=100*1024*1024, tcp_nodelay=True)

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        self.waypoints_organizer = PointsOrganizer(
            [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y]
             for waypoint in waypoints.waypoints])

    def traffic_cb(self, msg):
        self.lights = msg.lights
        light_wp = self.find_closest_traffic_light()
        self.publish_upcoming_red_light(light_wp)

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """
        self.has_image = True
        self.camera_image = msg
        state = self.get_light_state()
        rospy.logdebug('Image classified as state: {}.'.format(state))

        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
        self.state_count += 1

    def publish_upcoming_red_light(self, light_wp):
        """Verifies if the car should stot at the traffic light ahead
           and publish the closesst waypoint to the stop line or -1 if the car can continue
        """
        light_wp = light_wp if self.state != TrafficLight.GREEN else -1
        self.upcoming_red_light_pub.publish(Int32(light_wp))

    def get_light_state(self):
        """Determines the current color of the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        # Getting classification
        return self.light_classifier.get_classification(cv_image)


    def find_closest_traffic_light(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
        """
        if self.pose and self.waypoints_organizer and self.stop_line_organizer and self.lights:
            closest_waypoint_idx = self.waypoints_organizer.get_closest_point_idx(
                self.pose.pose.position.x, self.pose.pose.position.y, look_mode='AHEAD')
            closest_waypoint = self.waypoints.waypoints[closest_waypoint_idx]

            # Getting the closest stop line ahead of the vehicle
            closest_stop_line_idx = self.stop_line_organizer.get_closest_point_idx(
                closest_waypoint.pose.pose.position.x, closest_waypoint.pose.pose.position.y, look_mode='AHEAD')

            if closest_stop_line_idx is not None:
                closest_light = self.lights[closest_stop_line_idx]

                # Use this just for testing
                #self.state = closest_light.state

                dist_to_light = math.sqrt((self.pose.pose.position.x - closest_light.pose.pose.position.x)**2 +
                                          (self.pose.pose.position.y - closest_light.pose.pose.position.y)**2)
                # If the closest traffic light ahead is not within the maximum distance,
                # skips classifying and publishing it
                if dist_to_light > MAX_DETECTION_DIST:
                    return -1

                # Getting the stop line associated with the closest light
                closest_stop_line = self.stop_line_positions[closest_stop_line_idx]

                # Getting the waypoint closest to stop line
                stop_waypoint_idx = self.waypoints_organizer.get_closest_point_idx(
                    closest_stop_line[0], closest_stop_line[1], look_mode='AHEAD')
                #rospy.logdebug('CURRENT: {} -> STOP: {}'.format(closest_waypoint_idx, stop_waypoint_idx))

                return stop_waypoint_idx

        return -1


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
