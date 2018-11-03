#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
#from styx_msgs.msg import TLStatus
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree
import tf
import cv2
import yaml
import math
import time
import numpy as np
import PyKDL


STATE_COUNT_THRESHOLD = 3
UPDATE_RATE = 10

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.camera_image = None
        self.lights = []

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
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.image_display = rospy.Publisher('/image_proccessed', Image, queue_size=1)
        #self.active_tl_viz = rospy.Publisher('/wpts', PoseStamped, queue_size=1)
        #self.tl_viz = rospy.Publisher('tl_viz', MarkerArray, queue_size=1)
        #self.tl_front_viz = rospy.Publisher('tl_front_viz', Marker, queue_size=1)
        #self.wp_viz = rospy.Publisher('wp_viz', MarkerArray, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

#        rospy.spin()
        self.loop()

    def loop(self):
        rate = rospy.Rate(UPDATE_RATE)
        while not rospy.is_shutdown():
            rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x,
                                  waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
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

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_idx = 0
        if self.waypoint_tree is not None:
            closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        return closest_idx

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location
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
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #DONE Use tranform and rotation to calculate 2D position of light in image
        # import ipdb; ipdb.set_trace()
        f = 2300
        x_offset = -30
        y_offset = 340
        fx = f
        fy = f
        piw = PyKDL.Vector(point_in_world.x,point_in_world.y,point_in_world.z)
        R = PyKDL.Rotation.Quaternion(*rot)
        T = PyKDL.Vector(*trans)
        p_car = R*piw+T

        # x = -p_car[1]/p_car[0]*fx+image_width/2
        # y = -p_car[2]/p_car[0]*fx+image_height/2
        x = -p_car[1]/p_car[0]*fx+image_width/2 + x_offset
        y = -p_car[2]/p_car[0]*fx+image_height/2+y_offset

        return (int(x), int(y))

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # For testing
#        return light.state
    
        # if(not self.has_image):
        #     self.prev_light_loc = None
        #     return False

        image_orig = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        x, y = self.project_to_image_plane(light.pose.pose.position)
        rospy.loginfo("Project world to image X {} Y {}".format(x, y))
        #x = 400
        #y = 200
        if (x<0) or (y<0) or (x>=image_orig.shape[1]) or (y>=image_orig.shape[0]):
            return False

        xcrop = 100
        ycrop = 100
        xmin = x - xcrop if (x-xcrop) >= 0 else 0
        ymin = y - ycrop if (y-ycrop) >= 0 else 0

        # TODO:
        xmax = x + xcrop if (x + xcrop) <= image_orig.shape[1]-1 else image_orig.shape[1]-1
        ymax = y + ycrop if (y + ycrop) <= image_orig.shape[0]-1 else image_orig.shape[0]-1
        image_cropped = image_orig[ymin:ymax,xmin:xmax]
        image_display = image_cropped.copy()
#        cv2.circle(image_display, (xcrop,ycrop), 10, (255,0,0), 4)
        cv2.circle(image_display, ((xmax+xmin)/2,(ymax+ymin)/2), 10, (255,0,0), 4)
        image_message = self.bridge.cv2_to_imgmsg(image_display, "bgr8")
        
        self.image_display.publish(image_message)

        #TODO use light location to zoom in on traffic light in image

        #Get classification
        return self.light_classifier.get_classification(image_cropped)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x,
                                                   self.pose.pose.position.y)

            #TODO find the closest visible traffic light (if one exists)
            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                # Get stop line waypoint index
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                # Find closest stop line waypoint index
                d = temp_wp_idx - car_wp_idx
                if d >= 0 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx

        if closest_light:
            state = self.get_light_state(closest_light)
            if(state == TrafficLight.GREEN):
                rospy.loginfo("TL_detector GREEN {}".format(line_wp_idx))
            elif(state == TrafficLight.RED):
                rospy.loginfo("TL_detector RED {}".format(line_wp_idx))
            elif(state == TrafficLight.YELLOW):
                rospy.loginfo("TL_detector YELLOW {}".format(line_wp_idx))
            return line_wp_idx, state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
