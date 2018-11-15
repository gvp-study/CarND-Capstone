import rospy
import cv2
import numpy as np
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        # HSV limits for RGB.
        self.lower_red = np.array([0,50,50],np.uint8) 
        self.upper_red = np.array([10,255,255],np.uint8) 
        self.lower_yellow = np.array([40.0/360*255, 120, 120],np.uint8)
        self.upper_yellow = np.array([66.0/360*255, 255, 255],np.uint8)
        self.lower_green = np.array([90.0/360*255, 120, 120],np.uint8)
        self.upper_green = np.array([140.0/360*255, 255, 255],np.uint8)
        
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        #TODO implement light color prediction

        image_hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

        #red
        frame_threshed = cv2.inRange(image_hsv, self.lower_red, self.upper_red)
        r = cv2.countNonZero(frame_threshed)
        if r > 50:
            rospy.loginfo("TL_classifier RED {} pixels".format(r))
            return TrafficLight.RED

        frame_threshed = cv2.inRange(image_hsv, self.lower_yellow, self.upper_yellow)
        y = cv2.countNonZero(frame_threshed)
        if y > 50:
            rospy.loginfo("TL_classifier YELLOW {} pixels".format(y))
            return TrafficLight.YELLOW

        frame_threshed = cv2.inRange(image_hsv, self.lower_green, self.upper_green)
        g = cv2.countNonZero(frame_threshed)
        if g > 50:
            rospy.loginfo("TL_classifier GREEN {} pixels".format(g))
            return TrafficLight.GREEN

        # import ipdb; ipdb.set_trace()

        return TrafficLight.UNKNOWN

