import rospy
import cv2
import numpy as np
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier

        self.lower_red_left = np.array([0,50,50]) # in HSV
        self.upper_red_limit = np.array([10,255,255]) # in HSV
        self.lower_red_right = np.array([160,50,50]) # in HSV
        self.upper_red_right = np.array([179,255,255]) # in HSV
        
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
        frame_threshed = cv2.inRange(image_hsv, self.lower_red_left, self.upper_red_limit)
        r = cv2.countNonZero(frame_threshed)

        if r > 50:
            # Apply Red limits
            red_image = cv2.inRange(image_hsv, self.lower_red_left, self.upper_red_limit)

            # Apply all between blue and red
            rest_image = cv2.inRange(image_hsv, self.lower_red_right, self.upper_red_right)

            # Weight the entire image
            weighted_image = cv2.addWeighted(red_image, 1.0, rest_image, 1.0, 0.0)

            blurred_image = cv2.GaussianBlur(weighted_image, (15,15), 0.0)
        
            circles = cv2.HoughCircles(blurred_image,cv2.HOUGH_GRADIENT,
                                         0.5,41,
                                         param1=70,param2=30,minRadius=5,
                                         maxRadius=150)
            # ensure at least some circles were found
            if circles is not None:
	        # convert the (x, y) coordinates and radius of the circles to integers
	        circles = np.round(circles[0, :]).astype("int")
 
	        # loop over the (x, y) coordinates and radius of the circles
	        for (x, y, r) in circles:
		    # draw the circle in the output image, then draw a rectangle
		    # corresponding to the center of the circle
		    cv2.circle(image, (x, y), r, (0, 255, 0), 4)
		    cv2.rectangle(image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                    
                rospy.loginfo("TL_classifier RED {} pixels".format(r))
                return TrafficLight.RED, image

        YELLOW_MIN = np.array([40.0/360*255, 120, 120],np.uint8)
        YELLOW_MAX = np.array([66.0/360*255, 255, 255],np.uint8)
        frame_threshed = cv2.inRange(image_hsv, YELLOW_MIN, YELLOW_MAX)
        y = cv2.countNonZero(frame_threshed)
        if y > 50:
            rospy.loginfo("TL_classifier YELLOW {} pixels".format(y))
            return TrafficLight.YELLOW, image

        GREEN_MIN = np.array([90.0/360*255, 120, 120],np.uint8)
        GREEN_MAX = np.array([140.0/360*255, 255, 255],np.uint8)
        frame_threshed = cv2.inRange(image_hsv, GREEN_MIN, GREEN_MAX)
        g = cv2.countNonZero(frame_threshed)
        if g > 50:
            rospy.loginfo("TL_classifier GREEN {} pixels".format(g))
            return TrafficLight.GREEN, image

        # import ipdb; ipdb.set_trace()

        return TrafficLight.UNKNOWN, image

