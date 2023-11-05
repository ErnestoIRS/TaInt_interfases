#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np

class GreenObjectDetector:
    def __init__(self):
        rospy.init_node('green_object_detector')
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.green_object_pub = rospy.Publisher("/green_object_coordinates", String, queue_size=1)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Convert the image to the HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for the green color
        lower_green = np.array([35, 100, 100])
        upper_green = np.array([85, 255, 255])

        # Create a mask to identify green pixels
        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)

        # Find contours in the mask
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        green_object_coordinates = ""

        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                green_object_coordinates += "({}, {}) ".format(cx, cy)

        if green_object_coordinates:
            self.green_object_pub.publish(green_object_coordinates)

if __name__ == '__main__':
    try:
        node = GreenObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
