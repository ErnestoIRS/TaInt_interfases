#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class GreenObjectDetector:
    def __init__(self):
        rospy.init_node('green_object_detector_node')
        self.bridge = CvBridge()
        self.green_object_pub = rospy.Publisher("/green_object_coordinates", PointStamped, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

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
        contours = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Crear un mensaje PointStamped
                green_object_msg = PointStamped()
                green_object_msg.header.stamp = rospy.Time.now()
                green_object_msg.header.frame_id = "camera_frame"  # Sustituye con el frame_id correcto si es necesario
                green_object_msg.point.x = cx
                green_object_msg.point.y = cy
                green_object_msg.point.z = 0

                self.green_object_pub.publish(green_object_msg)

if __name__ == '__main__':
    try:
        node = GreenObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
