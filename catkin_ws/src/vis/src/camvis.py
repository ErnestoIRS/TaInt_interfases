#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import requests

# Initialize the ROS node
rospy.init_node('yawcam_image_publisher')

# Create a publisher to publish the image data
image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
bridge = CvBridge()

port = '8081'  # Replace with the actual port of your Yawcam server
stream_url = 'http://' + "192.168.1.68" + ':' + port + '/video.mjpg'

stream = requests.get(stream_url, stream=True)
bytes = b''

while not rospy.is_shutdown():
    bytes += stream.raw.read(1024)
    a = bytes.find(b'\xff\xd8')
    b = bytes.find(b'\xff\xd9')
    if a != -1 and b != -1:
        jpg = bytes[a:b + 2]
        bytes = bytes[b + 2:]
        frame = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

        # Convert the OpenCV image to a ROS image message
        ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        # Publish the ROS image message
        image_pub.publish(ros_image)

# Clean up when done
stream.close()
cv2.destroyAllWindows()

