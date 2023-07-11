#!/usr/bin/env python

import time

import cv2 as cv
from cv_bridge import CvBridge

import numpy as np
import math

import rospy
import sensor_msgs
import std_msgs


# In OpenCV hue ranges from 0 to 179
MAX_HUE = 179

# HSV thresholds
LOWER_YELLOW = (0, 10, 10)
UPPER_YELLOW = (40, 255, 255)

# Colors expressed in BGR format
TRACK_OUTLINE_COLOR = (255, 0, 255)
CROSSHAIR_COLOR = (255, 255, 255)
CENTROID_COLOR = (255, 255, 255)
ERROR_COLOR = (0, 0, 255)


# Performs track detection from raw camera images and publishes waypoint error data
class CameraNode:
    def __init__(self, cv_bridge):
        self.cv_bridge = cv_bridge

        # Receive camera images
        self.camera_sub = rospy.Subscriber(
            "/car/image_raw", sensor_msgs.msg.Image, self.camera_callback
        )

        # Advertise waypoint x-axis offset
        self.error_pub = rospy.Publisher(
            "/perception/waypoint_offset", std_msgs.msg.Float32, queue_size=1
        )

        # Whether to print debug data or not
        self.viz = rospy.get_param("/line_tracking/CameraNode/viz", False)

        rospy.loginfo("Camera node initialized!")

    # Called upon receiving raw camera input
    def camera_callback(self, msg):
        # start = time.time()

        image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        height, width, _ = image.shape
        image = image[int(height / 2) : height, 200 : width - 200]
        height, width, _ = image.shape

        # Convert to HSV and threshold the image to extract the (yellow) track
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, np.array(LOWER_YELLOW), np.array(UPPER_YELLOW))

        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        # Compute centroid
        M = cv.moments(contours[0])
        if M["m00"] != 0:
            centroid_x = int(M["m10"] / M["m00"])
            centroid_y = int(M["m01"] / M["m00"])
        else:
            centroid_x, centroid_y = 0, 0

        # Compute crosshair
        center_x = math.floor(width / 2)
        center_y = math.floor(height / 2)

        # Publish x-axis offset between the centroid and the crosshair
        msg = std_msgs.msg.Float32()
        msg.data = centroid_x - center_x
        self.error_pub.publish(msg)

        if self.viz:
            self.display_data(
                image,
                contours,
                (centroid_x, centroid_y),
                (center_x, center_y),
            )

        # end = time.time()
        # rospy.loginfo(end - start)

    # Visualize data for debugging purposes
    def display_data(self, image, contours, centroid, crosshair):
        cv.drawContours(image, contours, -1, TRACK_OUTLINE_COLOR, 2)

        cv.circle(image, centroid, 5, CENTROID_COLOR, 2)

        cv.circle(image, crosshair, 5, CROSSHAIR_COLOR, 2)

        cv.line(image, centroid, crosshair, ERROR_COLOR, 2)

        cv.imshow("Visualization", image)
        cv.waitKey(1)


if __name__ == "__main__":
    cv_bridge = CvBridge()

    rospy.init_node("Camera")
    CameraNode(cv_bridge)
    rospy.spin()
    rospy.loginfo("Camera node shutting down.")
