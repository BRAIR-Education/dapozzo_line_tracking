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
LOWER_YELLOW = (20, 50, 50)
UPPER_YELLOW = (30, 255, 255)

# Colors expressed in BGR format
TRACK_OUTLINE_COLOR = (255, 255, 255)
LEFT_LIMIT_COLOR = (255, 150, 0)
RIGHT_LIMIT_COLOR = (0, 255, 255)
CENTERLINE_COLOR = (0, 255, 0)
CROSSHAIR_COLOR = (255, 255, 255)
WAYPOINT_COLOR = (255, 255, 255)
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

        track_outline = self.get_track_outline(image)

        # Crop image to remove unwanted border pixels and to ignore faraway parts of the track
        cropped_outline = track_outline[
            int(height / 2) : (height - 10), 10 : (width - 10)
        ]
        cr_height, cr_width = cropped_outline.shape

        left_limit, right_limit = self.extract_track_limits(cropped_outline)
        centerline = self.compute_centerline(left_limit, right_limit)

        if left_limit.size == 0 or right_limit.size == 0:
            rospy.logwarn("WARNING: empty track limit")
            return

        # Compute crosshair
        center_x = math.floor(cr_width / 2)
        center_y = math.floor(cr_height / 2)

        # Detect waypoint (centerline point closest to crosshair)
        waypoint, waypoint_offset = self.get_next_waypoint(
            centerline, (center_x, center_y)
        )

        # Publish x-axis offset between the projected waypoint and the crosshair
        msg = std_msgs.msg.Float32()
        msg.data = waypoint_offset
        self.error_pub.publish(msg)

        if self.viz:
            self.display_data(
                left_limit,
                right_limit,
                centerline,
                (center_x, center_y),
                waypoint,
                cr_height,
                cr_width,
            )

        # end = time.time()
        # rospy.loginfo(end - start)

    # Detect the track in the input image, draw its contour on a new binary image and return it
    def get_track_outline(self, input):
        height, width, _ = input.shape
        track_outline = np.zeros((height, width), dtype=np.uint8)

        # Convert to HSV and threshold the image to extract the (yellow) track
        hsv = cv.cvtColor(input, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, np.array(LOWER_YELLOW), np.array(UPPER_YELLOW))

        # Detect track outline and draw it on a new image
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        cv.drawContours(track_outline, contours, 0, TRACK_OUTLINE_COLOR)

        return track_outline

    # Compute the centerline given the left and right track limits and return it
    def compute_centerline(self, left, right):
        centerline = []
        for (x1, y1), (x2, y2) in list(zip(left, right)):
            xc = math.floor((x1 + x2) / 2)
            yc = math.floor((y1 + y2) / 2)

            centerline.append((xc, yc))

        return np.array(centerline)

    # Return the left and right track limits from the provided track outline
    def extract_track_limits(self, track_outline):
        # We expect 3 labels:
        #   0: background
        #   1: left track limit
        #   2: right track limit
        _, labels = cv.connectedComponents(track_outline)

        left_limit_cols, left_limit_rows = np.where(labels == 1)
        left_limit = np.column_stack((left_limit_rows, left_limit_cols))[::10]

        right_limit_cols, right_limit_rows = np.where(labels == 2)
        right_limit = np.column_stack((right_limit_rows, right_limit_cols))[::10]

        return left_limit, right_limit

    # Obtain the next waypoint based on crosshair position
    #   and return it along with its x-axis offset
    def get_next_waypoint(self, trajectory, crosshair):
        if trajectory.size == 0:
            return crosshair, 0

        center_x, center_y = crosshair

        closest = 0
        closest_dist = float("inf")
        for i, (x, y) in enumerate(trajectory):
            # Ignore waypoints below crosshair
            if y > center_y - 30:
                continue

            dist = math.sqrt(abs(x - center_x) ** 2 + abs(y - center_y) ** 2)
            if dist < closest_dist:
                closest_dist = dist
                closest = i

        return trajectory[closest], x - center_x

    # Visualize data for debugging purposes
    def display_data(
        self, left_limit, right_limit, centerline, crosshair, waypoint, height, width
    ):
        canvas = np.zeros((height, width, 3), dtype=np.uint8)

        for point in left_limit:
            cv.circle(canvas, point, 1, LEFT_LIMIT_COLOR, 1)

        for point in right_limit:
            cv.circle(canvas, point, 1, RIGHT_LIMIT_COLOR, 1)

        for point in centerline:
            cv.circle(canvas, point, 1, CENTERLINE_COLOR, 1)

        cv.circle(canvas, crosshair, 3, CROSSHAIR_COLOR)

        cv.circle(canvas, waypoint, 3, WAYPOINT_COLOR)

        cv.line(canvas, crosshair, waypoint, ERROR_COLOR, 1)

        cv.imshow("Visualization", canvas)
        cv.waitKey(1)


if __name__ == "__main__":
    cv_bridge = CvBridge()

    rospy.init_node("Camera")
    CameraNode(cv_bridge)
    rospy.spin()
    rospy.loginfo("Camera node shutting down.")
