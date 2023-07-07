#!/usr/bin/env python

import time

import cv2 as cv
from cv_bridge import CvBridge

import numpy as np
import matplotlib.pyplot as plt
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
LEFT_LIMIT_COLOR = (255, 150, 0)
RIGHT_LIMIT_COLOR = (0, 255, 255)
CENTERLINE_COLOR = (0, 255, 0)
CROSSHAIR_COLOR = (255, 255, 255)
WAYPOINT_COLOR = (255, 255, 255)
ERROR_COLOR = (0, 0, 255)


class CameraNode:
    def __init__(self, cv_bridge):
        self.cv_bridge = cv_bridge

        # Subscribe to camera output
        self.camera_sub = rospy.Subscriber(
            "/car/image_raw", sensor_msgs.msg.Image, self.camera_callback
        )

        # Advertise
        self.error_pub = rospy.Publisher(
            "/perception/waypoint_dist", std_msgs.msg.Float32
        )

    def camera_callback(self, msg):
        start = time.time()

        image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        rows, cols, _ = image.shape

        # Convert to HSV and threshold the image to extract the (yellow) track
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, np.array(LOWER_YELLOW), np.array(UPPER_YELLOW))

        # Detect track outline and draw it on a new image
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        track_outline = np.zeros((rows, cols), dtype=np.uint8)
        cv.drawContours(track_outline, contours, 0, CROSSHAIR_COLOR)

        # Crop image to remove unwanted border pixels and to ignore faraway parts of the track
        cropped_outline = track_outline[int(rows / 2) : (rows - 10), 10 : (cols - 10)]
        cropped_original = image[int(rows / 2) : (rows - 10), 10 : (cols - 10)]

        # We expect 3 labels:
        #   0: background
        #   1: left track limit
        #   2: right track limit
        _, labels = cv.connectedComponents(cropped_outline)

        final = np.zeros(cropped_original.shape, dtype=np.uint8)
        cr_rows, cr_cols, _ = final.shape

        # Draw left track limit
        left_limit_cols, left_limit_rows = np.where(labels == 1)
        left_limit = np.column_stack((left_limit_rows, left_limit_cols))[::10]
        for i, point in enumerate(left_limit):
            cv.circle(final, point, 1, LEFT_LIMIT_COLOR)
            # cv.putText(final, str(i), point, cv.FONT_HERSHEY_PLAIN, 0.5, BLUE)

        # Draw right track limit
        right_limit_cols, right_limit_rows = np.where(labels == 2)
        right_limit = np.column_stack((right_limit_rows, right_limit_cols))[::10]
        for i, point in enumerate(right_limit):
            cv.circle(final, point, 1, RIGHT_LIMIT_COLOR)
            # cv.putText(final, str(i), point, cv.FONT_HERSHEY_PLAIN, 0.5, YELLOW)

        # Compute and draw centerline
        centerline = []
        for i, ((x1, y1), (x2, y2)) in enumerate(list(zip(left_limit, right_limit))):
            xc = math.floor((x1 + x2) / 2)
            yc = math.floor((y1 + y2) / 2)
            cv.circle(final, (xc, yc), 1, CENTERLINE_COLOR)
            # cv.putText(
            #     final, str(i), (xc, yc), cv.FONT_HERSHEY_PLAIN, 0.5, PURPLE
            # )

            centerline.append((xc, yc))

        # Draw crosshair
        center_x = math.floor(cr_cols / 2)
        center_y = math.floor(cr_rows / 2)
        cv.circle(final, (center_x, center_y), 1, CROSSHAIR_COLOR)

        # Detect waypoint (centerline point closest to crosshair)
        closest = 0
        closest_dist = float("inf")
        for i, (x, y) in enumerate(centerline):
            dist = math.sqrt(abs(x - center_x) ** 2 + abs(y - center_y) ** 2)
            if dist < closest_dist:
                closest_dist = dist
                closest = i

        # Publish the distance between the projected waypoint and the crosshair
        msg = std_msgs.msg.Float32()
        msg.data = closest_dist
        self.error_pub.publish(msg)

        # Draw line between crosshair and waypoint
        cv.line(
            final,
            (centerline[closest][0], centerline[closest][1]),
            (center_x, center_y),
            ERROR_COLOR,
        )
        cv.circle(
            final, (centerline[closest][0], centerline[closest][1]), 1, WAYPOINT_COLOR
        )

        end = time.time()
        print(end - start)

        # cv.imshow("points", final)
        # cv.waitKey(1)


if __name__ == "__main__":
    cv_bridge = CvBridge()

    rospy.init_node("Camera")
    CameraNode(cv_bridge)
    rospy.spin()
