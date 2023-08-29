import math
import numpy as np

import cv2 as cv
from cv_bridge import CvBridge

import rospy

from line_tracking.planning_strategies.error_type import ErrorType
from line_tracking.visualizer import Visualizer

# In OpenCV, hue ranges from 0 to 179
MAX_HUE = 179

# HSV thresholds for track detection
LOWER_YELLOW = (20, 50, 50)
UPPER_YELLOW = (30, 255, 255)


# This planning strategy revolves around finding the centroid
#  of the track at each iteration and using it as waypoint.
class CentroidStrategy:
    def __init__(self, error_type, should_visualize):
        self.error_type = error_type

        if should_visualize:
            self.viz = Visualizer()
        else:
            self.viz = None

        self.cv_bridge = CvBridge()

        self.prev_centroid_x = 0
        self.prev_centroid_y = 0

    def plan(self, img_msg):
        image = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        height, width, _ = image.shape

        # Convert to HSV and threshold the image to extract the (yellow) track
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        mask = cv.inRange(
            hsv,
            np.array(LOWER_YELLOW),
            np.array(UPPER_YELLOW),
        )

        # Compute centroid
        M = cv.moments(mask)
        if M["m00"] != 0:
            centroid = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            self.prev_centroid = centroid
        else:
            rospy.logwarn("No centroid found, reusing previous waypoint.")
            centroid = self.prev_centroid

        # Compute crosshair
        crosshair = (math.floor(width / 2), math.floor(height / 2))

        # Compute (very rough) position
        position = (math.floor(width / 2), height - 1)

        if self.error_type == ErrorType.OFFSET:
            err, offset = self.compute_offset_error(centroid, crosshair, width / 2)
        elif self.error_type == ErrorType.ANGLE:
            err, angle = self.compute_angle_error(centroid, position)
        else:
            rospy.logerr(f"Unknown error type. Exiting")
            rospy.signal_shutdown("")

            # Visualize data
        if self.viz is not None:
            self.viz.build_basic_bg(image)

            if self.error_type == ErrorType.OFFSET:
                self.viz.build_offset_error_overlay(crosshair, centroid)
            elif self.error_type == ErrorType.ANGLE:
                self.viz.build_angle_error_overlay(crosshair, centroid, position, angle)
            else:
                rospy.logerr(f"Unknown error type. Exiting")
                rospy.signal_shutdown("")

            self.viz.show()

        return err

    def compute_offset_error(self, waypoint, crosshair, max_offset):
        offset = waypoint[0] - crosshair[0]
        # Map the value obtained by remapping the offset to the [-1, 1] range
        return (offset + max_offset) / max_offset - 1, offset

    def compute_angle_error(self, waypoint, position):
        # Compute angle between centroid and heading
        dist = math.sqrt(
            (waypoint[0] - position[0]) ** 2 + (waypoint[1] - position[1]) ** 2
        )
        angle = math.asin((waypoint[0] - position[0]) / dist)
        angle_deg = angle * 180 / math.pi

        # Map the value obtained by remapping the angle from [-90, 90] to [-1, 1]
        return (angle_deg + 90) / 90 - 1, angle_deg
