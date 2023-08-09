import math
import numpy as np

import cv2 as cv
from cv_bridge import CvBridge

import rospy

from line_tracking.planning_strategies.error_type import ErrorType

# In OpenCV, hue ranges from 0 to 179
MAX_HUE = 179

# HSV thresholds for track detection
LOWER_YELLOW = (20, 50, 50)
UPPER_YELLOW = (30, 255, 255)

# Colors expressed in BGR format
TRACK_OUTLINE_COLOR = (255, 0, 255)
CROSSHAIR_COLOR = (255, 255, 255)
CENTROID_COLOR = (255, 255, 255)
ERROR_COLOR = (0, 0, 255)
ERROR_AUX_COLOR = (0, 0, 0)


# This planning strategy revolves around finding the centroid
#  of the track at each iteration and using it as waypoint.
class CentroidStrategy:
    def __init__(self, error_type, viz):
        self.error_type = error_type
        self.viz = viz

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

        # Find the track's contours
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        if len(contours) < 1:
            rospy.logwarn(f"No contours found, skipping.")
            return None

        # Compute centroid
        M = cv.moments(contours[0])
        if M["m00"] != 0:
            centroid_x = int(M["m10"] / M["m00"])
            centroid_y = int(M["m01"] / M["m00"])

            self.prev_centroid_x = centroid_x
            self.prev_centroid_y = centroid_y
        else:
            rospy.logwarn("No centroid found, reusing previous waypoint.")
            centroid_x = self.prev_centroid_x
            centroid_y = self.prev_centroid_y

        # Compute crosshair
        center_x = math.floor(width / 2)
        center_y = math.floor(height / 2)

        # Compute (very rough) position
        position_x = center_x
        position_y = height - 1

        if self.error_type == ErrorType.OFFSET:
            err, offset = self.compute_offset_error(
                (centroid_x, centroid_y), (center_x, center_y), width / 2
            )
        elif self.error_type == ErrorType.ANGLE:
            err, angle = self.compute_angle_error(
                (centroid_x, centroid_y), (position_x, position_y)
            )
        else:
            rospy.logerr(f"Unknown error type. Exiting")
            rospy.signal_shutdown("")

        # ugly beyond reason
        if self.viz:
            cv.drawContours(image, contours, -1, TRACK_OUTLINE_COLOR, 2)
            cv.circle(image, (centroid_x, centroid_y), 5, CENTROID_COLOR, 2)
            cv.circle(image, (center_x, center_y), 5, CROSSHAIR_COLOR, 2)

            if self.error_type == ErrorType.OFFSET:
                cv.line(
                    image,
                    (center_x, center_y),
                    (centroid_x, centroid_y),
                    ERROR_AUX_COLOR,
                    1,
                )
                cv.line(
                    image,
                    (center_x, center_y),
                    (center_x, centroid_y),
                    ERROR_AUX_COLOR,
                    1,
                )
                cv.line(
                    image,
                    (center_x, centroid_y),
                    (centroid_x, centroid_y),
                    ERROR_COLOR,
                    2,
                )
            elif self.error_type == ErrorType.ANGLE:
                cv.ellipse(
                    image,
                    (position_x, position_y),
                    (60, 60),
                    180,
                    90,
                    90 + angle,
                    ERROR_COLOR,
                    2,
                )
                cv.line(
                    image,
                    (position_x, position_y),
                    (centroid_x, centroid_y),
                    ERROR_COLOR,
                    2,
                )
                cv.line(
                    image,
                    (position_x, position_y),
                    (center_x, center_y),
                    ERROR_COLOR,
                    2,
                )

            cv.imshow("Visualization", image)
            cv.waitKey(1)

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
