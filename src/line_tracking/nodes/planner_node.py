#!/usr/bin/env python

import rospy
import sensor_msgs
import std_msgs

from line_tracking.planning_strategies.centroid_strategy import CentroidStrategy
from line_tracking.planning_strategies.centerline_strategy import CenterlineStrategy
from line_tracking.planning_strategies.error_type import ErrorType


# Performs track detection from raw camera images and publishes waypoint error data
class PlannerNode:
    def __init__(self):
        # Type of error to compute
        error_type_arg = rospy.get_param("/line_tracking/PlannerNode/error_type")
        if error_type_arg == "offset":
            error_type = ErrorType.OFFSET
        elif error_type_arg == "angle":
            error_type = ErrorType.ANGLE
        else:
            rospy.logerr(f"Unknown error type {error_type_arg}. Exiting.")
            rospy.signal_shutdown("")

        # Whether to print debug data or not
        self.viz = rospy.get_param("/line_tracking/PlannerNode/viz", False)

        # Type of path planning approach to use
        planning_strategy = rospy.get_param("/line_tracking/PlannerNode/strategy")
        if planning_strategy == "centroid":
            self.strategy = CentroidStrategy(error_type, self.viz)
        elif planning_strategy == "centerline":
            self.strategy = CenterlineStrategy(error_type, self.viz)
        else:
            rospy.logerr(f"Unknown strategy {planning_strategy}. Exiting.")
            rospy.signal_shutdown("")

        # Receive camera images
        self.camera_sub = rospy.Subscriber(
            "/car/image_raw", sensor_msgs.msg.Image, self.camera_callback
        )

        # Advertise error to be corrected
        self.error_pub = rospy.Publisher(
            "/planning/error", std_msgs.msg.Float32, queue_size=1
        )

        rospy.loginfo("Planner node initialized!")

    # Called upon receiving raw camera input
    def camera_callback(self, msg):
        # start = time.time()

        # Compute the error based on the selected strategy and publish it
        err = self.strategy.plan(msg)
        if err == None:
            return

        err_msg = std_msgs.msg.Float32()
        err_msg.data = err
        self.error_pub.publish(err_msg)

        # end = time.time()
        # rospy.loginfo(end - start)


if __name__ == "__main__":
    rospy.init_node("Planner")
    PlannerNode()
    rospy.spin()
    rospy.loginfo("Planner node shutting down.")
