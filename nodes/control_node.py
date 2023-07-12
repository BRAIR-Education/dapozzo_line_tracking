import os
import time
from datetime import datetime
import csv

import rospy
import rospkg
import std_msgs

# Base wheel speed
BASE_SPEED = 3


# Controls the car in order to approach a given waypoint.
class ControlNode:
    def __init__(self):
        # Time to run for. Useful for doing multiple runs with the same duration.
        #   -1 indicates no limit.
        #   Note: this limit is only checked when doing a control iteration, so
        #   if no input is received it may never be checked.
        self.max_duration = rospy.get_param("/line_tracking/ControlNode/duration", -1)

        # PID parameters
        self.k_p = rospy.get_param("/line_tracking/ControlNode/k_p", 0.01)
        self.k_i = rospy.get_param("/line_tracking/ControlNode/k_i", 0.00)
        self.k_d = rospy.get_param("/line_tracking/ControlNode/k_d", 0.00)

        rospy.loginfo(f"PID params: {self.k_p}, {self.k_i}, {self.k_d}")

        # Logging utilities
        self.rospack = rospkg.RosPack()
        self.open_logfile()

        # Other PID variables
        self.setpoint = 0
        self.prev_error = 0
        self.accumulated_integral = 0
        self.time_start = time.time()
        self.time_prev = self.time_start

        self.log_interval = 0.1
        self.last_log = self.time_start

        # Publisher to control the left wheel
        self.left_wheel_pub = rospy.Publisher(
            "/car/front_left_velocity_controller/command",
            std_msgs.msg.Float64,
            queue_size=1,
        )

        # Publisher to control the right wheel
        self.right_wheel_pub = rospy.Publisher(
            "/car/front_right_velocity_controller/command",
            std_msgs.msg.Float64,
            queue_size=1,
        )

        # Subscriber to the x-axis offset of the waypoint
        self.offset_sub = rospy.Subscriber(
            "/perception/waypoint_offset",
            std_msgs.msg.Float32,
            self.handle_offset_callback,
        )

        rospy.loginfo("Control node initialized!")

    # React to the current offset from the waypoint by returning
    #   a new control command
    def handle_offset_callback(self, msg):
        measurement = msg.data
        time_now = time.time()

        # Check if we should stop
        if self.max_duration >= 0 and time_now - self.time_start > self.max_duration:
            rospy.loginfo("Max duration reached.")
            self.stop()

        # We consider the provided offset as the error we want to reduce to 0
        error = measurement
        dt = time_now - self.time_prev

        if dt == 0:
            return

        self.accumulated_integral += error * dt

        p_term = self.k_p * error
        i_term = self.k_i * self.accumulated_integral
        d_term = self.k_d * (error - self.prev_error) / dt
        control = p_term + i_term + d_term

        self.prev_error = error
        self.time_prev = time_now

        self.log_data(time_now - self.time_start, error, control)
        self.publish_wheel_control(control)

    # Publishe the provided control command
    def publish_wheel_control(self, control):
        msg = std_msgs.msg.Float64()
        msg.data = control + BASE_SPEED
        self.left_wheel_pub.publish(msg)
        msg.data = -control + BASE_SPEED
        self.right_wheel_pub.publish(msg)

    def open_logfile(self):
        pkgdir = self.rospack.get_path("dapozzo_line_tracking")
        date = datetime.today().strftime("%Y-%m-%d_%H-%M-%S")
        pid_params = f"{self.k_p}-{self.k_i}-{self.k_d}".replace(".", ",")
        filepath = os.path.join(pkgdir, "logs", f"pid_log_{date}_[{pid_params}].csv")

        self.logfile = open(filepath, "w+")
        self.logwriter = csv.writer(self.logfile)
        field = ["Time", "Error", "CV"]
        self.logwriter.writerow(field)

    def log_data(self, elapsed, error, control):
        if time.time() - self.last_log < self.log_interval:
            return

        self.logwriter.writerow([elapsed, error, control])

    # Stop the car
    def stop(self):
        msg = std_msgs.msg.Float64()
        msg.data = 0
        # Send the messages multiple times since they are not
        #   guaranteed to be delivered. Ugly, but it seems to work.
        for _ in range(10):
            self.left_wheel_pub.publish(msg)
            self.right_wheel_pub.publish(msg)

        self.logfile.close()
        rospy.loginfo("Control node shutting down.")
        rospy.signal_shutdown("Duration limit reached.")


if __name__ == "__main__":
    rospy.init_node("Control")
    node = ControlNode()
    rospy.on_shutdown(node.stop)
    rospy.spin()
