import os
from datetime import datetime
import csv

import rospy
import rospkg
import std_msgs


MAX_THRUST = 15  # Maximum base wheel speed
RAMP_UP = 0.5  # Acceleration step

TURNING_THRUST = 5  # Speed devoted to turning


# Controls the car in order to approach a given waypoint.
class ControlNode:
    # Initialize the control node
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
        self.time_start = 0
        self.time_prev = 0

        # Initial base wheel thrust
        #   This will slowly accelerate up to MAX_THRUST
        self.thrust = 0

        self.started = False

        # Publisher to control the left wheel
        self.left_wheel_pub = rospy.Publisher(
            "/car/front_left_velocity_controller/command",
            std_msgs.msg.Float64,
            queue_size=10,
        )

        # Publisher to control the right wheel
        self.right_wheel_pub = rospy.Publisher(
            "/car/front_right_velocity_controller/command",
            std_msgs.msg.Float64,
            queue_size=10,
        )

        # Subscriber to the error
        self.angle_sub = rospy.Subscriber(
            "/planning/error",
            std_msgs.msg.Float32,
            self.handle_error_callback,
        )

        rospy.loginfo("Control node initialized!")

    # React to the newly received angle to be corrected
    #
    # Arguments:
    #   msg: message containing the error to be fed into the PID
    def handle_error_callback(self, msg):
        measurement = msg.data
        time_now = rospy.get_rostime()

        if time_now == 0:
            rospy.logwarn("get_rostime() returned 0. Skipping.")
            return

        if self.started == False:
            self.time_start = time_now
            self.time_prev = time_now
            self.started = True
            return

        elapsed = (time_now - self.time_start).to_sec()

        # Check if we should stop
        if self.max_duration >= 0 and elapsed > self.max_duration:
            rospy.logwarn("Max duration reached.")
            self.stop()

        error = measurement
        dt = (time_now - self.time_prev).to_sec()

        if dt <= 0:
            return

        self.accumulated_integral += error * dt

        # Compute PID output
        p_term = self.k_p * error
        i_term = self.k_i * self.accumulated_integral
        d_term = self.k_d * (error - self.prev_error) / dt
        control = p_term + i_term + d_term

        self.prev_error = error
        self.time_prev = time_now

        # TODO: output saturation checks?

        # Gradually increase base thrust to prevent sudden accelerations at the beginning
        if self.thrust < MAX_THRUST:
            self.thrust += RAMP_UP

        v_l = self.thrust + TURNING_THRUST * control
        v_r = self.thrust - TURNING_THRUST * control

        self.log_data(elapsed, dt, error, control, v_l, v_r, p_term, i_term, d_term)
        self.publish_wheel_control(v_l, v_r)

    # Publish the provided control command
    #
    # Arguments:
    #   v_l: new left wheel speed
    #   v_r: new right wheel speed
    def publish_wheel_control(self, v_l, v_r):
        msg = std_msgs.msg.Float64()
        msg.data = v_l
        self.left_wheel_pub.publish(msg)
        msg.data = v_r
        self.right_wheel_pub.publish(msg)

    # Create a new log file
    def open_logfile(self):
        pkgdir = self.rospack.get_path("dapozzo_line_tracking")
        date = datetime.today().strftime("%Y-%m-%d_%H-%M-%S")
        pid_params = f"{self.k_p}-{self.k_i}-{self.k_d}".replace(".", ",")
        filepath = os.path.join(pkgdir, "logs", f"pid_log_{date}_[{pid_params}].csv")

        self.logfile = open(filepath, "w+")
        self.logwriter = csv.writer(self.logfile)
        field = ["Time", "dt", "Error", "CV", "LWheel", "RWheel", "P", "I", "D"]
        self.logwriter.writerow(field)

    # Write data to the log file
    #
    # Arguments:
    #   elapsed: time elapsed since the launch of the control node
    #   dt: time elapsed since the last iteration
    #   error: waypoint error
    #   control: PID output
    #   v_l: left wheel speed
    #   v_r: right wheel speed
    #   p_term: proportional PID term
    #   i_term: integral PID term
    #   d_term: derivative PID term
    def log_data(self, elapsed, dt, error, control, v_l, v_r, p_term, i_term, d_term):
        self.logwriter.writerow(
            [elapsed, dt, error, control, v_l, v_r, p_term, i_term, d_term]
        )

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
