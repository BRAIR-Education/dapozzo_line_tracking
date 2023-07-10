import time

import rospy
import std_msgs

# Base wheel speed
BASE_SPEED = 3


# Controls the car in order to approach a given waypoint.
class ControlNode:
    def __init__(self):
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

        # PID parameters TODO: read from configuration file
        self.k_p = 0.01
        self.k_i = 0.01
        self.k_d = 0.01

        # Other PID variables
        self.setpoint = 0
        self.prev_error = 0
        self.accumulated_error = 0
        self.time_prev = time.time()

        rospy.loginfo("Initialized!")

    # React to the current offset from the waypoint by returning
    #   a new control command
    def handle_offset_callback(self, msg):
        measurement = msg.data
        time_now = time.time()

        # We consider the provided offset as the error we want to reduce to 0
        error = measurement
        dt = time_now - self.time_prev

        if dt == 0:
            return

        p_term = self.k_p * error
        i_term = self.k_i * self.accumulated_error * dt
        d_term = self.k_d * (error - self.prev_error) / dt
        control = p_term + i_term + d_term

        self.prev_error = error
        self.accumulated_error += error
        self.time_prev = time_now

        self.publish_wheel_control(control)

    # Publishe the provided control command
    def publish_wheel_control(self, control):
        msg = std_msgs.msg.Float64()
        msg.data = control + BASE_SPEED
        self.left_wheel_pub.publish(msg)
        msg.data = -control + BASE_SPEED
        self.right_wheel_pub.publish(msg)

    # Stop the car
    def stop(self):
        msg = std_msgs.msg.Float64()
        msg.data = 0
        # Send the messages multiple times since they are not
        #   guaranteed to be delivered. Ugly, but it seems to work.
        for _ in range(10):
            self.left_wheel_pub.publish(msg)
            self.right_wheel_pub.publish(msg)

        rospy.loginfo("Shutting down.")


if __name__ == "__main__":
    rospy.init_node("Control")
    node = ControlNode()
    rospy.on_shutdown(node.stop)
    rospy.spin()
