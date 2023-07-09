import rospy
import std_msgs

# Base wheel speed
BASE_SPEED = 3


# Controls the car in order to approach a given waypoint.
# TODO: currently it's just a proportional controller,
#       the integral and derivative components will be added later
class Controlnode:
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

        # PID parameters
        self.k_p = 0.01

        # Other PID variables
        self.setpoint = 0
        self.prev_error = 0

        print("Initialized!")

    # React to the current offset from the waypoint by returning
    #   a new control command
    def handle_offset_callback(self, msg):
        measurement = msg.data

        error = self.setpoint - measurement
        self.prev_error = error

        control = self.k_p * measurement
        self.publish_wheel_control(control)

    # Publishe the provided control command
    def publish_wheel_control(self, control):
        msg = std_msgs.msg.Float64()
        msg.data = control + BASE_SPEED
        self.left_wheel_pub.publish(msg)
        msg.data = -control + BASE_SPEED
        self.right_wheel_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("Control")
    Controlnode()
    rospy.spin()
