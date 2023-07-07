import rospy
import std_msgs

from pynput import keyboard


class ManualControlnode:
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

        self.listener = keyboard.Listener(
            on_press=self.on_press,
        )
        self.listener.start()

        print("Initialized!")

    # Implements basic keyboard control
    def on_press(self, key):
        # Turn left
        if key == keyboard.Key.left:
            msg = std_msgs.msg.Float64()
            msg.data = 1
            self.right_wheel_pub.publish(msg)
            msg.data = 0
            self.left_wheel_pub.publish(msg)

        # Turn right
        elif key == keyboard.Key.right:
            msg = std_msgs.msg.Float64()
            msg.data = 1
            self.left_wheel_pub.publish(msg)
            msg.data = 0
            self.right_wheel_pub.publish(msg)

        # Accelerate both wheels equally
        elif key == keyboard.Key.up:
            msg = std_msgs.msg.Float64()
            msg.data = 1
            self.left_wheel_pub.publish(msg)
            self.right_wheel_pub.publish(msg)

        # Brake
        elif key == keyboard.Key.down:
            msg = std_msgs.msg.Float64()
            msg.data = 0
            self.left_wheel_pub.publish(msg)
            self.right_wheel_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("ManualControl")
    ManualControlnode()
    rospy.spin()
