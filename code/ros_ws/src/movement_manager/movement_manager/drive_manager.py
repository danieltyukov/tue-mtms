#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from movement_manager.movement_manager import Buttons


class DriveManager(Node):
    def __init__(self):
        super().__init__("drive_manager")
        self.drive_sub = self.create_subscription(Joy, "drive", self.drive_callback, 10)
        self.right_pub = self.create_publisher(Float32, "drive_right", 10)
        self.left_pub = self.create_publisher(Float32, "drive_left", 10)

        self.MAX_PWM_INPUT = 255

    def drive_callback(self, msg):
        # Handle the drive message here
        #self.get_logger().info(f"Drive message received: {msg}")

        # Compute max voltage based on the thrust_factor from the message
        # In our case, the thrus factor is a voltage penalty, so it slows down the motors
        thrust_factor = msg.buttons[Buttons.BOOST.value]
        pwm_input = self.MAX_PWM_INPUT / thrust_factor

        # compute left and right motor outputs for given axes input
        (left, right) = self.computeMotorOutputs((msg.axes))

        left_msg = Float32()
        left_msg.data = left * pwm_input
        right_msg = Float32()
        right_msg.data = right * pwm_input

        # publish the left and right motor outputs
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

    # Joystick axes to left/right 'tank' wheels output
    def computeMotorOutputs(self, axes):
        s = axes[0]
        t = axes[1]

        # If moving backwards, invert steering
        if t < 0:
            s = -s

        # Calculate semi-constant curvature values
        left = ((t + abs(t) * s) + (t + s)) / 2
        right = ((t - abs(t) * s) + (t - s)) / 2

        # determine maximum output
        m = max(abs(left), abs(right))

        # scale if needed
        if m > 1.0:
            left /= m
            right /= m

        return (left, -right)


def main(args=None):
    rclpy.init(args=args)
    node = DriveManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
