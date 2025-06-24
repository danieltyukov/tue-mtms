#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from movement_msgs.action import MorphAction
from sensor_msgs.msg import Joy
from enum import Enum


class Mode(Enum):
    DRIVE = 1
    FLY = 2


class Buttons(Enum):
    BOOST = 0
    MORPH = 1


class MovementManager(Node):
    def __init__(self):
        super().__init__("movement_manager")

        # initialize mode and thrust_factor
        self.mode = Mode.DRIVE
        self.movement_boost_factor = 1
        self.morph_in_progress = False

        self.last_morph_time = self.get_clock().now()

        # Subscribe to joy movement events:
        self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)

        # crate publishers for drive and fly topics
        self.drive_pub = self.create_publisher(Joy, "drive", 10)
        self.fly_pub = self.create_publisher(Joy, "fly", 10)

        # Declare action server:
        self.morph_action_client = ActionClient(self, MorphAction, "morph")

    # joy topic callback: react to input from joystick
    def joy_callback(self, msg):
        # self.get_logger().info(f"Joy message received: {msg}")
        # Get rid of duplicate keypresses
        now = self.get_clock().now()

        # check if morph button is pressed and if morph is not already in progress
        if msg.buttons[Buttons.MORPH.value] == 1 and not self.morph_in_progress:
            if (now - self.last_morph_time).nanoseconds > 500_000_000:  # 500ms debounce
                # morph: send action goal
                self.last_morph_time = now
                self.morph_in_progress = True
                if self.mode == Mode.DRIVE:
                    self.get_logger().info("Q button pressed, changing to fly mode")
                    self.send_morph_goal(Mode.FLY)
                else:
                    self.get_logger().info("Q button pressed, changing to drive mode")
                    self.send_morph_goal(Mode.DRIVE)
            return  # Skip further processing during morph

        if not self.morph_in_progress:
            self.movement_boost_factor = 1
            # Update thrust number if space (first button) is pressed
            if msg.buttons[0] == 1:
                # Update thrust number
                self.get_logger().info("Space button pressed, updating thrust number")
                # Update thrust number logic here
                self.movement_boost_factor = 2

            # Publish joy axis to either /drive or /fly topic depending on the mode
            # Add thrust number in button field.
            move_data = Joy
            move_data = msg
            move_data.buttons[Buttons.BOOST.value] = self.movement_boost_factor
            if self.mode == Mode.DRIVE:
                # self.get_logger().info("Publishing to /drive topic")
                self.drive_pub.publish(move_data)
            else:
                # self.get_logger().info("Publishing to /fly topic")
                self.fly_pub.publish(move_data)

    def send_morph_goal(self, mode):
        # Create goal message
        goal_msg = MorphAction.Goal()
        goal_msg.mode = mode.value

        self.morph_action_client.wait_for_server()

        self.send_goal_future = self.morph_action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        self.send_goal_future.add_done_callback(self.morph_goal_response_callback)

    def morph_goal_response_callback(self, future):
        goal_handle = future.result()
        # Check if goal is accepted by server
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected :(")
            self.morph_in_progress = False
            return
        self.get_logger().info("Goal accepted :)")

        # Update mode based on the result
        if self.mode == Mode.DRIVE:
            self.mode = Mode.FLY
            self.get_logger().info("Changed to FLY mode")
        else:
            self.mode = Mode.DRIVE
            self.get_logger().info("Changed to DRIVE mode")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # handle result from action server
        result = future.result().result
        self.get_logger().info(f"result message: {result.message}")
        if result.success:
            self.get_logger().info("Action succeeded")
        else:
            self.get_logger().error("Action failed")

        self.morph_in_progress = False

    def feedback_callback(self, feedback_msg):
        # Handle feedback from the action server
        self.get_logger().info(f"Feedback: {feedback_msg.feedback.progress}%")
        # You can also update the mode here if needed
        # self.mode = feedback_msg.feedback.mode


def main(args=None):
    rclpy.init(args=args)
    node = MovementManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
