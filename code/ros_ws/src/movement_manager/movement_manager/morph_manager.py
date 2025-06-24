#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action.server import ActionServer
from movement_msgs.action import MorphAction
from movement_manager.movement_manager import Mode
from std_srvs.srv import SetBool
from movement_msgs.srv import SetAngle


class MorphManager(Node):
    def __init__(self):
        super().__init__("morph_manager")
        self.cli = self.create_client(SetAngle, "servo_service")

        # connect to the servo_service
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Waiting for servo_angle service..")
        self.mode = Mode.DRIVE

        # declare action server
        self.morph_action_server = ActionServer(
            self,
            MorphAction,
            "morph",
            self.handle_action_request,
        )

    # Send request to servo nodes via servo_service
    # See servo node for specific behavior for true or false
    # TODO: maybe make this more transparent by deciding exact servo behavior in this function instead of in the servo node.
    #   e.g.: custom srv message that sends degrees.
    def send_request(self, angle: int):
        request = SetAngle.Request()
        request.angle = angle 
        future = self.cli.call_async(request)
        future.add_done_callback(self.response_callback)

    # Response callback for the service call to handle service response
    # TODO: maybe return the response so we can react to the response and the main function knows when an error occured and can react to that aswell
    # (could be merged into send_request function to make it easier possibly)
    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f"Succes: {response.success}, Message: {response.message}"
            )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    # Function to handle action requests
    def handle_action_request(self, goal_handle):
        self.get_logger().info("Received morph action request")

        result = MorphAction.Result()

        # accept or reject the goal and call appropriate function to execute the action
        if self.mode.value == goal_handle.request.mode:
            result.success = True
            result.message = "Already in requested mode"

        elif goal_handle.request.mode == Mode.DRIVE.value:
            self.get_logger().info("Changing to DRIVE mode")
            self.fly_to_drive(goal_handle)

            self.mode = Mode.DRIVE

            result.success = True
            result.message = "Changed to DRIVE mode"

        elif goal_handle.request.mode == Mode.FLY.value:
            self.get_logger().info("Changing to FLY mode")
            self.drive_to_fly(goal_handle)

            self.mode = Mode.FLY

            result.success = True
            result.message = "Changed to FLY mode"

        else:
            result.success = False
            result.message = "Invalid mode requested"

        goal_handle.succeed()  # Dont know if this is the correctr location to call this.
        # Maybe before modifying result, or in the function we call to send instructions to Servo nodes
        return result

    def fly_to_drive(self, goal_handle):
        # Logic to change from fly to drive mode
        self.get_logger().info("Executing fly to drive transition")

        # Send update sequence to servo nodes via service using send_request function
        # Temporary update sequence:
        feedback_msg = MorphAction.Feedback()
        for i in range(1, 10):
            feedback_msg.progress = i * 10
            self.send_request(90 - i * 10)
            self.get_logger().info(f"Transition progress: {i}%")
            time.sleep(1)
            goal_handle.publish_feedback(feedback_msg)

    def drive_to_fly(self, goal_handle):
        # Logic to change from drive to fly mode
        self.get_logger().info("Executing drive to fly transition")

        # Send update sequence to servo nodes via service using send_request function
        # Temporary update sequence:
        feedback_msg = MorphAction.Feedback()
        for i in range(1, 10):
            feedback_msg.progress = i * 10
            self.send_request(i * 10)
            self.get_logger().info(f"Transition progress: {i}%")
            time.sleep(1)
            goal_handle.publish_feedback(feedback_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MorphManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
