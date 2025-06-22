#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PlanningOptions

class PickExecutor(Node):
    def __init__(self):
        super().__init__('pick_executor')
        self.cli = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info("Waiting for move_group action server...")
        self.cli.wait_for_server()
        self.get_logger().info("Connected to move_group action server")

    def send_named_target(self, group, named_target):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group
        goal_msg.request.goal_constraints.append(Constraints(name=named_target))
        goal_msg.request.num_planning_attempts = 3
        goal_msg.request.allowed_planning_time = 5.0

        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = False

        self.get_logger().info(f"Sending named target '{named_target}' to group '{group}'")

        future = self.cli.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"Goal for pose '{named_target}' rejected!")
            return

        self.get_logger().info(f"Goal for pose '{named_target}' accepted.")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info(f"Successfully moved to pose '{named_target}'")
        else:
            self.get_logger().error(f"Motion to '{named_target}' failed. Error code: {result.error_code.val}")

def main():
    rclpy.init()
    executor = PickExecutor()

    sequence = ["home", "my_new_pose", "pre_close", "lift"]
    for pose in sequence:
        executor.send_named_target("lss_arm", pose)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
