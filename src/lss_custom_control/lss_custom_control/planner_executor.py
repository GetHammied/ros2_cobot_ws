import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PlanningOptions
import yaml
import os
import time

SAVE_PATH = os.path.expanduser('~/ros2_ws/src/lss_custom_control/lss_custom_control/saved_positions.yaml')

def load_position(name):
    if not os.path.exists(SAVE_PATH):
        raise FileNotFoundError(f"Position file not found: {SAVE_PATH}")
    with open(SAVE_PATH, 'r') as f:
        data = yaml.safe_load(f)
    if name not in data:
        raise KeyError(f"Position '{name}' not found in {SAVE_PATH}")
    return data[name]

def plan_and_execute(node, group, joint_names, position_name, delay=1.0):
    """Helper to send a named pose to a joint group with optional delay"""
    position = load_position(position_name)
    node.get_logger().info(f'➡️ Executing {position_name} for {group}...')
    node.send_goal(group, joint_names, position)
    time.sleep(delay)

class MoveItPlannerClient(Node):
    def __init__(self):
        super().__init__('planner_executor')
        self.client = ActionClient(self, MoveGroup, '/move_action')

    def send_goal(self, group_name, joint_names, joint_goal):
        goal_msg = MoveGroup.Goal()
        
        request = MotionPlanRequest()
        request.planner_id = "BiTRRT"
        request.group_name = group_name
        request.max_velocity_scaling_factor = 0.2  # modify
        request.max_acceleration_scaling_factor = 0.1

        constraint = Constraints()

        for name, position in zip(joint_names, joint_goal):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraint.joint_constraints.append(jc)

        request.goal_constraints.append(constraint)

        goal_msg.request = request
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = False

        self.client.wait_for_server()
        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Planning goal for {group_name} was rejected')
            return

        self.get_logger().info(f'Planning goal for {group_name} accepted. Executing...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f'Motion execution for {group_name} finished.')


def main():
    rclpy.init()
    node = MoveItPlannerClient()

    try:
        # Define joint names
        arm_joints = ["lss_arm_joint_1", "lss_arm_joint_2", "lss_arm_joint_3", "lss_arm_joint_4"]
        gripper_joints = ["lss_arm_joint_5"]

        # Ordered sequence of actions
        sequence = [
            ("lss_arm", arm_joints, "arm_position_1"),
            ("lss_arm", arm_joints, "arm_position_2"),
            ("gripper", gripper_joints, "gripper_position_2"),
            ("lss_arm", arm_joints, "arm_position_3"),
            ("gripper", gripper_joints, "gripper_position_1"),
            ("lss_arm", arm_joints, "arm_position_4"),
            ("lss_arm", arm_joints, "arm_position_5"),
        ]

        for group, joints, position_name in sequence:
            plan_and_execute(node, group, joints, position_name, delay=1.0)

    except (FileNotFoundError, KeyError) as e:
        node.get_logger().error(str(e))

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
