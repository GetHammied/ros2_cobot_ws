import os
import yaml
import time  # Added for delays
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg
from typing import List, Union

SAVE_PATH = os.path.expanduser('~/ros2_ws/src/lss_custom_control/lss_custom_control/saved_positions.yaml')

def load_position(name) -> Union[List[float], float]:
    if not os.path.exists(SAVE_PATH):
        raise FileNotFoundError(f"Position file not found: {SAVE_PATH}")
    with open(SAVE_PATH, 'r') as f:
        data = yaml.safe_load(f)
    if name not in data:
        raise KeyError(f"Position '{name}' not found in {SAVE_PATH}")
    return data[name]

class ArmAndGripperExecutor(Node):
    def __init__(self):
        super().__init__('arm_gripper_executor')

        self.arm_joint_names = [
            'lss_arm_joint_1',
            'lss_arm_joint_2',
            'lss_arm_joint_3',
            'lss_arm_joint_4'
        ]
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_trajectory_controller/follow_joint_trajectory'
        )

        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_action_controller/gripper_cmd'
        )

    def send_arm_trajectory(self, joint_positions: List[float]):
        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self.arm_joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 2
        traj.points.append(point)

        goal_msg.trajectory = traj

        self.arm_client.wait_for_server()
        future = self.arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm trajectory goal rejected')
            return

        self.get_logger().info('Arm trajectory goal accepted')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Arm motion completed')

    def send_gripper_command(self, position: float):
        goal_msg = GripperCommand.Goal()
        goal_msg.command = GripperCommandMsg()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 1.0

        self.gripper_client.wait_for_server()
        future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper command rejected')
            return

        self.get_logger().info(f'Gripper command accepted (position={position})')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Gripper action completed')


def main():
    rclpy.init()
    node = ArmAndGripperExecutor()

    try:
        # Ordered sequence of movements with delay between steps
        sequence = [
            ('arm', load_position("arm_position_1")),
            ('arm', load_position("arm_position_2")),
            ('gripper', load_position("gripper_position_2")),
            ('arm', load_position("arm_position_3")),
            ('gripper', load_position("gripper_position_1")),
            ('arm', load_position("arm_position_3")),
            ('arm', load_position("arm_position_4")),
            ('arm', load_position("arm_position_5"))
        ]

        for move_type, value in sequence:
            if move_type == 'arm':
                node.send_arm_trajectory(value)
            elif move_type == 'gripper':
                if isinstance(value, list):
                    value = value[0]  # Take first value if saved as [x]
                node.send_gripper_command(value)

            time.sleep(1.0)  # ⏱ Add delay between steps here

    except (FileNotFoundError, KeyError) as e:
        node.get_logger().error(str(e))

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
