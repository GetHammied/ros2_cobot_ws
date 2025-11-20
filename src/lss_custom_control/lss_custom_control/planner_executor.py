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
from lss_custom_control.shared_state import shutdown_event



def load_sequence(sequence_name):
    seq_path = os.path.expanduser(f'~/ros2_ws/src/lss_custom_control/lss_custom_control/sequences/{sequence_name}.yaml')
    if not os.path.exists(seq_path):
        raise FileNotFoundError(f"Sequence file not found: {seq_path}")
    with open(seq_path, 'r') as f:
        return yaml.safe_load(f)


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
        self.goal_handle = None

    def send_goal(self, group_name, joint_names, joint_goal):
        goal_msg = MoveGroup.Goal()
    
        request = MotionPlanRequest()
        request.planner_id = "STOMP"
        request.group_name = group_name
        request.max_velocity_scaling_factor = 0.8
        request.max_acceleration_scaling_factor = 0.6

        constraint = Constraints()

        for name, position in zip(joint_names, joint_goal):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 0.005
            jc.tolerance_below = 0.005
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
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error(f'Planning goal for {group_name} was rejected')
            return

        self.get_logger().info(f'Planning goal for {group_name} accepted. Executing...')
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f'Motion execution for {group_name} finished.')

    def cancel_goal(self):
        if self.goal_handle:
            self.get_logger().warn("🔴 Cancelling active MoveGroup goal...")
            cancel_future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            self.get_logger().info("✅ Goal cancel request sent.")




def run_planned_sequence(sequence_name):
    # Reset the shutdown flag before starting a new sequence
    shutdown_event.clear()
    if not rclpy.ok():
        rclpy.init()


    node = MoveItPlannerClient()

    try:
        arm_joints = ["lss_arm_joint_1", "lss_arm_joint_2", "lss_arm_joint_3", "lss_arm_joint_4"]
        gripper_joints = ["lss_arm_joint_5"]

        sequence = load_sequence(sequence_name)

        GROUP_NAME_MAP = {
            "arm": "lss_arm",
            "gripper": "gripper"
        }

        GROUP_JOINTS = {
            "arm": arm_joints,
            "gripper": gripper_joints
        }

        for item in sequence:
            if shutdown_event.is_set():
                node.get_logger().warn("❌ Shutdown requested, cancelling goal and stopping.")
                node.cancel_goal()
                break


            original_group = item["group"]
            group = GROUP_NAME_MAP.get(original_group, original_group)
            joints = item.get("joints", GROUP_JOINTS[original_group])
            pose = item["pose"]
            plan_and_execute(node, group, joints, pose)

    except (FileNotFoundError, KeyError) as e:
        node.get_logger().error(str(e))

    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    run_planned_sequence("test1")  

