"""
Planner executor for MoveIt 2 + Lynxmotion LSS arm.

- Loads named joint positions from a YAML file (`saved_positions.yaml`)
- Loads sequences of poses (for arm & gripper) from per-sequence YAML files
- Sends goals to MoveIt via the MoveGroup action interface
- Supports external shutdown via `shutdown_event` from shared_state
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration  # NOTE: currently not used
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PlanningOptions
import yaml
import os
import time

# Global path to the file that stores all named joint positions.
# Structure is expected to be something like:
# {
#   "home": [0.0, 0.5, -0.3, ...],
#   "pick": [...],
#   ...
# }
SAVE_PATH = os.path.expanduser(
    '~/ros2_ws/src/lss_custom_control/lss_custom_control/saved_positions.yaml'
)

# Shutdown event set from other parts of the application
# (e.g. web UI or FastAPI server) to interrupt active sequences.
from lss_custom_control.shared_state import shutdown_event


def load_sequence(sequence_name):
    """
    Load a motion sequence from a YAML file.

    The file is expected at:
        ~/ros2_ws/src/lss_custom_control/lss_custom_control/sequences/<sequence_name>.yaml

    The YAML file typically contains a list of items like:
    - group: arm
      pose: home
    - group: gripper
      pose: open
    """
    seq_path = os.path.expanduser(
        f'~/ros2_ws/src/lss_custom_control/lss_custom_control/sequences/{sequence_name}.yaml'
    )

    if not os.path.exists(seq_path):
        raise FileNotFoundError(f"Sequence file not found: {seq_path}")

    with open(seq_path, 'r') as f:
        return yaml.safe_load(f)


def load_position(name):
    """
    Load a single named joint position from SAVE_PATH.

    Raises:
        FileNotFoundError: if the positions file does not exist.
        KeyError: if the given name is not defined in the YAML.
    """
    if not os.path.exists(SAVE_PATH):
        raise FileNotFoundError(f"Position file not found: {SAVE_PATH}")

    with open(SAVE_PATH, 'r') as f:
        data = yaml.safe_load(f)

    if name not in data:
        raise KeyError(f"Position '{name}' not found in {SAVE_PATH}")

    return data[name]


def plan_and_execute(node, group, joint_names, position_name, delay=1.0):
    """
    Helper to send a named pose to a joint group with an optional delay afterwards.

    Args:
        node (MoveItPlannerClient): Node used to communicate with the MoveGroup action.
        group (str): MoveIt planning group name (e.g. "lss_arm", "gripper").
        joint_names (list[str]): List of joint names for this group.
        position_name (str): Name of the saved position in saved_positions.yaml.
        delay (float): Seconds to sleep after execution (simple pacing between steps).
    """
    position = load_position(position_name)
    node.get_logger().info(f'➡️ Executing {position_name} for {group}...')
    node.send_goal(group, joint_names, position)
    time.sleep(delay)


class MoveItPlannerClient(Node):
    """
    Thin wrapper around MoveIt's MoveGroup action client.

    Responsibilities:
    - Create an ActionClient for /move_action
    - Build a MotionPlanRequest with joint constraints
    - Send the goal and wait for execution to finish
    - Handle cancelling an active goal
    """

    def __init__(self):
        super().__init__('planner_executor')
        # Action client for MoveIt MoveGroup
        self.client = ActionClient(self, MoveGroup, '/move_action')
        self.goal_handle = None

    def send_goal(self, group_name, joint_names, joint_goal):
        """
        Build and send a MoveGroup goal for the given joint group.

        Args:
            group_name (str): Name of the MoveIt group (e.g. "lss_arm", "gripper").
            joint_names (list[str]): Joint names corresponding to the positions.
            joint_goal (list[float]): Desired joint positions in radians or degrees
                                      consistent with your MoveIt setup.
        """
        goal_msg = MoveGroup.Goal()

        # Build the motion planning request
        request = MotionPlanRequest()
        request.planner_id = "STOMP"  # Planner plugin ID as configured in MoveIt
        request.group_name = group_name
        request.max_velocity_scaling_factor = 0.8
        request.max_acceleration_scaling_factor = 0.6

        # Build constraints: one JointConstraint for each target joint
        constraint = Constraints()

        for name, position in zip(joint_names, joint_goal):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            # Tight tolerances, tweak if planning becomes flaky
            jc.tolerance_above = 0.005
            jc.tolerance_below = 0.005
            jc.weight = 1.0
            constraint.joint_constraints.append(jc)

        # MoveIt accepts a list of goal constraints, so we add our single one
        request.goal_constraints.append(constraint)

        goal_msg.request = request

        # Planning options: directly execute the plan, no re-planning/look-around
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = False

        # Wait until MoveGroup action server is ready
        self.client.wait_for_server()

        # Send goal asynchronously and block until accepted/rejected
        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        self.goal_handle = future.result()

        if not self.goal_handle or not self.goal_handle.accepted:
            self.get_logger().error(f'Planning goal for {group_name} was rejected')
            return

        self.get_logger().info(f'Planning goal for {group_name} accepted. Executing...')

        # Wait for execution result
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f'Motion execution for {group_name} finished.')

    def cancel_goal(self):
        """
        Cancel the currently active MoveGroup goal, if there is one.

        This is triggered when `shutdown_event` is set during a sequence.
        """
        if self.goal_handle:
            self.get_logger().warn("🔴 Cancelling active MoveGroup goal...")
            cancel_future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            self.get_logger().info("✅ Goal cancel request sent.")


def run_planned_sequence(sequence_name):
    """
    Execute a named sequence of poses defined in a YAML sequence file.

    The sequence YAML is expected to be a list of items like:
    - group: arm
      pose: home
    - group: gripper
      pose: open
    - group: arm
      pose: pick

    `group` refers to a logical group ("arm" or "gripper"), which is then
    mapped to the actual MoveIt group names.

    The function:
    - Initializes rclpy (if needed)
    - Creates the MoveItPlannerClient node
    - Iterates over the sequence and executes each step
    - Checks `shutdown_event` before each step to allow external interruption
    """

    # Reset the shutdown flag before starting a new sequence
    shutdown_event.clear()

    # Initialize rclpy if not already running
    if not rclpy.ok():
        rclpy.init()

    node = MoveItPlannerClient()

    try:
        # Joint names defined in your URDF / MoveIt config for arm and gripper groups
        arm_joints = ["lss_arm_joint_1", "lss_arm_joint_2", "lss_arm_joint_3", "lss_arm_joint_4"]
        gripper_joints = ["lss_arm_joint_5"]

        # Load sequence definition from YAML
        sequence = load_sequence(sequence_name)

        # Map logical group names used in YAML to actual MoveIt group names
        GROUP_NAME_MAP = {
            "arm": "lss_arm",
            "gripper": "gripper"
        }

        # Default joint lists for each logical group
        GROUP_JOINTS = {
            "arm": arm_joints,
            "gripper": gripper_joints
        }

        for item in sequence:
            # Allow external stop (e.g. from web UI)
            if shutdown_event.is_set():
                node.get_logger().warn("❌ Shutdown requested, cancelling goal and stopping.")
                node.cancel_goal()
                break

            # Group as written in YAML ("arm" / "gripper" / etc.)
            original_group = item["group"]

            # Translate to the actual MoveIt group name
            group = GROUP_NAME_MAP.get(original_group, original_group)

            # Either use explicit joints from YAML or fall back to the default mapping
            joints = item.get("joints", GROUP_JOINTS[original_group])

            # Name of the pose in saved_positions.yaml
            pose = item["pose"]

            # Plan and execute the motion for this step
            plan_and_execute(node, group, joints, pose)

    except (FileNotFoundError, KeyError) as e:
        # Typical user errors: missing sequence or missing pose name
        node.get_logger().error(str(e))

    finally:
        # Clean shutdown of the node and rclpy context
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # Local test: run a sequence called "test1"
    # This expects a file test1.yaml in the sequences directory.
    run_planned_sequence("test1")
