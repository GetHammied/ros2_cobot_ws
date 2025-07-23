import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml
import os
import argparse

SAVE_PATH = os.path.expanduser('~/ros2_ws/src/lss_custom_control/lss_custom_control/saved_positions.yaml')

GROUP_JOINTS = {
    'arm': [
        'lss_arm_joint_1',
        'lss_arm_joint_2',
        'lss_arm_joint_3',
        'lss_arm_joint_4',
    ],
    'gripper': [
        'lss_arm_joint_5',
    ]
}

class JointReader(Node):
    def __init__(self, position_name, group_name):
        super().__init__('joint_reader')
        self.position_name = position_name
        self.group_name = group_name
        self.joint_names_of_interest = GROUP_JOINTS.get(group_name)

        if not self.joint_names_of_interest:
            raise ValueError(f"Unknown group name: {group_name}")

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )
        self.got_data = False

    def listener_callback(self, msg):
        if self.got_data:
            return

        positions = []
        for name in self.joint_names_of_interest:
            if name in msg.name:
                index = msg.name.index(name)
                positions.append(round(msg.position[index], 4))
            else:
                self.get_logger().warn(f'Joint {name} not found in /joint_states')
                return

        self.got_data = True
        self.get_logger().info(f'✅ Saving {self.position_name}: {positions}')

        if os.path.exists(SAVE_PATH):
            with open(SAVE_PATH, 'r') as f:
                data = yaml.safe_load(f) or {}
        else:
            data = {}

        data[self.position_name] = positions

        with open(SAVE_PATH, 'w') as f:
            yaml.dump(data, f)

        self.get_logger().info(f'📁 Saved to {SAVE_PATH}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', required=True)
    parser.add_argument('--group', choices=['arm', 'gripper'], required=True)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = JointReader(args.name, args.group)
    print(f"🔍 Waiting for /joint_states for group '{args.group}'...")

    try:
        while rclpy.ok() and not node.got_data:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("❌ Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
