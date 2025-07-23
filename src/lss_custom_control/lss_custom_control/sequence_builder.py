import os
import yaml
import time
import threading
from fastapi import FastAPI, Query, HTTPException
from lss_custom_control.read_joint_position import JointReader
from pathlib import Path

app = FastAPI()

SAVE_PATH = os.path.expanduser('~/ros2_ws/src/lss_custom_control/lss_custom_control/saved_positions.yaml')
SEQUENCE_DIR = os.path.expanduser('~/ros2_ws/src/lss_custom_control/lss_custom_control/sequences')

GROUP_JOINTS = {
    'arm': ["lss_arm_joint_1", "lss_arm_joint_2", "lss_arm_joint_3", "lss_arm_joint_4"],
    'gripper': ["lss_arm_joint_5"]
}

os.makedirs(SEQUENCE_DIR, exist_ok=True)

# Helper to get unique pose name
pose_counter = {
    "arm": 1,
    "gripper": 1
}

def get_unique_pose_name(group):
    global pose_counter
    base_name = f"{group}_pos"

    # Load existing positions to avoid duplicates
    existing = {}
    if os.path.exists(SAVE_PATH):
        with open(SAVE_PATH, 'r') as f:
            existing = yaml.safe_load(f) or {}

    while True:
        candidate = f"{base_name}{pose_counter[group]}"
        if candidate not in existing:
            pose_counter[group] += 1
            return candidate
        pose_counter[group] += 1

# sequence_builder.py

def start_sequence_builder(app, name):
    if hasattr(app.state, "current_sequence_name"):
        raise ValueError("A sequence is already in progress.")

    app.state.current_sequence_name = name
    app.state.current_sequence_data = []
    return {"status": "started", "name": name}

def add_pose_to_sequence(app, group):
    if not hasattr(app.state, "current_sequence_name"):
        raise ValueError("No active sequence. Call start_sequence_builder first.")

    import rclpy
    pose_name = get_unique_pose_name(group)

    rclpy.init()
    node = JointReader(pose_name, group)
    try:
        while rclpy.ok() and not node.got_data:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    pose_entry = {
        "group": group,
        "joints": GROUP_JOINTS[group],
        "pose": pose_name,
    }

    app.state.current_sequence_data.append(pose_entry)
    return {"status": "added", "pose": pose_name}

def finalize_sequence(app):
    if not hasattr(app.state, "current_sequence_name") or not hasattr(app.state, "current_sequence_data"):
        return {"error": "No active sequence to finalize."}, 400

    path = os.path.join(SEQUENCE_DIR, f"{app.state.current_sequence_name}.yaml")

    with open(path, 'w') as f:
        yaml.dump(app.state.current_sequence_data, f)

    response = {
        "status": "saved",
        "name": app.state.current_sequence_name,
        "path": path,
        "steps": len(app.state.current_sequence_data),
    }

    # Clean up in-memory state
    del app.state.current_sequence_name
    del app.state.current_sequence_data

    return response

@app.post("/delete_sequence")
def delete_sequence(name: str = Query(...)):
    sequence_path = os.path.join(SEQUENCE_DIR, f"{name}.yaml")
    if not os.path.exists(sequence_path):
        raise HTTPException(status_code=404, detail="Sequence not found.")

    os.remove(sequence_path)
    return {"status": "deleted", "sequence_name": name}

@app.get("/list_sequences")
def list_sequences():
    files = Path(SEQUENCE_DIR).glob("*.yaml")
    return {"sequences": [f.stem for f in files]}
