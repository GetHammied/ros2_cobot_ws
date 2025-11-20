"""
Sequence builder utilities for the LSS arm.

This module serves two roles:

1. Helper functions used by the main FastAPI app:
   - start_sequence_builder(app, name)
   - add_pose_to_sequence(app, group)
   - finalize_sequence(app)
   - delete_sequence(name)
   - list_sequences()

   These functions operate on `app.state` to accumulate a sequence
   of poses and finally write them as YAML files.

2. A standalone FastAPI app (the `app = FastAPI()` below) that exposes
   `/delete_sequence` and `/list_sequences` directly if this module is
   run as the main application.

The main control server imports the helper functions and uses its own
FastAPI instance, so the `app` defined here is usually not the one that
serves requests in your full system.
"""

import os
import yaml
import time
import threading
from pathlib import Path

from fastapi import FastAPI, Query, HTTPException
from lss_custom_control.read_joint_position import JointReader

# Standalone FastAPI app for this module (rarely used directly in your main setup)
app = FastAPI()

# Paths where positions and sequences are stored
SAVE_PATH = os.path.expanduser(
    '~/ros2_ws/src/lss_custom_control/lss_custom_control/saved_positions.yaml'
)
SEQUENCE_DIR = os.path.expanduser(
    '~/ros2_ws/src/lss_custom_control/lss_custom_control/sequences'
)

# Mapping of logical groups to joint names in the URDF / MoveIt setup
GROUP_JOINTS = {
    'arm': ["lss_arm_joint_1", "lss_arm_joint_2", "lss_arm_joint_3", "lss_arm_joint_4"],
    'gripper': ["lss_arm_joint_5"],
}

# Ensure the sequence directory exists
os.makedirs(SEQUENCE_DIR, exist_ok=True)

# In-memory counters for generating unique pose names per group
pose_counter = {
    "arm": 1,
    "gripper": 1,
}


def get_unique_pose_name(group):
    """
    Generate a unique pose name for a given group ("arm" or "gripper").

    Pattern:
        "<group>_pos<N>"

    It checks the existing entries in saved_positions.yaml so that it
    does not collide with previously saved poses.

    Args:
        group (str): "arm" or "gripper".

    Returns:
        str: A pose name that does not yet exist in the positions file.
    """
    global pose_counter
    base_name = f"{group}_pos"

    # Load existing positions to avoid name collisions
    existing = {}
    if os.path.exists(SAVE_PATH):
        with open(SAVE_PATH, 'r') as f:
            existing = yaml.safe_load(f) or {}

    # Increment pose_counter[group] until we find a free name
    while True:
        candidate = f"{base_name}{pose_counter[group]}"
        if candidate not in existing:
            pose_counter[group] += 1
            return candidate
        pose_counter[group] += 1


# ---------------------------------------------------------------------------
# Sequence builder helpers (used by the main FastAPI app)
# ---------------------------------------------------------------------------

def start_sequence_builder(app, name):
    """
    Initialize a new sequence-building session.

    Stores the sequence name and an empty list in app.state, so that
    subsequent calls to add_pose_to_sequence() can append their steps.

    Args:
        app (FastAPI): The main FastAPI app instance whose state we use.
        name (str): Name of the sequence (used as filename without .yaml).

    Raises:
        ValueError: if a sequence is already in progress.

    Returns:
        dict: status information with the started sequence name.
    """
    if hasattr(app.state, "current_sequence_name"):
        raise ValueError("A sequence is already in progress.")

    app.state.current_sequence_name = name
    app.state.current_sequence_data = []
    return {"status": "started", "name": name}


def add_pose_to_sequence(app, group):
    """
    Capture the current joint positions and add them as a new step
    to the active sequence.

    This:
    - Generates a unique pose name for the group (arm/gripper).
    - Uses JointReader to read from ROS and store the pose in
      saved_positions.yaml under that name.
    - Appends a step entry to app.state.current_sequence_data.

    Args:
        app (FastAPI): The main FastAPI app instance whose state we use.
        group (str): "arm" or "gripper".

    Raises:
        ValueError: if no sequence has been started.

    Returns:
        dict: status information and the created pose name.
    """
    if not hasattr(app.state, "current_sequence_name"):
        raise ValueError("No active sequence. Call start_sequence_builder first.")

    import rclpy

    pose_name = get_unique_pose_name(group)

    # Create a temporary JointReader node that saves the pose to SAVE_PATH
    rclpy.init()
    node = JointReader(pose_name, group)
    try:
        while rclpy.ok() and not node.got_data:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # Build a sequence step that references the newly stored pose
    pose_entry = {
        "group": group,
        "joints": GROUP_JOINTS[group],
        "pose": pose_name,
    }

    app.state.current_sequence_data.append(pose_entry)
    return {"status": "added", "pose": pose_name}


def finalize_sequence(app):
    """
    Finalize and save the currently active sequence to a YAML file.

    Writes app.state.current_sequence_data to:
        <SEQUENCE_DIR>/<sequence_name>.yaml

    After writing the file, clears the in-memory sequence state.

    Args:
        app (FastAPI): The main FastAPI app instance whose state we use.

    Returns:
        dict or (dict, int):
            - On success: a dictionary with status, name, path, and number of steps.
            - On error (no active sequence): ({"error": ...}, 400)
              This tuple style is mainly for compatibility with FastAPI responses.
    """
    if not hasattr(app.state, "current_sequence_name") or not hasattr(
        app.state, "current_sequence_data"
    ):
        return {"error": "No active sequence to finalize."}, 400

    path = os.path.join(SEQUENCE_DIR, f"{app.state.current_sequence_name}.yaml")

    # Dump all steps to a YAML file
    with open(path, 'w') as f:
        yaml.dump(app.state.current_sequence_data, f)

    response = {
        "status": "saved",
        "name": app.state.current_sequence_name,
        "path": path,
        "steps": len(app.state.current_sequence_data),
    }

    # Clean up in-memory state so a new sequence can be started
    del app.state.current_sequence_name
    del app.state.current_sequence_data

    return response


# ---------------------------------------------------------------------------
# Standalone FastAPI endpoints for sequences (if this module is run directly)
# ---------------------------------------------------------------------------

@app.post("/delete_sequence")
def delete_sequence(name: str = Query(...)):
    """
    Delete a sequence file by name through HTTP.

    Args:
        name (str): Base sequence name (without .yaml).

    Raises:
        HTTPException(404): if the sequence file does not exist.

    Returns:
        dict: status message and deleted sequence name.
    """
    sequence_path = os.path.join(SEQUENCE_DIR, f"{name}.yaml")
    if not os.path.exists(sequence_path):
        raise HTTPException(status_code=404, detail="Sequence not found.")

    os.remove(sequence_path)
    return {"status": "deleted", "sequence_name": name}


@app.get("/list_sequences")
def list_sequences():
    """
    List all available sequences in SEQUENCE_DIR as a JSON list.

    Returns:
        dict: {"sequences": [<name1>, <name2>, ...]}
              where names do not include the .yaml extension.
    """
    files = Path(SEQUENCE_DIR).glob("*.yaml")
    return {"sequences": [f.stem for f in files]}
