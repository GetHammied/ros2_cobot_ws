"""
FastAPI-based web interface for controlling the LSS arm via ROS 2.

Responsibilities:
- Provide HTTP endpoints to:
  - Send the robot home / run planner-based sequences
  - Save named positions and build sequences
  - Power the robot on/off safely
  - Set servo effort (torque)
  - Trigger emergency shutdown (Not-Aus)
- Bridge between the web UI and various ROS 2 nodes/actions/topics.
"""

import os
import threading
import time
from pathlib import Path

import yaml
import rclpy
import lss_custom_control

from fastapi import FastAPI, Query, Request
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Float64MultiArray
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

from lss_custom_control.shared_state import shutdown_event
from lss_custom_control.sequence_builder import (
    start_sequence_builder,
    add_pose_to_sequence,
    finalize_sequence,
    delete_sequence,
    list_sequences,
)
from lss_custom_control.read_joint_position import JointReader
from lss_custom_control.sequence_executor import execute_sequence


# ---------------------------------------------------------------------------
# FastAPI + static/template setup
# ---------------------------------------------------------------------------

# BASE_DIR points to the Python package root (…/lss_custom_control/)
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Main FastAPI app instance
app = FastAPI()

# Serve static files (JS, CSS, images, etc.) under /static
app.mount(
    "/static",
    StaticFiles(directory=os.path.join(BASE_DIR, "static")),
    name="static",
)

# Jinja2 template engine for rendering index.html, etc.
templates = Jinja2Templates(directory=os.path.join(BASE_DIR, "templates"))

# Directory where YAML sequences are stored
SEQUENCES_DIR = Path(lss_custom_control.__file__).parent / "sequences"


# ---------------------------------------------------------------------------
# Low-level ROS helpers
# ---------------------------------------------------------------------------

def publish_hold_position(positions):
    """
    Publish a single-point JointTrajectory to overwrite the controller's target.

    Used to:
    - Freeze the current position as a "hold position" in the controller
    - Avoid drift when changing torque / cancelling trajectories
    """
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration

    # Ensure rclpy is initialized
    if not rclpy.ok():
        rclpy.init()

    class HoldPub(Node):
        def __init__(self):
            super().__init__("hold_position_pub")
            self.pub = self.create_publisher(
                JointTrajectory,
                "/arm_trajectory_controller/joint_trajectory",
                10,
            )

    node = HoldPub()

    # Wait (briefly) until at least one subscriber is connected
    t0 = time.time()
    while node.pub.get_subscription_count() == 0 and (time.time() - t0) < 1.0:
        time.sleep(0.02)

    msg = JointTrajectory()
    msg.joint_names = [
        "lss_arm_joint_1",
        "lss_arm_joint_2",
        "lss_arm_joint_3",
        "lss_arm_joint_4",
    ]

    pt = JointTrajectoryPoint()
    pt.positions = positions
    pt.velocities = [0.0] * len(positions)
    # Very short trajectory duration, just to overwrite the last command
    pt.time_from_start = Duration(sec=0, nanosec=10_000_000)  # 10 ms

    msg.points = [pt]
    node.pub.publish(msg)
    time.sleep(0.1)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def cancel_all_trajectories():
    """
    Cancel all active FollowJointTrajectory goals.

    Used before:
    - Power on / off sequences
    - Emergency shutdown
    to ensure no ongoing trajectory is fighting against user interaction.
    """
    if not rclpy.ok():
        rclpy.init()

    class CancelNode(Node):
        def __init__(self):
            super().__init__("cancel_goals_node")
            self.client = ActionClient(
                self,
                FollowJointTrajectory,
                "/arm_trajectory_controller/follow_joint_trajectory",
            )

        def cancel_all(self):
            # Wait for the action server to be available
            if not self.client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn("Action server not available, nothing to cancel")
                return

            # Access internal goal handles (hacky but effective)
            active_goals = list(self.client._goal_handles.keys())
            if not active_goals:
                self.get_logger().info("No active trajectory goals to cancel")
                return

            self.get_logger().info(f"Cancelling {len(active_goals)} goals...")
            futures = [gh.cancel_goal_async() for gh in active_goals]
            for f in futures:
                rclpy.spin_until_future_complete(self, f, timeout_sec=1.0)
            self.get_logger().info("All trajectory goals cancelled.")

    node = CancelNode()
    node.cancel_all()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def read_current_joint_positions(timeout=1.0):
    """
    Read /joint_states once and return a list of arm joint positions.

    Returns:
        list[float]: Positions ordered as:
            ["lss_arm_joint_1", ..., "lss_arm_joint_4"]
        Falls back to [0, 0, 0, 0] if nothing is received within the timeout.
    """
    rclpy.init()

    class JSReader(Node):
        def __init__(self):
            super().__init__("joint_state_reader")
            self.positions = None
            self.sub = self.create_subscription(
                JointState,
                "/joint_states",
                self.cb,
                10,
            )

        def cb(self, msg: JointState):
            if msg.name and msg.position:
                # Order according to your MoveIt arm group
                arm_joint_order = [
                    "lss_arm_joint_1",
                    "lss_arm_joint_2",
                    "lss_arm_joint_3",
                    "lss_arm_joint_4",
                ]
                pos_map = dict(zip(msg.name, msg.position))
                self.positions = [
                    pos_map[j] for j in arm_joint_order if j in pos_map
                ]

    node = JSReader()
    start = time.time()
    while (
        rclpy.ok()
        and node.positions is None
        and (time.time() - start) < timeout
    ):
        rclpy.spin_once(node, timeout_sec=0.05)

    positions = node.positions or [0.0, 0.0, 0.0, 0.0]
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return positions


def save_current_position_sequence():
    """
    Save the current arm pose as a one-step planner sequence.

    Side effects:
    - Updates `saved_positions.yaml` with a "temp_pose" entry.
    - Creates/overwrites `current_position.yaml` sequence which references "temp_pose".

    Returns:
        str: The name of the generated sequence file (without .yaml),
             currently always "current_position".
    """
    positions = read_current_joint_positions()

    # Single-step sequence pointing to the "temp_pose"
    seq = [{
        "group": "arm",
        "joints": [
            "lss_arm_joint_1",
            "lss_arm_joint_2",
            "lss_arm_joint_3",
            "lss_arm_joint_4",
        ],
        "pose": "temp_pose",  # same key each time -> overwritten
    }]

    positions_file = Path(lss_custom_control.__file__).parent / "saved_positions.yaml"

    # Load existing positions (if any)
    if positions_file.exists():
        with open(positions_file, "r") as f:
            poses_data = yaml.safe_load(f) or {}
    else:
        poses_data = {}

    # Overwrite temp_pose with current positions
    poses_data["temp_pose"] = positions
    with open(positions_file, "w") as f:
        yaml.safe_dump(poses_data, f)
        f.flush()
        os.fsync(f.fileno())

    # Overwrite the single-step sequence for the planner
    seq_file = SEQUENCES_DIR / "current_position.yaml"
    with open(seq_file, "w") as f:
        yaml.safe_dump(seq, f)
        f.flush()
        os.fsync(f.fileno())

    print(f"[power_on] wrote temp_pose -> {positions_file}")
    print(f"[power_on] wrote current_position -> {seq_file}")
    return "current_position"


# ---------------------------------------------------------------------------
# FastAPI lifecycle hooks
# ---------------------------------------------------------------------------

@app.on_event("shutdown")
def on_fastapi_shutdown():
    """
    Ensure rclpy is shut down when FastAPI stops.

    Prevents lingering ROS 2 contexts when the server is stopped.
    """
    if rclpy.ok():
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# High-level control endpoints
# ---------------------------------------------------------------------------

@app.post("/go_home")
def go_home():
    """
    Move the robot to the "home" sequence using the planner.

    Runs in a background thread so the HTTP request returns immediately.
    """
    def go_home_thread():
        from lss_custom_control.planner_executor import run_planned_sequence
        run_planned_sequence("home")

    thread = threading.Thread(target=go_home_thread)
    thread.start()
    return {"message": "🏠 Roboter fährt in die Home-Position."}


@app.post("/save_position")
def save_position(name: str = Query(...), group: str = Query(...)):
    """
    Capture and store the current joint position under a given name and group.

    Uses JointReader to read the joints and write them to saved_positions.yaml.
    """
    def run_node():
        if not rclpy.ok():
            rclpy.init()

        node = JointReader(position_name=name, group_name=group)
        try:
            while rclpy.ok() and not node.got_data:
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    thread = threading.Thread(target=run_node)
    thread.start()
    thread.join()
    return {"message": f"💾 Position '{name}' für Gruppe '{group}' gespeichert."}


@app.post("/run_sequence")
def run_sequence():
    """
    Execute a simple pre-defined sequence via sequence_executor.execute_sequence().
    """
    def run_sequence_thread():
        execute_sequence()

    thread = threading.Thread(target=run_sequence_thread)
    thread.start()
    return {"message": "▶️ Sequenz gestartet."}


@app.post("/run_planner_sequence")
def run_planner_sequence(name: str = Query(...)):
    """
    Execute a named MoveIt planner sequence defined in the sequences folder.
    """
    def planner_thread():
        from lss_custom_control.planner_executor import run_planned_sequence
        run_planned_sequence(name)

    thread = threading.Thread(target=planner_thread)
    thread.start()
    return {"message": f"▶️ Sequenz '{name}' wird ausgeführt."}


@app.post("/set_effort")
def set_effort(value: float = Query(...)):
    """
    Set the torque/effort value for all 5 servos via /effort_controller/commands.

    Note:
        This is a one-shot publisher; all motors get the same effort value.
    """
    def publish_once():
        if not rclpy.ok():
            rclpy.init()

        class OneShotPublisher(Node):
            def __init__(self):
                super().__init__('effort_oneshot_publisher')
                self.publisher = self.create_publisher(
                    Float64MultiArray,
                    '/effort_controller/commands',
                    10,
                )

            def publish(self):
                msg = Float64MultiArray()
                msg.data = [value] * 5
                self.publisher.publish(msg)
                time.sleep(0.5)

        node = OneShotPublisher()
        node.publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    thread = threading.Thread(target=publish_once)
    thread.start()
    return {"status": "sent", "effort": value}


@app.post("/power_on")
def power_on():
    """
    Power-on sequence for the robot:

    1) Save current pose as a planner sequence ("current_position").
    2) Cancel all active trajectory goals for safety.
    3) Send a hold-position trajectory to freeze the controller target.
    4) Optionally run MoveIt planner to "current_position".
    5) Enable torque by setting effort to 6.8.

    Runs in a background thread.
    """
    def run():
        # 1) Save the current pose
        seq_name = save_current_position_sequence()

        # 2) Cancel old goals (for safety)
        cancel_all_trajectories()

        # 3) Read and send hold position to overwrite controller's target
        positions = read_current_joint_positions()
        publish_hold_position(positions)

        # 4) Run MoveIt planner to "current_position"
        from lss_custom_control.planner_executor import run_planned_sequence
        run_planned_sequence(seq_name)

        # 5) Finally, enable torque
        time.sleep(0.5)
        set_effort(value=6.8)

    threading.Thread(target=run, daemon=True).start()
    return {"message": "⚡ Roboter eingeschaltet und in sichere Halteposition gebracht."}


@app.post("/power_off")
def power_off():
    """
    Power-off sequence:

    - Wait 3 seconds to give the user time to hold the arm.
    - Then set effort to 0.0 (torque off).
    """
    def delayed_power_off():
        time.sleep(3)  # 3 second warning delay
        set_effort(value=0.0)

    threading.Thread(target=delayed_power_off).start()
    return {
        "status": "⚠️ Der Roboter wird in 3 Sekunden stromlos. Bitte halten Sie den Arm fest!"
    }


# ---------------------------------------------------------------------------
# Sequence-builder endpoints (interactive from the web UI)
# ---------------------------------------------------------------------------

@app.post("/start_sequence_builder")
def api_start_sequence_builder(name: str = Query(...)):
    """
    Start building a new sequence with the given name.

    This initializes state in `app.state` and prepares for subsequent
    calls to add_pose_to_sequence / finalize_sequence.
    """
    start_sequence_builder(app, name)
    return {"message": f"🧱 Sequenz-Builder gestartet: '{name}'", "name": name}


@app.post("/add_pose_to_sequence")
def api_add_pose_to_sequence(group: str = Query(...)):
    """
    Add the current pose to the active sequence for the given group ("arm"/"gripper").
    """
    add_pose_to_sequence(app, group)
    return {
        "message": "🤖 Aktuelle Armposition wurde zur Sequenz hinzugefügt.",
        "group": group,
    }


@app.post("/finalize_sequence")
def api_finalize_sequence():
    """
    Finalize the current sequence and write it to a YAML file.
    """
    return finalize_sequence(app)


@app.post("/delete_sequence")
def api_delete_sequence(name: str = Query(...)):
    """
    Delete a sequence YAML file by name (without .yaml).
    """
    return delete_sequence(name)


@app.get("/list_sequences")
def api_list_sequences():
    """
    Return a JSON list of all existing sequence names (without .yaml).
    """
    return list_sequences()


@app.post("/shutdown_robot")
def shutdown_robot():
    """
    Emergency stop / Not-Aus:

    - Signal shutdown_event to stop planner_executor loops.
    - Cancel all active trajectories.
    - Read current joint state and publish a hold-position trajectory.
    """
    def shutdown_sequence():
        shutdown_event.set()
        cancel_all_trajectories()
        positions = read_current_joint_positions()
        publish_hold_position(positions)

    threading.Thread(target=shutdown_sequence).start()
    return {"message": "🛑 Not-Stopp: Roboter hält in Position."}


@app.get("/get_sequence_names")
def get_sequence_names():
    """
    Get names of all sequence YAML files in the sequences directory.
    """
    folder = SEQUENCES_DIR
    if not folder.exists():
        return JSONResponse(content={"sequences": []})

    files = [f.stem for f in folder.glob("*.yaml")]
    return JSONResponse(content={"sequences": files})


# ---------------------------------------------------------------------------
# Frontend + gripper helpers
# ---------------------------------------------------------------------------

@app.get("/", response_class=HTMLResponse)
async def read_root(request: Request):
    """
    Render the main web UI (index.html).
    """
    return templates.TemplateResponse("index.html", {"request": request})


@app.post("/add_open_gripper")
def add_open_gripper():
    """
    Add an "open_gripper" step to the currently active sequence.
    """
    _add_gripper_pose_to_sequence("open_gripper")
    return {"message": "👐 Greifer geöffnet und zur Sequenz hinzugefügt."}


@app.post("/add_close_gripper")
def add_close_gripper():
    """
    Add a "close_gripper" step to the currently active sequence.
    """
    _add_gripper_pose_to_sequence("close_gripper")
    return {"message": "✊ Greifer geschlossen und zur Sequenz hinzugefügt."}


def _add_gripper_pose_to_sequence(pose_name: str):
    """
    Internal helper to append a gripper pose (open/close) to the active sequence.

    Reads the pose from saved_positions.yaml and pushes one step to
    app.state.current_sequence_data.
    """
    positions_file = Path(lss_custom_control.__file__).parent / "saved_positions.yaml"
    if not positions_file.exists():
        return {"error": "Datei saved_positions.yaml nicht gefunden."}

    with open(positions_file, "r") as f:
        poses_data = yaml.safe_load(f)

    if pose_name not in poses_data:
        return {"error": f"'{pose_name}' nicht in saved_positions.yaml gefunden."}

    position = poses_data[pose_name]

    # Sequence builder must already have set these attributes
    if not hasattr(app.state, "current_sequence_name") or not hasattr(
        app.state, "current_sequence_data"
    ):
        return {"error": "Keine aktive Sequenz. Bitte zuerst eine Sequenz starten."}

    pose_entry = {
        "group": "gripper",
        "joints": ["lss_arm_joint_5"],
        "pose": pose_name,
    }

    app.state.current_sequence_data.append(pose_entry)

    # (Optional) Save updated poses_data back to file (even though we only read)
    with open(positions_file, "w") as f:
        yaml.safe_dump(poses_data, f)

    return {"status": f"Greiferaktion '{pose_name}' hinzugefügt."}


@app.post("/generate_place_sequence")
def generate_place_sequence(from_sequence: str = Query(...)):
    """
    Generate a "place" sequence automatically from an existing "pick" sequence:

    - Load the original sequence.
    - Deep-copy and reverse the steps.
    - In the reversed sequence, replace:
        gripper pose "close_gripper" -> "open_gripper"
    - Save as <from_sequence>_ablegen.yaml
    """
    import copy

    pick_path = SEQUENCES_DIR / f"{from_sequence}.yaml"
    if not pick_path.exists():
        return {"error": f"Sequence '{from_sequence}' not found."}

    with open(pick_path, "r") as f:
        pick_data = yaml.safe_load(f)

    # Deep copy and reverse
    place_data = copy.deepcopy(pick_data)[::-1]

    # Replace close_gripper with open_gripper on the way back
    for step in place_data:
        if step["group"] == "gripper" and step["pose"] == "close_gripper":
            step["pose"] = "open_gripper"

    # Save to new file (preserve original, append "_ablegen")
    new_name = f"{from_sequence}_ablegen"
    place_path = SEQUENCES_DIR / f"{new_name}.yaml"
    with open(place_path, "w") as f:
        yaml.safe_dump(place_data, f)

    return {"status": "place sequence created", "name": new_name}
