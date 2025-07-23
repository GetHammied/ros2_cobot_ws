import os
import threading
import time

import rclpy
from fastapi import FastAPI, Query, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node

from lss_custom_control.sequence_builder import (
    start_sequence_builder,
    add_pose_to_sequence,
    finalize_sequence,
    delete_sequence,
    list_sequences,
)
from lss_custom_control.read_joint_position import JointReader
from lss_custom_control.sequence_executor import execute_sequence

# Setup static and templates
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
app = FastAPI()
app.mount("/static", StaticFiles(directory=os.path.join(BASE_DIR, "static")), name="static")
templates = Jinja2Templates(directory=os.path.join(BASE_DIR, "static"))

# Root page (HTML UI)
@app.get("/", response_class=HTMLResponse)
async def read_root(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@app.on_event("shutdown")
def shutdown_event():
    if rclpy.ok():
        rclpy.shutdown()

@app.post("/save_position")
def save_position(name: str = Query(...), group: str = Query(...)):
    def run_node():
        rclpy.init()
        node = JointReader(position_name=name, group_name=group)
        try:
            while rclpy.ok() and not node.got_data:
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            node.destroy_node()
            rclpy.shutdown()
    thread = threading.Thread(target=run_node)
    thread.start()
    thread.join()
    return {"status": "saved", "name": name, "group": group}

@app.post("/run_sequence")
def run_sequence():
    def run_sequence_thread():
        execute_sequence()
    thread = threading.Thread(target=run_sequence_thread)
    thread.start()
    return {"status": "started"}

@app.post("/run_planner_sequence")
def run_planner_sequence(name: str = Query(...)):
    def planner_thread():
        from lss_custom_control.planner_executor import run_planned_sequence
        run_planned_sequence(name)
    thread = threading.Thread(target=planner_thread)
    thread.start()
    return {"status": "started", "sequence": name}

@app.post("/set_effort")
def set_effort(value: float = Query(...)):
    def publish_once():
        rclpy.init()
        class OneShotPublisher(Node):
            def __init__(self):
                super().__init__('effort_oneshot_publisher')
                self.publisher = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
            def publish(self):
                msg = Float64MultiArray()
                msg.data = [value] * 5
                self.publisher.publish(msg)
                time.sleep(0.5)
        node = OneShotPublisher()
        node.publish()
        node.destroy_node()
        rclpy.shutdown()
    thread = threading.Thread(target=publish_once)
    thread.start()
    return {"status": "sent", "effort": value}

@app.post("/power_on")
def power_on():
    return set_effort(value=6.8)

@app.post("/power_off")
def power_off():
    return set_effort(value=0.0)

@app.post("/start_sequence_builder")
def api_start_sequence_builder(name: str = Query(...)):
    return start_sequence_builder(app, name)

@app.post("/add_pose_to_sequence")
def api_add_pose_to_sequence(group: str = Query(...)):
    return add_pose_to_sequence(app, group)

@app.post("/finalize_sequence")
def api_finalize_sequence():
    return finalize_sequence(app)

@app.post("/delete_sequence")
def api_delete_sequence(name: str = Query(...)):
    return delete_sequence(name)

@app.get("/list_sequences")
def api_list_sequences():
    return list_sequences()
