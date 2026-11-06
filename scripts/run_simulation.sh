#!/usr/bin/env bash
set -eo pipefail
robotics_root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$robotics_root"
: "${COPPELIASIM_BIN:?Set COPPELIASIM_BIN to your simulator executable}"
python3 -c 'import cv2, rclpy, cv_bridge, robomaster'
python3 -c 'from ament_index_python.packages import get_package_prefix; print(get_package_prefix("robomaster_example")); print(get_package_prefix("robomaster_ros"))'
mkdir -p outputs/simulation
export ROS_LOG_DIR="$robotics_root/outputs/ros-logs"
# A Python supervisor owns process groups, so bridge children also terminate.
exec python3 - "$robotics_root" "$COPPELIASIM_BIN" <<'PY'
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import rclpy
from sensor_msgs.msg import Image

root, simulator = Path(sys.argv[1]), sys.argv[2]
children, logs = [], []

def start(command, name):
    log = open(root / 'outputs/simulation' / (name + '.log'), 'w')
    logs.append(log)
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    children.append(process)
    return process


def ros(*args):
    return [sys.executable, '-c', 'from ros2cli.cli import main; main()', *args]


def check_alive():
    for child in children:
        if child.poll() is not None:
            raise RuntimeError('A simulation process exited; inspect outputs/simulation/*.log')


def interrupted(*_):
    raise KeyboardInterrupt

signal.signal(signal.SIGTERM, interrupted)
try:
    start([simulator, '-s0', '-f', str(root / 'scenes/easy_scene_dynamic.ttt')], 'simulator')
    for index in (0, 1):
        name = f'bridge-rm{index}'
        start(ros('launch', 'robomaster_ros', 'main.launch', f'name:=rm{index}',
                  f'serial_number:=RM{index}', 'with_model_description:=false',
                  'video_resolution:=720', 'video_raw:=1', 'video_h264:=-1', 'video_ffmpeg:=-1'), name)
        deadline = time.monotonic() + 60
        while ': Connected' not in (root / f'outputs/simulation/{name}.log').read_text():
            check_alive()
            if time.monotonic() > deadline:
                raise RuntimeError(f'{name} did not connect; inspect its log and loopback addresses')
            time.sleep(0.2)
        print(f'{name} connected', flush=True)
    rclpy.init()
    node = rclpy.create_node('navigation_camera_check')
    frames = []
    subscription = node.create_subscription(Image, '/rm1/camera/image_color', frames.append, 1)
    deadline = time.monotonic() + 30
    while not frames:
        check_alive()
        if time.monotonic() > deadline:
            raise RuntimeError('No overhead frames; inspect the camera bridge log')
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    start(ros('launch', 'robomaster_example', 'robomaster_controller.launch',
              'name:=rm0', 'update_mode:=true'), 'controller')
    print('Navigation started. Logs: outputs/simulation/controller.log', flush=True)
    children[0].wait()
except KeyboardInterrupt:
    pass
finally:
    for child in reversed(children):
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
    for child in reversed(children):
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGTERM)
    for log in logs:
        log.close()
PY
