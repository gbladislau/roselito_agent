import numpy as np
import pyrealsense2 as rs
from subprocess import run
import subprocess
import os
import signal
import cv2
import streamlit as st
import glob
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from pathlib import Path

# 1. ROS 2 Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

st.set_page_config(layout="wide")

# ==========================================
# CLASS DEFINITION FOR BUFFER
# ==========================================
class CameraFrameBuffer:
    def __init__(self):
        self.bgr_frame = None
        self.depth_frame = None
        self.lock = threading.Lock()
        self.camera_available = False
        self.camera_error = ""

# ==========================================
# 1. UNIFIED & CACHED HARDWARE SYSTEM
# ==========================================
@st.cache_resource
def initialize_camera_system():
    # Create a single, persistent buffer instance that survives script reruns
    buffer = CameraFrameBuffer()
    
    # Define the isolated camera hardware thread inside the enclosure
    def camera_loop():
        try:
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            pipeline.start(config)
            align = rs.align(rs.stream.color)
            colorizer = rs.colorizer()
            
            with buffer.lock:
                buffer.camera_available = True
            
            while True:
                frames = pipeline.wait_for_frames(timeout_ms=1000)
                aligned_frames = align.process(frames)
                depth_f = aligned_frames.get_depth_frame()
                color_f = aligned_frames.get_color_frame()
                
                if not depth_f or not color_f:
                    continue
                
                local_bgr = np.asanyarray(color_f.get_data())
                local_colorized_depth = np.asanyarray(colorizer.colorize(depth_f).get_data())
                local_depth_bgr = cv2.cvtColor(local_colorized_depth, cv2.COLOR_RGB2BGR)

                with buffer.lock:
                    buffer.bgr_frame = local_bgr
                    buffer.depth_frame = local_depth_bgr
                    
        except Exception as e:
            with buffer.lock:
                buffer.camera_available = False
                buffer.camera_error = str(e)

    cam_thread = threading.Thread(target=camera_loop, daemon=True)
    cam_thread.start()

    # Define the local streaming server rules locked onto this specific buffer instance
    class VideoStreamHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/rgb':
                self.send_response(200)
                self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                while True:
                    with buffer.lock:
                        if buffer.bgr_frame is None:
                            time.sleep(0.01)
                            continue
                        _, encoded_img = cv2.imencode('.jpg', buffer.bgr_frame)
                    try:
                        self.wfile.write(b'--frame\r\n')
                        self.send_header('Content-type', 'image/jpeg')
                        self.send_header('Content-length', str(len(encoded_img)))
                        self.end_headers()
                        self.wfile.write(encoded_img.tobytes())
                        self.wfile.write(b'\r\n')
                    except (ConnectionResetError, BrokenPipeError):
                        break
                    time.sleep(0.03)

            elif self.path == '/depth':
                self.send_response(200)
                self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                while True:
                    with buffer.lock:
                        if buffer.depth_frame is None:
                            time.sleep(0.01)
                            continue
                        _, encoded_img = cv2.imencode('.jpg', buffer.depth_frame)
                    try:
                        self.wfile.write(b'--frame\r\n')
                        self.send_header('Content-type', 'image/jpeg')
                        self.send_header('Content-length', str(len(encoded_img)))
                        self.end_headers()
                        self.wfile.write(encoded_img.tobytes())
                        self.wfile.write(b'\r\n')
                    except (ConnectionResetError, BrokenPipeError):
                        break
                    time.sleep(0.03)

    class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
        pass

    # Launch local background web server on Port 8089
    server = ThreadedHTTPServer(('0.0.0.0', 8089), VideoStreamHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    
    return buffer

# Pull our unified persistence frame manager
frame_buffer = initialize_camera_system()

# ==========================================
# 2. ROS 2 TELEMETRY SUB-NODE
# ==========================================
class DistanceSubscriber(Node):
    def __init__(self):
        super().__init__('streamlit_distance_subscriber')
        self.subscription = self.create_subscription(Float32, '/person_distance', self.listener_callback, 10)
        self.current_distance = None

    def listener_callback(self, msg):
        self.current_distance = msg.data

@st.cache_resource
def init_ros():
    if not rclpy.ok():
        rclpy.init()
    node = DistanceSubscriber()
    
    # Safe multi-threaded ROS executor execution
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    return node

ros_node = init_ros()


# ==========================================
# 3. MAIN USER INTERFACE
# ==========================================
st.title("🤖 Roselito Agent API")
st.write("Interact with the guide robot and trigger pre-mapped navigation routes.")

if "robot_pid" not in st.session_state:
    st.session_state.robot_pid = None
if "running_route_pid" not in st.session_state:
    st.session_state.running_route_pid = None

# Status Headers
status_col1, status_col2 = st.columns(2)
with status_col1:
    if st.session_state.robot_pid:
        st.success(f"🤖 Main Robot: RUNNING (PID: {st.session_state.robot_pid})")
    else:
        st.error("🤖 Main Robot: STOPPED")

with status_col2:
    if st.session_state.running_route_pid:
        st.success(f"📍 Navigation Route: ACTIVE (PID: {st.session_state.running_route_pid})")
    else:
        st.error("📍 Navigation Route: INACTIVE")

# Operational Buttons
col_start, col_stop = st.columns(2)
with col_start:
    if st.button("🚀 Start the Robot", use_container_width=True):
        if st.session_state.robot_pid is not None:
            st.warning("The robot is already running! Stop it first before restarting.")
        else:
            st.success("Robot initiation command sent!")
            combined_cmd = (
                "cd ~/Projects/roselito_robot && "
                "source ./scripts/setup.bash && "
                "source /opt/ros/humble/setup.bash && "
                "ros2 launch roselito_ugv_jetson start.launch map:=/home/jetson/Projects/roselito_robot/braga/mapa_lcad > /tmp/roselito_robot_start.log 2>&1"
            )
            process = subprocess.Popen(combined_cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid)
            st.session_state.robot_pid = process.pid
            st.rerun()

with col_stop:
    if st.button("🛑 Emergency Stop", type="primary", use_container_width=True):
        killed_any = False
        if st.session_state.robot_pid is not None:
            try:
                os.killpg(os.getpgid(st.session_state.robot_pid), signal.SIGINT)
                killed_any = True
            except ProcessLookupError:
                pass
            finally:
                st.session_state.robot_pid = None

        if st.session_state.running_route_pid is not None:
            try:
                os.killpg(os.getpgid(st.session_state.running_route_pid), signal.SIGINT)
                killed_any = True
            except ProcessLookupError:
                pass
            finally:
                st.session_state.running_route_pid = None

        if killed_any:
            st.error("Emergency halt signals sent successfully.")
        st.rerun()

st.write("---")

# ==========================================
# 4. ROUTE BUTTONS GRID
# ==========================================
st.subheader("Map Routes Grid")

def run_route(route_path: Path):
    st.info(f"Launching route: {route_path.name}...")
    combined_cmd = (
        "cd ~/Projects/roselito_agent && "
        "source ./install/setup.bash && "
        "source ../roselito_interfaces/install/setup.bash && "
        "source /opt/ros/humble/setup.bash && "
        f"ros2 launch roselito_agent replay_route.launch path:={route_path}  > /tmp/roselito_agent_replay_route.log 2>&1"
    )
    process = subprocess.Popen(combined_cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid)
    st.session_state.running_route_pid = process.pid
    st.rerun()

routes_dir = Path(os.getcwd()).parent 
route_files = [Path(f) for f in glob.glob(os.path.join(routes_dir, "*.pon"))]

if not route_files:
    st.info(f"No `.pon` route files found in `{routes_dir}`.")
else:
    grid_columns = 4
    cols = st.columns(grid_columns)
    is_route_busy = st.session_state.running_route_pid is not None
    
    for idx, route_path in enumerate(sorted(route_files)):
        filename = os.path.basename(route_path)
        display_name = filename.replace(".pon", "").replace("_", " ").title()
        with cols[idx % grid_columns]:
            if st.button(f"📍 {display_name}", key=filename, use_container_width=True, disabled=is_route_busy):
                run_route(Path(routes_dir / route_path))

st.write("---")

# ==========================================
# 5. LIVE FEEDS RENDERING PANEL
# ==========================================
st.subheader("Hardware Feeds & Telemetry")

show_feeds = st.toggle("Enable Live Camera & ROS Monitor", value=False)

if show_feeds:
    col1, col2, col3 = st.columns([2, 2, 1])
    
    with col1:
        st.caption("RGB Feed")
        if frame_buffer.camera_available:
            st.markdown('<img src="http://localhost:8089/rgb" style="width:100%; border-radius:10px;">', unsafe_allow_html=True)
        else:
            st.warning("Camera hardware stream is offline.")
            
    with col2:
        st.caption("Depth Map")
        if frame_buffer.camera_available:
            st.markdown('<img src="http://localhost:8089/depth" style="width:100%; border-radius:10px;">', unsafe_allow_html=True)
        else:
            st.info(f"Reason: {frame_buffer.camera_error or 'Initializing hardware...'}")
            
    with col3:
        st.caption("ROS Topics")
        distance_placeholder = st.empty()

    # Telemetry update loop runs unencumbered
    while show_feeds:
        if ros_node.current_distance is not None:
            distance_placeholder.metric(label="Person Distance", value=f"{ros_node.current_distance:.2f} m")
        else:
            distance_placeholder.info("Waiting for `/person_distance`...")
        time.sleep(0.1)