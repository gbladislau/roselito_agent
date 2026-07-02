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

# 1. ROS 2 Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

st.set_page_config(layout="wide")

# ==========================================
# GLOBAL THREAD SAFE BUFFER FOR CAMERAS
# ==========================================
class CameraFrameBuffer:
    def __init__(self):
        self.bgr_frame = None
        self.depth_frame = None
        self.lock = threading.Lock()
        self.camera_available = False
        self.camera_error = ""

frame_buffer = CameraFrameBuffer()

# ==========================================
# 1. BACKGROUND MJPEG STREAMING SERVER
# ==========================================
class VideoStreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/rgb':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                with frame_buffer.lock:
                    if frame_buffer.bgr_frame is None:
                        time.sleep(0.01)
                        continue
                    _, encoded_img = cv2.imencode('.jpg', frame_buffer.bgr_frame)
                
                try:
                    self.wfile.write(b'--frame\r\n')
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', str(len(encoded_img)))
                    self.end_headers()
                    self.wfile.write(encoded_img.tobytes())
                    self.wfile.write(b'\r\n')
                except (ConnectionResetError, BrokenPipeError):
                    break
                time.sleep(0.03)  # Cap around ~30 FPS

        elif self.path == '/depth':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                with frame_buffer.lock:
                    if frame_buffer.depth_frame is None:
                        time.sleep(0.01)
                        continue
                    _, encoded_img = cv2.imencode('.jpg', frame_buffer.depth_frame)
                
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
    """Handle camera stream network requests in independent threads."""
    pass

@st.cache_resource
def start_mjpeg_server():
    server = ThreadedHTTPServer(('0.0.0.0', 8089), VideoStreamHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    return server

# ==========================================
# 2. HARDWARE CAPTURE THREAD (REALSENSE)
# ==========================================
@st.cache_resource
def start_camera_thread():
    def camera_loop():
        try:
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            pipeline.start(config)
            align = rs.align(rs.stream.color)
            colorizer = rs.colorizer()
            
            with frame_buffer.lock:
                frame_buffer.camera_available = True
            
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

                with frame_buffer.lock:
                    frame_buffer.bgr_frame = local_bgr
                    frame_buffer.depth_frame = local_depth_bgr
                    
        except Exception as e:
            with frame_buffer.lock:
                frame_buffer.camera_available = False
                frame_buffer.camera_error = str(e)

    cam_thread = threading.Thread(target=camera_loop, daemon=True)
    cam_thread.start()
    return True

# Initialize Background Pipelines
start_mjpeg_server()
start_camera_thread()

# ==========================================
# 3. ROS 2 TELEMETRY SUB-NODE
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
    return DistanceSubscriber()

ros_node = init_ros()


# ==========================================
# 4. MAIN INTERFACE LAYOUT (MOST IMPORTANT)
# ==========================================
st.title("🤖 Roselito Agent API")
st.write("Interact with the guide robot and trigger pre-mapped navigation routes.")

# Initialize the session state for process tracking
if "robot_pid" not in st.session_state:
    st.session_state.robot_pid = None
if "running_route_pid" not in st.session_state:
    st.session_state.running_route_pid = None

# Status Display indicators
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

# Core Action Buttons
col_start, col_stop = st.columns(2)

with col_start:
    if st.button("🚀 Start the Robot", width="stretch"):
        if st.session_state.robot_pid is not None:
            st.warning("The robot is already running! Stop it first before restarting.")
        else:
            st.success("Robot initiation command sent!")
            
            combined_cmd = (
                "cd ~/Projects/roselito_robot && "
                "source ./scripts/setup.bash && "
                "source /opt/ros/humble/setup.bash && "
                "ros2 launch roselito_ugv_jetson start.launch map:=/home/jetson/Projects/roselito_robot/braga/mapa_lcad"
            )
            
            process = subprocess.Popen(
                combined_cmd,
                shell=True,
                executable="/bin/bash",
                preexec_fn=os.setsid
            )
            
            st.session_state.robot_pid = process.pid
            st.rerun()

with col_stop:
    if st.button("🛑 Emergency Stop", type="primary", width="stretch"):
        killed_any = False
        
        # Kill core robot if running
        if st.session_state.robot_pid is not None:
            try:
                os.killpg(os.getpgid(st.session_state.robot_pid), signal.SIGINT)
                st.error(f"Terminated robot process group {st.session_state.robot_pid}.")
                killed_any = True
            except ProcessLookupError:
                st.warning("Robot process group was already dead.")
            finally:
                st.session_state.robot_pid = None

        # Kill active navigation route if running
        if st.session_state.running_route_pid is not None:
            try:
                os.killpg(os.getpgid(st.session_state.running_route_pid), signal.SIGINT)
                st.error(f"Terminated route process group {st.session_state.running_route_pid}.")
                killed_any = True
            except ProcessLookupError:
                st.warning("Route process group was already dead.")
            finally:
                st.session_state.running_route_pid = None

        if not killed_any:
            st.info("No active processes were found to stop.")
        
        st.rerun()

st.write("---")

# ==========================================
# 5. DYNAMIC ROUTE BUTTONS GRID
# ==========================================
st.subheader("Map Routes Grid")

def run_route(route_path):
    st.info(f"Launching route: {os.path.basename(route_path)}...")
    combined_cmd = (
        "cd ~/Projects/roselito_agent && "
        "source ./install/setup.bash && "
        "source ../roselito_interfaces/install/setup.bash && "
        "source /opt/ros/humble/setup.bash && "
        "ros2 launch roselito_agent replay_route.launch path:={route_path}"
    )
            
    process = subprocess.Popen(
        combined_cmd,
        shell=True,
        executable="/bin/bash",
        preexec_fn=os.setsid
    )
    st.session_state.running_route_pid = process.pid
    st.rerun()  # Instantly refresh to lock the route selection grid

# Scan for all .pon files in current working directory
routes_dir = os.getcwd() 
route_files = glob.glob(os.path.join(routes_dir, "*.pon"))

if not route_files:
    st.info(f"No `.pon` route files found in `{routes_dir}`.")
else:
    grid_columns = 4
    cols = st.columns(grid_columns)
    
    # Check if a route is currently tracking as running to handle blocking logic
    is_route_busy = st.session_state.running_route_pid is not None
    
    for idx, route_path in enumerate(sorted(route_files)):
        filename = os.path.basename(route_path)
        display_name = filename.replace(".pon", "").replace("_", " ").title()
        
        with cols[idx % grid_columns]:
            # Fixed Parameter Syntax: changed width="stretch" to width="stretch"
            # Fixed Parameter Syntax: changed enabled=True to disabled=is_route_busy
            if st.button(f"📍 {display_name}", key=filename, width="stretch", disabled=is_route_busy):
                run_route(route_path)

st.write("---")

# ==========================================
# 6. LIVE HARDWARE FEEDS & TELEMETRY
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
            st.info(f"Reason: {frame_buffer.camera_error or 'Not Initialized'}")
            
    with col3:
        st.caption("ROS Topics")
        distance_placeholder = st.empty()

    # Telemetry Update Loop
    # Runs efficiently because video streaming is offloaded completely to the browser thread
    while show_feeds:
        rclpy.spin_once(ros_node, timeout_sec=0.01)
        if ros_node.current_distance is not None:
            distance_placeholder.metric(label="Person Distance", value=f"{ros_node.current_distance:.2f} m")
        else:
            distance_placeholder.info("Waiting for `/person_distance`...")
        time.sleep(0.05)