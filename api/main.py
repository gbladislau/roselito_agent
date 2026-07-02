import numpy as np
import subprocess
import os
import signal
import cv2
import streamlit as st
import glob
import threading
import time
import socket
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

st.set_page_config(layout="wide")

# ==========================================
# AUTOMATIC JETSON NETWORK IP DETECTION
# ==========================================
def get_jetson_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.254.254.254', 1))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip

JETSON_IP = get_jetson_ip()

# ==========================================
# CENTRALIZED STORAGE BUFFER FOR ROS DATA
# ==========================================
class RosDataBuffer:
    def __init__(self):
        self.bgr_frame = None
        self.depth_frame = None
        self.current_distance = None
        self.lock = threading.Lock()
        self.camera_available = False

# ==========================================
# STREAMING SERVER HANDLER
# ==========================================
class VideoStreamHandler(BaseHTTPRequestHandler):
    buffer_ref = None  

    def do_GET(self):
        if self.buffer_ref is None:
            self.send_response(500)
            self.end_headers()
            return

        if self.path == '/rgb':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                img_to_stream = None
                with self.buffer_ref.lock:
                    if self.buffer_ref.bgr_frame is not None:
                        img_to_stream = self.buffer_ref.bgr_frame.copy()
                
                if img_to_stream is None:
                    time.sleep(0.03)
                    continue

                _, encoded_img = cv2.imencode('.jpg', img_to_stream)
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
                img_to_stream = None
                with self.buffer_ref.lock:
                    if self.buffer_ref.depth_frame is not None:
                        img_to_stream = self.buffer_ref.depth_frame.copy()
                
                if img_to_stream is None:
                    time.sleep(0.03)
                    continue

                _, encoded_img = cv2.imencode('.jpg', img_to_stream)
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

# ==========================================
# ROS 2 MASTER SUBSCRIBER NODE
# ==========================================
class StreamlitRosReceiver(Node):
    def __init__(self, data_buffer):
        super().__init__('streamlit_ros_receiver')
        self.buffer = data_buffer
        self.bridge = CvBridge()

        # Telemetry Topic Subscriptions
        self.create_subscription(PointStamped, '/person_position', self.distance_callback, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)

    def distance_callback(self, msg):
        with self.buffer.lock:
            # Capturing x coordinate (forward distance) from the PointStamped message
            self.buffer.current_distance = msg.point.x

    def color_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.buffer.lock:
                self.buffer.bgr_frame = cv_img
                self.buffer.camera_available = True
        except Exception as e:
            self.get_logger().error(f"Failed color frame parse: {e}")

    def depth_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.buffer.lock:
                self.buffer.depth_frame = cv_img
        except Exception as e:
            self.get_logger().error(f"Failed depth frame parse: {e}")

# ==========================================
# UNIFIED & CACHED ROS INFRASTRUCTURE
# ==========================================
@st.cache_resource
def start_ros_and_server():
    shared_buffer = RosDataBuffer()
    VideoStreamHandler.buffer_ref = shared_buffer

    # Initialize ROS 2
    if not rclpy.ok():
        rclpy.init()
    
    ros_node = StreamlitRosReceiver(shared_buffer)
    
    # Run ROS 2 processing in an independent background runner
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    # Spin up image distribution server on Port 8089
    server = ThreadedHTTPServer(('0.0.0.0', 8089), VideoStreamHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    
    return shared_buffer

# Initialize backend pipeline
data_pipeline = start_ros_and_server()

# ==========================================
# MAIN USER INTERFACE LAYOUT
# ==========================================
st.title("🤖 Roselito Agent API")
st.write("Interact with the guide robot and trigger pre-mapped navigation routes.")

if "robot_pid" not in st.session_state:
    st.session_state.robot_pid = None
if "running_route_pid" not in st.session_state:
    st.session_state.running_route_pid = None

# Status Monitors
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

# Operational Actions
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
                "ros2 launch roselito_ugv_jetson start.launch map:=/home/jetson/Projects/roselito_robot/braga/mapa_lcad"
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
# ROUTE BUTTONS GRID
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
    process = subprocess.Popen(combined_cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid)
    st.session_state.running_route_pid = process.pid
    st.rerun()

routes_dir = os.getcwd() 
route_files = glob.glob(os.path.join(routes_dir, "*.pon"))

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
                run_route(route_path)

st.write("---")

# ==========================================
# LIVE FEEDS PANEL (SUBSCRIBER DEPENDENT)
# ==========================================
st.subheader("Hardware Feeds & Telemetry")

show_feeds = st.toggle("Enable Live Camera & ROS Monitor", value=False)

if show_feeds:
    col1, col2, col3 = st.columns([2, 2, 1])
    
    with col1:
        st.caption("RGB Feed")
        if data_pipeline.camera_available:
            st.markdown(f'<img src="http://{JETSON_IP}:8089/rgb" style="width:100%; border-radius:10px;">', unsafe_allow_html=True)
        else:
            st.warning("Waiting for ROS topic `/camera/color/image_raw`...")
            
    with col2:
        st.caption("Depth Map")
        if data_pipeline.camera_available:
            st.markdown(f'<img src="http://{JETSON_IP}:8089/depth" style="width:100%; border-radius:10px;">', unsafe_allow_html=True)
        else:
            st.info("Waiting for ROS topic `/camera/depth/image_raw`...")
            
    with col3:
        st.caption("ROS Topics")
        distance_placeholder = st.empty()

    # Continuous telemetry update loop
    while show_feeds:
        # Safely pull values captured from ROS network loops
        with data_pipeline.lock:
            current_dist = data_pipeline.current_distance

        if current_dist is not None:
            distance_placeholder.metric(label="Person Distance", value=f"{current_dist:.2f} m")
        else:
            distance_placeholder.info("Waiting for `/person_position`...")
        time.sleep(0.1)