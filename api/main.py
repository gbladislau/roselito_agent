import numpy as np
import pyrealsense2 as rs
from subprocess import run
import subprocess
import os
import signal
import cv2
import streamlit as st
import glob

# 1. ROS 2 Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

st.set_page_config(layout="wide")

# ==========================================
# 1. ROS 2 & CAMERA INITIALIZATION (CACHED)
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

@st.cache_resource
def init_realsense():
    # Returns (pipeline, align, colorizer, success_flag, error_message)
    try:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config)
        align = rs.align(rs.stream.color)
        colorizer = rs.colorizer()
        return pipeline, align, colorizer, True, ""
    except Exception as e:
        return None, None, None, False, str(e)

# Initialize background elements gracefully
ros_node = init_ros()
pipeline, align, colorizer, camera_available, camera_error = init_realsense()


# ==========================================
# 2. MAIN API SECTION (MOST IMPORTANT)
# ==========================================
st.title("🤖 - Roselito Agent API")
st.write("Interact with the guide robot and trigger pre-mapped navigation routes.")

# Core Action Buttons
if "robot_pid" not in st.session_state:
    st.session_state.robot_pid = None
if "running_route_pid" not in st.session_state:
    st.session_state.running_route_pid = None

# Optional: Add a visual status indicator for peace of mind
if st.session_state.robot_pid:
    st.success(f"🤖 Robot Status: RUNNING (PID: {st.session_state.robot_pid})")
else:
    st.error("🤖 Robot Status: STOPPED")


col_start, col_stop = st.columns(2)

with col_start:
    if st.button("🚀 Start the Robot", use_container_width=True):
        if st.session_state.robot_pid is not None:
            st.warning("The robot is already running! Stop it first before restarting.")
        else:
            st.success("Robot initiation command sent!")
            
            # Chain the commands: navigate, source environmental variables, and launch
            combined_cmd = (
                "cd ~/Projects/roselito_robot && "
                "source ./scripts/setup.bash && "
                "source /opt/ros/humble/setup.bash && "
                "ros2 launch roselito_ugv_jetson start.launch map:=/home/jetson/Projects/roselito_robot/braga/mapa_lcad"
            )
            
            # Launch as an asynchronous background process group
            process = subprocess.Popen(
                combined_cmd,
                shell=True,
                executable="/bin/bash",      # Forces Python to use Bash so 'source' works
                preexec_fn=os.setsid         # Creates a new process group ID (PGID) matching the PID
            )
            
            # Save the PID to session state so it survives Streamlit page refreshes
            st.session_state.robot_pid = process.pid
            st.rerun()  # Force a quick rerun to update the UI status immediately

with col_stop:
    if st.button("🛑 Emergency Stop", type="primary", use_container_width=True):
        if st.session_state.robot_pid is not None:
            pid_to_kill = st.session_state.robot_pid
            try:
                # Send SIGINT (Ctrl+C equivalent) to the ENTIRE process group.
                # ROS 2 nodes handle SIGINT beautifully to unregister and park safely.
                os.killpg(os.getpgid(pid_to_kill), signal.SIGINT)
                st.error(f"Emergency Stop triggered! Successfully terminated process group {pid_to_kill}.")
            except ProcessLookupError:
                st.warning("Process group was already closed or dead.")
            except Exception as e:
                st.error(f"Failed to cleanly stop the process: {e}")
            finally:
                # Reset state variables regardless of execution success
                st.session_state.robot_pid = None
                st.rerun()
        if st.session_state.running_route_pid is not None:
            pid_to_kill = st.session_state.running_route_pid
            try:
                # Send SIGINT (Ctrl+C equivalent) to the ENTIRE process group.
                # ROS 2 nodes handle SIGINT beautifully to unregister and park safely.
                os.killpg(os.getpgid(pid_to_kill), signal.SIGINT)
                st.error(f"Emergency Stop triggered! Successfully terminated route group {pid_to_kill}.")
            except ProcessLookupError:
                st.warning("Process group was already closed or dead.")
            except Exception as e:
                st.error(f"Failed to cleanly stop the process: {e}")
            finally:
                # Reset state variables regardless of execution success
                st.session_state.running_route_pid = None
                st.rerun()
        else:
            st.info("No active robot process is currently tracked to stop.")
st.write("---")

# ==========================================
# 3. DYNAMIC ROUTE BUTTONS GRID
# ==========================================
st.subheader("Map Routes Grid")

# Optional: Add a visual status indicator for peace of mind
if st.session_state.running_route_pid:
    st.success(f"🤖 Robot Status: RUNNING ROUTE (PID: {st.session_state.running_route_pid})")
else:
    st.error("🤖 Robot Status: STOPPED ROUTE")

def run_route(route_path):
    st.info(f"Launching route: {os.path.basename(route_path)}...")
    combined_cmd = (
        "cd ~/Projects/roselito_agent && "
        "source ./install/setup.bash && "
        "source ../roselito_interfaces/install/setup.bash && "
        "source /opt/ros/humble/setup.bash && "
        "ros2 launch roselito_agent replay_route.launch path:={route_path}"
    )
            
    # Launch as an asynchronous background process group
    process = subprocess.Popen(
        combined_cmd,
        shell=True,
        executable="/bin/bash",      # Forces Python to use Bash so 'source' works
        preexec_fn=os.setsid         # Creates a new process group ID (PGID) matching the PID
    )
    
    # Save the PID to session state so it survives Streamlit page refreshes
    st.session_state.running_route_pid = process.pid

# Scan for all .pon files in the current working directory
# Change path string if your files live specifically one folder up
routes_dir = os.getcwd() 
route_files = glob.glob(os.path.join(routes_dir, "*.pon"))

if not route_files:
    st.info(f"No `.pon` route files found in `{routes_dir}`.")
else:
    # Build a 4-column wide grid dynamically
    grid_columns = 4
    cols = st.columns(grid_columns)
    
    for idx, route_path in enumerate(sorted(route_files)):
        filename = os.path.basename(route_path)
        display_name = filename.replace(".pon", "").replace("_", " ").title()
        
        # Distribute buttons evenly across columns
        with cols[idx % grid_columns]:
            if st.button(f"📍 {display_name}", key=filename, use_container_width=True, enabled=True if not st.session_state.running_route_pid else False):
                run_route(route_path)

st.write("---")

# ==========================================
# 4. OPTIONAL LIVE STREAM & TELEMETRY SECTION
# ==========================================
st.subheader("Hardware Feeds & Telemetry")

# Toggle activation switch to spin the heavy camera loop
show_feeds = st.toggle("Enable Live Camera & ROS Monitor", value=False)

if show_feeds:
    # Setup live placeholders
    col1, col2, col3 = st.columns([2, 2, 1])
    with col1:
        st.caption("RGB Feed")
        rgb_placeholder = st.empty()
    with col2:
        st.caption("Depth Map")
        depth_placeholder = st.empty()
    with col3:
        st.caption("ROS Topics")
        distance_placeholder = st.empty()

    # Loop execution container
    while show_feeds:
        # A. Always update ROS telemetry, even if camera fails
        rclpy.spin_once(ros_node, timeout_sec=0.005)
        if ros_node.current_distance is not None:
            distance_placeholder.metric(label="Person Distance", value=f"{ros_node.current_distance:.2f} m")
        else:
            distance_placeholder.info("Waiting for `/person_distance`...")

        # B. Handle Camera frames conditionally
        if camera_available:
            try:
                frames = pipeline.wait_for_frames(timeout_ms=20)
                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                
                if depth_frame and color_frame:
                    color_image = np.asanyarray(color_frame.get_data())
                    color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                    
                    colorized_depth = colorizer.colorize(depth_frame)
                    depth_image = np.asanyarray(colorized_depth.get_data())

                    rgb_placeholder.image(color_image_rgb, channels="RGB", use_container_width=True)
                    depth_placeholder.image(depth_image, channels="RGB", use_container_width=True)
            except RuntimeError:
                # Frame dropped, skip iteration safely
                pass
        else:
            rgb_placeholder.warning("Camera connection unavailable.")
            depth_placeholder.info(f"Reason: {camera_error}")
            
            # Since camera isn't updating frames, add a sleep to prevent maxing out CPU during ROS-only mode
            import time
            time.sleep(0.1)