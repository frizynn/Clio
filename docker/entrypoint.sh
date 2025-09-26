#!/bin/bash
set -e

echo "Starting Clio Docker Environment..."

# Source ROS environment
source /opt/ros/noetic/setup.bash

# Source catkin workspace if it exists
if [ -f "/catkin_ws/devel/setup.bash" ]; then
    source /catkin_ws/devel/setup.bash
    echo "Catkin workspace sourced successfully"
else
    echo "Catkin workspace not built yet. Building now..."
    cd /catkin_ws
    catkin build || catkin_make
    if [ -f "/catkin_ws/devel/setup.bash" ]; then
        source /catkin_ws/devel/setup.bash
        echo "Catkin workspace built and sourced"
    else
        echo "Failed to build catkin workspace"
    fi
fi

# Optional: Start noVNC-based web viewer
if [ "${ENABLE_NOVNC}" = "true" ] || [ "${ENABLE_NOVNC}" = "1" ]; then
    echo "Enabling noVNC web viewer (DISPLAY :1)"
    export DISPLAY=:1
    # Start a virtual framebuffer X server
    Xvfb :1 -screen 0 1280x800x24 -ac +extension GLX +render -noreset &
    # Start a lightweight window manager (optional but recommended)
    fluxbox >/dev/null 2>&1 &
    # Start VNC server on the virtual display
    x11vnc -display :1 -forever -shared -rfbport 5900 -nopw -quiet >/dev/null 2>&1 &
    # Serve noVNC on port 6080
    if [ -d "/usr/share/novnc" ]; then
        websockify --web=/usr/share/novnc/ 6080 localhost:5900 >/dev/null 2>&1 &
        echo "noVNC available at: http://localhost:6080/vnc.html?autoconnect=1"
    else
        echo "noVNC not found at /usr/share/novnc. Ensure 'novnc' is installed."
    fi
else
    echo "noVNC disabled. Using DISPLAY=${DISPLAY:-:0}"
fi

echo "ROS Environment:"
echo "   ROS_DISTRO: $ROS_DISTRO"
echo "   ROS_MASTER_URI: $ROS_MASTER_URI"
echo "   Workspace: /catkin_ws"

# Check if datasets are available
if [ -d "/datasets" ]; then
    echo "Datasets directory mounted: /datasets"
    if [ -d "/datasets/apartment" ]; then
        echo "   Apartment dataset found"
        if [ -f "/datasets/apartment/apartment.bag" ]; then
            echo "   apartment.bag available"
        fi
        if [ -f "/datasets/apartment/tasks_apartment.yaml" ]; then
            echo "   tasks_apartment.yaml available"
        fi
        if [ -f "/datasets/apartment/region_tasks_apartment.yaml" ]; then
            echo "   region_tasks_apartment.yaml available"
        fi
    fi
fi

echo ""
echo "Available Commands:"
echo "   1. Start ROS master:           roscore &"
echo "   2. Launch Clio with apartment: roslaunch clio_ros realsense.launch object_tasks_file:=/datasets/apartment/tasks_apartment.yaml place_tasks_file:=/datasets/apartment/region_tasks_apartment.yaml"
echo "   3. Play apartment rosbag:      rosbag play /datasets/apartment/apartment.bag --clock"
echo "   4. Start RViz:                 rviz"
echo "   5. List ROS topics:            rostopic list"
echo "   6. Interactive shell:          bash"
echo ""

# Execute the provided command or start an interactive shell
if [ $# -eq 0 ]; then
    echo "Starting interactive shell..."
    exec bash
else
    echo "Executing: $@"
    exec "$@"
fi