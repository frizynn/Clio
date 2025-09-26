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