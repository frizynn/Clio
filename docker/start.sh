#!/bin/bash

echo "Starting Clio Minimal Docker Environment..."

# Activate ROS
source /opt/ros/noetic/setup.bash
if [ -f "/catkin_ws/devel/setup.bash" ]; then
    source /catkin_ws/devel/setup.bash
fi

echo "ROS Environment activated"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_MASTER_URI: $ROS_MASTER_URI"
echo "Workspace: /catkin_ws"

# Check if we have task files
TASKS_FILE=""
PLACE_TASKS_FILE=""

if [ -f "/datasets/tasks.yaml" ]; then
    TASKS_FILE="/datasets/tasks.yaml"
    echo "Found tasks file: $TASKS_FILE"
fi

if [ -f "/datasets/region_tasks.yaml" ]; then
    PLACE_TASKS_FILE="/datasets/region_tasks.yaml"
    echo "Found place tasks file: $PLACE_TASKS_FILE"
fi

echo ""
echo "Clio Minimal Environment Ready!"
echo ""
echo "IMPORTANT: To use ROS commands, first start the ROS master:"
echo "  roscore &"
echo ""
echo "Then you can use these commands:"
echo "1. Start Clio with RealSense: roslaunch clio_ros realsense.launch"
echo "2. Start Clio with tasks: roslaunch clio_ros realsense.launch object_tasks_file:=$TASKS_FILE place_tasks_file:=$PLACE_TASKS_FILE"
echo "3. Play rosbag: rosbag play /datasets/office.bag --clock"
echo "4. Start RViz: rviz"
echo "5. List topics: rostopic list"
echo "6. Interactive shell: bash"
echo ""

# Start interactive shell by default
exec /bin/bash