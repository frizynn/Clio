#!/bin/bash

echo "Starting Clio with ROS..."

# Activate ROS
source /opt/ros/noetic/setup.bash
if [ -f "/catkin_ws/devel/setup.bash" ]; then
    source /catkin_ws/devel/setup.bash
fi

# Activate Python environment (if exists)
if [ -f "/environments/clio_ros/bin/activate" ]; then
    source /environments/clio_ros/bin/activate
fi

echo "Environments activated"
echo "Workspace: /catkin_ws"
echo "Python: $(which python)"
echo "ROS: $ROS_DISTRO"

# Check if task file exists
if [ -f "/datasets/tasks.yaml" ]; then
    echo "Using task file: /datasets/tasks.yaml"
    TASKS_FILE="/datasets/tasks.yaml"
else
    echo "No task file found, using default configuration"
    TASKS_FILE=""
fi

# Check if place task file exists
if [ -f "/datasets/region_tasks.yaml" ]; then
    echo "Using place task file: /datasets/region_tasks.yaml"
    PLACE_TASKS_FILE="/datasets/region_tasks.yaml"
else
    echo "No place task file found, using default configuration"
    PLACE_TASKS_FILE=""
fi

echo ""
echo "Available options:"
echo "1. Start Clio with RealSense camera (if available)"
echo "2. Start Clio with example data"
echo "3. Enter interactive shell"
echo ""

# Function to start Clio with camera
start_realsense() {
    echo "Starting Clio with RealSense camera..."
    if [ -n "$TASKS_FILE" ] && [ -n "$PLACE_TASKS_FILE" ]; then
        roslaunch clio_ros realsense.launch \
            object_tasks_file:=$TASKS_FILE \
            place_tasks_file:=$PLACE_TASKS_FILE
    else
        roslaunch clio_ros realsense.launch
    fi
}

# Function to start Clio with example data
start_example() {
    echo "Starting Clio with example data..."
    if [ -f "/datasets/office.bag" ]; then
        echo "Playing rosbag: /datasets/office.bag"
        # Play the rosbag in background
        rosbag play /datasets/office.bag --clock &
        sleep 2
    fi
    
    if [ -n "$TASKS_FILE" ] && [ -n "$PLACE_TASKS_FILE" ]; then
        roslaunch clio_ros realsense.launch \
            object_tasks_file:=$TASKS_FILE \
            place_tasks_file:=$PLACE_TASKS_FILE
    else
        roslaunch clio_ros realsense.launch
    fi
}

# Function for interactive shell
start_shell() {
    echo "Starting interactive shell..."
    echo "Useful commands:"
    echo "  roslaunch clio_ros realsense.launch"
    echo "  rosbag play /datasets/office.bag --clock"
    echo "  rviz"
    echo "  rostopic list"
    echo ""
    exec /bin/bash
}

# Interactive menu
echo "Select an option (1-3):"
read -p "> " choice

case $choice in
    1)
        start_realsense
        ;;
    2)
        start_example
        ;;
    3)
        start_shell
        ;;
    *)
        echo "Invalid option, starting interactive shell..."
        start_shell
        ;;
esac
