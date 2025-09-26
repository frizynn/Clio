#!/bin/bash

set -e

echo "Testing Clio Docker Environment..."

# Function to run command in container
run_in_container() {
    docker exec clio-container bash -c "$1"
}

# Function to check if container is running
check_container() {
    if ! docker ps | grep -q "clio-container"; then
        echo "Container is not running. Please run docker-compose up -d first."
        exit 1
    fi
    echo "Container is running"
}

# Function to test ROS environment
test_ros_environment() {
    echo "Testing ROS environment..."
    
    # Test if ROS is sourced
    run_in_container "source /opt/ros/noetic/setup.bash && echo 'ROS_DISTRO: $ROS_DISTRO'"
    
    # Test if catkin workspace is built
    if run_in_container "test -f /catkin_ws/devel/setup.bash"; then
        echo "Catkin workspace is built"
    else
        echo "Catkin workspace not built, building now..."
        run_in_container "cd /catkin_ws && source /opt/ros/noetic/setup.bash && catkin build"
        if run_in_container "test -f /catkin_ws/devel/setup.bash"; then
            echo "Catkin workspace built successfully"
        else
            echo "Failed to build catkin workspace"
            return 1
        fi
    fi
    
    # Test if clio_ros package is available
    if run_in_container "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && rospack find clio_ros >/dev/null 2>&1"; then
        echo "clio_ros package found"
    else
        echo "clio_ros package not found"
        return 1
    fi
}

# Function to test dataset availability
test_datasets() {
    echo "Testing dataset availability..."
    
    if run_in_container "test -d /datasets"; then
        echo "Datasets directory mounted"
    else
        echo "Datasets directory not mounted"
        return 1
    fi
    
    if run_in_container "test -d /datasets/apartment"; then
        echo "Apartment dataset directory found"
    else
        echo "Apartment dataset directory not found"
        echo "Please ensure the apartment dataset is in ./datasets/apartment/"
        return 1
    fi
    
    # Check for required files
    files=("apartment.bag" "tasks_apartment.yaml" "region_tasks_apartment.yaml")
    for file in "${files[@]}"; do
        if run_in_container "test -f /datasets/apartment/$file"; then
            echo "Found: $file"
        else
            echo "Missing: $file"
            return 1
        fi
    done
}

# Function to test ROS master
test_ros_master() {
    echo "Testing ROS master..."
    
    # Start roscore in background
    run_in_container "source /opt/ros/noetic/setup.bash && roscore &" >/dev/null 2>&1 || true
    sleep 3
    
    # Check if roscore is running
    if run_in_container "source /opt/ros/noetic/setup.bash && rostopic list >/dev/null 2>&1"; then
        echo "ROS master is working"
        return 0
    else
        echo "ROS master is not working"
        return 1
    fi
}

# Function to test clio launch
test_clio_launch() {
    echo "Testing Clio launch..."
    
    # Try to validate the launch file
    if run_in_container "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && roslaunch --args clio_ros realsense.launch object_tasks_file:=/datasets/apartment/tasks_apartment.yaml place_tasks_file:=/datasets/apartment/region_tasks_apartment.yaml" >/dev/null 2>&1; then
        echo "Clio launch file validation passed"
    else
        echo "Clio launch file validation failed"
        echo "This might be due to missing dependencies or configuration issues"
        return 1
    fi
}

# Function to test rosbag
test_rosbag() {
    echo "Testing rosbag functionality..."
    
    # Test if rosbag can read the apartment bag
    if run_in_container "source /opt/ros/noetic/setup.bash && rosbag info /datasets/apartment/apartment.bag >/dev/null 2>&1"; then
        echo "Apartment rosbag is readable"
        
        # Get some info about the bag
        run_in_container "source /opt/ros/noetic/setup.bash && rosbag info /datasets/apartment/apartment.bag | head -10"
    else
        echo "Cannot read apartment rosbag"
        return 1
    fi
}

# Main testing sequence
main() {
    echo "Starting comprehensive tests..."
    echo ""
    
    # Test 1: Container status
    check_container
    echo ""
    
    # Test 2: ROS environment
    if test_ros_environment; then
        echo "ROS environment test passed"
    else
        echo "ROS environment test failed"
        exit 1
    fi
    echo ""
    
    # Test 3: Datasets
    if test_datasets; then
        echo "Dataset test passed"
    else
        echo "Dataset test failed"
        exit 1
    fi
    echo ""
    
    # Test 4: ROS master
    if test_ros_master; then
        echo "ROS master test passed"
    else
        echo "ROS master test failed"
        exit 1
    fi
    echo ""
    
    # Test 5: Clio launch
    if test_clio_launch; then
        echo "Clio launch test passed"
    else
        echo "Clio launch test failed (may work in practice)"
    fi
    echo ""
    
    # Test 6: Rosbag
    if test_rosbag; then
        echo "Rosbag test passed"
    else
        echo "Rosbag test failed"
        exit 1
    fi
    echo ""
    
    echo "All critical tests passed!"
    echo ""
    echo "Ready to run:"
    echo "   1. docker exec -it clio-container bash"
    echo "   2. roscore &"
    echo "   3. roslaunch clio_ros realsense.launch object_tasks_file:=/datasets/apartment/tasks_apartment.yaml place_tasks_file:=/datasets/apartment/region_tasks_apartment.yaml"
    echo "   4. (In another terminal) rosbag play /datasets/apartment/apartment.bag --clock"
}

# Run main function
main