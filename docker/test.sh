#!/bin/bash

echo "Testing Clio Minimal Docker Setup..."

# Build the image
echo "Building Docker image..."
docker-compose build

if [ $? -ne 0 ]; then
    echo "❌ Failed to build Docker image"
    exit 1
fi

echo "✅ Docker image built successfully"

# Start the container
echo "Starting container..."
docker-compose up -d

if [ $? -ne 0 ]; then
    echo "❌ Failed to start container"
    exit 1
fi

echo "✅ Container started successfully"

# Wait for container to be ready
sleep 5

# Test basic functionality
echo "Testing basic functionality..."

# Test Python
echo "Testing Python..."
docker exec clio-minimal-container python3 --version
if [ $? -eq 0 ]; then
    echo "✅ Python is working"
else
    echo "❌ Python test failed"
fi

# Test ROS setup
echo "Testing ROS setup..."
docker exec clio-minimal-container bash -c "source /opt/ros/noetic/setup.bash && echo 'ROS_DISTRO:' \$ROS_DISTRO"
if [ $? -eq 0 ]; then
    echo "✅ ROS setup is working"
else
    echo "❌ ROS setup test failed"
fi

# Test ROS master
echo "Testing ROS master..."
docker exec clio-minimal-container bash -c "source /opt/ros/noetic/setup.bash && roscore &" &
sleep 3
docker exec clio-minimal-container bash -c "source /opt/ros/noetic/setup.bash && rostopic list"
if [ $? -eq 0 ]; then
    echo "✅ ROS master is working"
else
    echo "⚠️  ROS master test failed (this is expected if not started manually)"
fi

# Test workspace
echo "Testing workspace..."
docker exec clio-minimal-container ls -la /catkin_ws/
if [ $? -eq 0 ]; then
    echo "✅ Workspace is accessible"
else
    echo "❌ Workspace test failed"
fi

# Test datasets mount
echo "Testing datasets mount..."
docker exec clio-minimal-container ls -la /datasets/
if [ $? -eq 0 ]; then
    echo "✅ Datasets mount is working"
else
    echo "❌ Datasets mount test failed"
fi

echo ""
echo "Docker setup test completed!"
echo ""
echo "To access the container:"
echo "  docker exec -it clio-minimal-container bash"
echo ""
echo "To stop the container:"
echo "  docker-compose down"
