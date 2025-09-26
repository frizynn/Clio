#!/bin/bash

set -e  # Exit on any error

echo "Setting up Clio Docker Environment..."

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to create directory if it doesn't exist
create_dir() {
    if [ ! -d "$1" ]; then
        echo "Creating directory: $1"
        mkdir -p "$1"
    else
        echo "Directory already exists: $1"
    fi
}

# Check prerequisites
echo "Checking prerequisites..."

if ! command_exists docker; then
    echo "Error: Docker is not installed. Please install Docker first."
    echo "Visit: https://docs.docker.com/get-docker/"
    exit 1
fi

if ! command_exists docker-compose; then
    echo "Error: Docker Compose is not installed. Please install Docker Compose first."
    echo "Visit: https://docs.docker.com/compose/install/"
    exit 1
fi

echo "Docker and Docker Compose are installed"

# Create necessary directories
echo "Creating necessary directories..."
create_dir "datasets"
create_dir "clio_workspace"
create_dir "logs"

# Check if apartment dataset exists
if [ ! -d "datasets/apartment" ]; then
    echo "Apartment dataset not found in datasets/apartment"
    echo "Please download the dataset from:"
    echo "https://www.dropbox.com/scl/fo/5bkv8rsa2xvwmvom6bmza/AOc8VW71kuZCgQjcw_REbWA"
    echo "And extract it to the datasets/ directory"
fi

# Check if image already exists and ask user if they want to rebuild
IMAGE_EXISTS=$(docker images -q clio-clio)
if [ ! -z "$IMAGE_EXISTS" ]; then
    echo "Docker image already exists. Do you want to rebuild it? (y/N)"
    read -r response
    if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
        echo "Rebuilding Docker image..."
        docker-compose build --no-cache
    else
        echo "Using existing Docker image"
    fi
else
    echo "Building Docker image for the first time..."
    docker-compose build
fi

# Start the container
echo "Starting Docker container..."
docker-compose up -d

# Wait for container to be ready
echo "Waiting for container to be ready..."
sleep 5

# Check if container is running
if docker ps | grep -q "clio-container"; then
    echo "Container is running!"
    echo ""
    echo "Quick Start Commands:"
    echo ""
    echo "1. Access the container:"
    echo "   docker exec -it clio-container bash"
    echo ""
    echo "2. Run Clio with apartment dataset:"
    echo "   docker exec -it clio-container bash -c 'roscore & sleep 3 && roslaunch clio_ros realsense.launch object_tasks_file:=/datasets/apartment/tasks_apartment.yaml place_tasks_file:=/datasets/apartment/region_tasks_apartment.yaml'"
    echo ""
    echo "3. Play apartment rosbag (in another terminal):"
    echo "   docker exec -it clio-container bash -c 'rosbag play /datasets/apartment/apartment.bag --clock'"
    echo ""
    echo "4. Stop the container:"
    echo "   docker-compose down"
    echo ""
    echo "5. View logs:"
    echo "   docker-compose logs"
    echo ""
else
    echo "Container failed to start. Check logs with:"
    echo "   docker-compose logs"
    exit 1
fi