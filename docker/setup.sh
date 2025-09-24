#!/bin/bash

echo "Setting up Clio Minimal Docker Environment..."

# Create necessary directories
mkdir -p datasets
mkdir -p clio_workspace

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed. Please install Docker first."
    exit 1
fi

# Check if Docker Compose is installed
if ! command -v docker-compose &> /dev/null; then
    echo "Error: Docker Compose is not installed. Please install Docker Compose first."
    exit 1
fi

echo "Building minimal Docker image..."
docker-compose build

if [ $? -eq 0 ]; then
    echo "✅ Docker image built successfully!"
    echo ""
    echo "To run the container:"
    echo "  docker-compose up"
    echo ""
    echo "To run in detached mode:"
    echo "  docker-compose up -d"
    echo ""
    echo "To stop the container:"
    echo "  docker-compose down"
    echo ""
    echo "To access the container shell:"
    echo "  docker exec -it clio-minimal-container bash"
else
    echo "❌ Failed to build Docker image"
    exit 1
fi
