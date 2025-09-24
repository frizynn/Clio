# Minimal Dockerfile for Clio with ROS Noetic
FROM --platform=$BUILDPLATFORM osrf/ros:noetic-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV PYTHONUNBUFFERED=1

# Install only essential system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-catkin-tools \
    python3-vcstool \
    git \
    wget \
    curl \
    build-essential \
    cmake \
    libeigen3-dev \
    libzmq3-dev \
    pkg-config \
    libopencv-dev \
    python3-opencv \
    python3-matplotlib \
    python3-numpy \
    python3-scipy \
    python3-yaml \
    python3-networkx \
    python3-tqdm \
    python3-click \
    python3-pandas \
    python3-seaborn \
    python3-tabulate \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Install Python packages with specific versions
RUN pip3 install --upgrade pip && \
    pip3 install --no-cache-dir \
    torch==2.0.1 \
    torchvision==0.15.2 \
    opencv-python \
    matplotlib \
    numpy==1.23.5 \
    scipy \
    pandas \
    seaborn \
    distinctipy \
    tabulate \
    tqdm \
    click \
    networkx \
    pyyaml \
    open3d \
    open-clip-torch

# Configure ROS
RUN rosdep init || true && rosdep update

# Create workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Configure catkin with minimal settings
RUN catkin init && \
    catkin config -DCMAKE_BUILD_TYPE=Release && \
    catkin config --skiplist khronos_eval

# Copy the project
COPY . /catkin_ws/src/clio/

# Install ROS dependencies (skip if they fail)
RUN rosdep install --from-paths src --ignore-src -r -y || echo "Some dependencies failed, continuing..."

# Try to build (continue even if it fails)
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build" || echo "Build failed, continuing with basic setup"

# Create necessary directories
RUN mkdir -p /datasets /environments

# Copy startup script
COPY docker/start.sh /start.sh
RUN chmod +x /start.sh

# Expose ports
EXPOSE 11311

# Set working directory
WORKDIR /catkin_ws

# Default command
CMD ["/start.sh"]
