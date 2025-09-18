# Dockerfile for Clio with ROS Noetic
FROM osrf/ros:noetic-desktop-full

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-catkin-tools \
    python3-vcstool \
    python3-virtualenv \
    git \
    wget \
    curl \
    build-essential \
    cmake \
    libeigen3-dev \
    libzmq3-dev \
    pkg-config \
    ninja-build \
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
    && rm -rf /var/lib/apt/lists/*

# Install additional packages via pip
RUN pip3 install --upgrade pip
RUN pip3 install \
    cppzmq \
    nlohmann-json \
    distinctipy \
    torch==2.0.1 \
    torchvision==0.15.2 \
    opencv-python \
    matplotlib \
    numpy \
    scipy \
    pandas \
    seaborn \
    distinctipy \
    tabulate \
    tqdm \
    click \
    networkx \
    pyyaml

# Configurar ROS
RUN rosdep init && rosdep update

# Crear workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Configurar catkin
RUN catkin init
RUN catkin config -DCMAKE_BUILD_TYPE=Release
RUN catkin config --skiplist khronos_eval

# Clonar Clio
WORKDIR /catkin_ws/src
RUN git clone https://github.com/MIT-SPARK/Clio.git clio --recursive
RUN vcs import . < clio/install/clio.rosinstall

# Instalar dependencias de ROS
RUN rosdep install --from-paths . --ignore-src -r -y

# Construir el workspace
WORKDIR /catkin_ws
RUN catkin build

# Configurar entornos Python
RUN python3 -m virtualenv --system-site-packages -p /usr/bin/python3 /environments/clio_ros
RUN python3 -m virtualenv --download -p /usr/bin/python3 /environments/clio

# Instalar dependencias de Python
RUN /bin/bash -c "source /environments/clio_ros/bin/activate && pip install --upgrade pip"
RUN /bin/bash -c "source /environments/clio_ros/bin/activate && pip install torch==2.0.1 torchvision==0.15.2 --index-url https://download.pytorch.org/whl/cpu"
RUN /bin/bash -c "source /environments/clio_ros/bin/activate && pip install opencv-python matplotlib numpy scipy pandas seaborn distinctipy tabulate tqdm click networkx pyyaml"

# Instalar Clio
RUN /bin/bash -c "source /environments/clio/bin/activate && pip install -e /catkin_ws/src/clio"

# Crear directorio para datasets
RUN mkdir -p /datasets

# Script de inicio
COPY start_clio.sh /start_clio.sh
RUN chmod +x /start_clio.sh

# Exponer puertos para ROS
EXPOSE 11311

# Comando por defecto
CMD ["/start_clio.sh"]
