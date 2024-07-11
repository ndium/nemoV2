FROM nvidia/cuda:11.7.1-base-ubuntu22.04

WORKDIR /home

# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y tzdata && \
    rm -rf /var/lib/apt/lists/*

# --- PYTHON3 INSTALLATION ---

# Install Python 3.10
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository -y ppa:deadsnakes/ppa && \
    apt-get update && apt-get install -y python3.10 && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1

# Install pip and virtualenv
RUN apt-get install -y python3-pip && \
    python3 -m pip install --upgrade pip && \
    python3 -m pip install virtualenv

# Create a virtual environment
RUN python3 -m virtualenv /home/venv

# Install Python packages within the virtual environment
RUN /home/venv/bin/pip install \
    matplotlib \
    pandas \
    pyqtgraph \
    PyQt5 \
    torch torchvision torchaudio -f https://download.pytorch.org/whl/cu118 \
    norse

# --- GAZEBO INSTALLATION ---

RUN apt-get update && apt-get install -y wget

# Install Gazebo
RUN curl -sSL http://get.gazebosim.org | sh

# Download basic Gazebo models manually instead of complete (slow) download
ENV GAZEBO_MODEL_DATABASE_URI=""
RUN mkdir -p /root/.gazebo/models
WORKDIR /root/.gazebo/models
RUN wget https://raw.githubusercontent.com/osrf/gazebo_models/master/ground_plane/model.sdf -P ./ground_plane && \
    wget https://raw.githubusercontent.com/osrf/gazebo_models/master/ground_plane/model.config -P ./ground_plane && \
    wget https://raw.githubusercontent.com/osrf/gazebo_models/master/sun/model.sdf -P ./sun && \
    wget https://raw.githubusercontent.com/osrf/gazebo_models/master/sun/model.config -P ./sun

# --- ROS2 humble INSTALLATION ---

# Set locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Add ROS 2 apt repository
RUN apt-get update && apt-get install -y software-properties-common curl && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
RUN apt-get update && apt-get install -y ros-humble-ros-base python3-argcomplete

RUN apt-get update && apt-get install -y python3-tk && \
    apt-get install -y ros-humble-turtlebot3-description ros-humble-gazebo-ros-pkgs && \
    apt-get update && apt-get install -y ros-dev-tools

RUN apt-get update && apt-get install -y python3-rosdep2 && \
    rosdep update

# Source ROS 2 setup script
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Clean up
RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

CMD ["/bin/bash"]


