# Mobile Robot DRL Navigation

### A ROS2 framework for DRL autonomous navigation on mobile robots with LiDAR.

<p align="center">
 <img src="media/gitHub_header.png" width="700" />
</p>

We were inspired by the following repository: [tomasvr/turtlebot3_drlnav](https://github.com/tomasvr/turtlebot3_drlnav)


# Nemo Installation Guide 

## System Requirements 

- Ubuntu 22.04
- ROS Humble
- Gazebo 11

## System Update 

First, ensure your system is up to date:


```bash
sudo apt-get update && sudo apt-get upgrade
```

## Install Essential Tools 

Install necessary build tools:


```bash
sudo apt-get install cmake build-essential make git
```

## Setup ROS 2 Sources 

Add the ROS 2 apt repository to your system. First, ensure that the Ubuntu Universe repository is enabled:


```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Add the ROS 2 GPG key with apt:


```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the ROS 2 repository to your sources list:


```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## Install ROS 2 Packages 

Update your apt repository caches:


```bash
sudo apt update
```

Ensure your system is up to date:


```bash
sudo apt upgrade
```

Install ROS 2 Desktop (recommended) or ROS-Base:


```bash
sudo apt install ros-humble-desktop
```

Or for a bare-bones installation:


```bash
sudo apt install ros-humble-ros-base
```

Install development tools:


```bash
sudo apt install ros-dev-tools
```

## Environment Setup 
Source the setup script (replace `.bash` with your shell if you're not using bash):

```bash
source /opt/ros/humble/setup.bash
```

## Install Gazebo 

Install Gazebo:


```bash
curl -sSL http://get.gazebosim.org | sh
```

Install ROS Gazebo packages:


```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

## Install Additional Packages 

Install TurtleBot3 description and other dependencies:


```bash
sudo apt-get install ros-humble-turtlebot3-description
sudo apt install ros-humble-ros-core ros-humble-geometry2
```

Source the ROS setup script again:


```bash
source /opt/ros/humble/setup.bash
```

Install Python pip:


```bash
sudo apt install python3-pip
```

## Install PyTorch 

Install PyTorch with CUDA 11:


```bash
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

## Clone the Project 

Clone the project repository:


```bash
git clone https://github.com/ndium/nemoV2 . && cd nemoV2
```

Install specific numpy version and other dependencies:


```bash
pip uninstall numpy
pip install -r requirements.txt
```

## Install NVIDIA CUDA Toolkit 

Install NVIDIA CUDA toolkit:


```bash
sudo apt install nvidia-cuda-toolkit
```

Verify CUDA installation:


```bash
nvcc -V
nvidia-smi
```

## Install ROS Dep Tools 

Install ROS dependencies:


```bash
sudo apt install python3-rosdep2
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
```

Install Colcon:


```bash
sudo apt update
sudo apt install python3-colcon-common-extensions
source install/setup.bash
```

## Update .bashrc 
Add the following lines to your `~/.bashrc` file, replacing `~/path/to/nemov2/repo` with the path where you cloned the repository:

```bash
source /opt/ros/humble/setup.bash

# ROS2 domain ID for network communication
export ROS_DOMAIN_ID=1

# Workspace directory
WORKSPACE_DIR=~/path/to/turtlebot3_drlnav
export DRLNAV_BASE_PATH=$WORKSPACE_DIR

# Source the workspace
source $WORKSPACE_DIR/install/setup.bash

# Allow Gazebo to find our TurtleBot3 models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$WORKSPACE_DIR/src/turtlebot3_simulations/turtlebot3_gazebo/models

# Select TurtleBot3 model (default: burger, waffle, waffle_pi)
export TURTLEBOT3_MODEL=burger

# Allow Gazebo to find the plugin for moving the obstacles
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$WORKSPACE_DIR/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_drl_world/obstacle_plugin/lib
```

## Compile the Project 

Compile the project:


```bash
colcon build
```

If you encounter any CMake warnings:


```bash
source ~/.bashrc
colcon build
```

## Launch the Project 

Launch the simulation:


```bash
ros2 launch turtlebot3_gazebo turtlebot3_drl_stage4.launch.py
```

Run the environment:


```bash
ros2 run turtlebot3_drl environment
```

Train the agent:


```bash
ros2 run turtlebot3_drl train_agent dqn
```

Run Gazebo goals:


```bash
ros2 run turtlebot3_drl gazebo_goals
```
