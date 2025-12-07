# Quickstart Guide: Physical AI & Humanoid Robotics Book

## Overview
This guide will help you set up your development environment to follow along with the "Embodied Intelligence: Physical AI & Humanoid Robotics" book. The book content is available in the frontend/docs/ directory of this repository, with code examples in the ROS 2 workspace. The setup process covers both simulation and real hardware environments.

## Prerequisites

### Hardware Requirements
- **Workstation**: NVIDIA RTX 4070 Ti (12GB VRAM) minimum, with Ubuntu 22.04 LTS
- **Alternative**: Cloud instance with GPU (AWS g5.2xlarge or equivalent)
- **Robot Platform**: Unitree Go2 (quadruped) or Unitree G1 (humanoid) - for real hardware testing
- **Optional Sensors**: Intel RealSense D435i, RPLIDAR A1, ReSpeaker USB Mic Array

### Software Requirements
- Ubuntu 22.04 LTS (Jammy Jellyfish)
- NVIDIA GPU with drivers >= 535
- CUDA 12.2
- Docker and Docker Compose

## Environment Setup

### Step 1: Install NVIDIA Drivers and CUDA
```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install NVIDIA drivers (if not already installed)
sudo apt install nvidia-driver-535

# Reboot to apply driver changes
sudo reboot

# Install CUDA 12.2
wget https://developer.download.nvidia.com/compute/cuda/12.2.0/local_installers/cuda_12.2.0_535.54.03_linux.run
sudo sh cuda_12.2.0_535.54.03_linux.run

# Add CUDA to PATH (add to ~/.bashrc)
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Install ROS 2 Humble Hawksbill
```bash
# Set locale
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y python3-rosdep python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 (add to ~/.bashrc)
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### Step 3: Create ROS 2 Workspace
```bash
# Create workspace directory
mkdir -p ~/physical_ai_ws/src
cd ~/physical_ai_ws

# Build the workspace (initial empty build)
colcon build --symlink-install
source install/setup.bash
```

### Step 4: Clone Book Code Examples
```bash
cd ~/physical_ai_ws/src

# Clone the book's example repositories
git clone https://github.com/your-organization/physical_ai_bringup.git
git clone https://github.com/your-organization/physical_ai_description.git
git clone https://github.com/your-organization/physical_ai_simulation.git
git clone https://github.com/your-organization/physical_ai_perception.git
git clone https://github.com/your-organization/physical_ai_navigation.git
git clone https://github.com/your-organization/physical_ai_brain.git

# Build the workspace with all packages
cd ~/physical_ai_ws
colcon build --symlink-install
source install/setup.bash
```

### Step 5: Install Simulation Environments
```bash
# Install Gazebo Harmonic
sudo apt install -y ros-humble-gazebo-*

# Install Isaac Sim (download from NVIDIA developer portal)
# Follow NVIDIA's installation guide for Isaac Sim 4.0+

# Verify Gazebo installation
gz sim --version
```

### Step 6: Install AI Dependencies
```bash
# Install Python dependencies
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
pip3 install openai
pip3 install transformers
pip3 install sentencepiece

# Install ROS 2 AI packages
sudo apt install -y ros-humble-isaac-ros-* ros-humble-nav2-*
```

### Step 7: Install Additional Tools
```bash
# Install Docker
sudo apt update
sudo apt install ca-certificates curl gnupg lsb-release
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-compose-plugin

# Add user to docker group
sudo usermod -aG docker $USER
```

## Running Your First Example

### Launch the Robot Simulation
```bash
cd ~/physical_ai_ws
source install/setup.bash

# Launch the basic robot simulation
ros2 launch physical_ai_bringup robot_simulation.launch.py
```

### Test Basic ROS 2 Communication
```bash
# In a new terminal, source the workspace
cd ~/physical_ai_ws
source install/setup.bash

# List available topics
ros2 topic list

# Echo the robot's joint states
ros2 topic echo /joint_states sensor_msgs/msg/JointState
```

### Launch Navigation Example
```bash
# In a new terminal
cd ~/physical_ai_ws
source install/setup.bash

# Launch navigation stack
ros2 launch physical_ai_navigation navigation.launch.py

# Send a navigation goal (in another terminal)
ros2 run nav2_msgs navigate_to_pose.py --x 1.0 --y 1.0 --yaw 0.0
```

## Troubleshooting

### Common Issues and Solutions

1. **CUDA not found**: Ensure CUDA is properly installed and added to PATH
   ```bash
   nvidia-smi
   nvcc --version
   ```

2. **ROS 2 packages not found**: Make sure to source the setup.bash file
   ```bash
   source ~/physical_ai_ws/install/setup.bash
   ```

3. **Gazebo fails to start**: Check if GPU drivers are properly installed
   ```bash
   glxinfo | grep -i nvidia
   ```

4. **Permission denied for Docker**: Log out and log back in after adding user to docker group

## Next Steps

After completing this quickstart guide, you're ready to begin with Chapter 1 of the book. The first chapter will guide you through creating your first ROS 2 node and understanding the basic communication patterns.

Continue with:
1. Chapter 1: Hardware & OS Configuration
2. Chapter 2: The Edge Ecosystem (for Jetson setup)
3. Chapter 3: ROS 2 Architecture