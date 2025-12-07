---
sidebar_position: 1
title: "Quickstart Guide"
---

# Quickstart Guide: Physical AI & Humanoid Robotics

## Overview

This quickstart guide provides a streamlined path to set up and run your humanoid robot with Physical AI capabilities. Follow these steps to get your robot operational quickly.

## Prerequisites

### Hardware Requirements
- **Computer**: Ubuntu 22.04 LTS with NVIDIA RTX 4070 Ti (12GB VRAM) or better
- **Robot**: Unitree Go2 quadruped or G1 humanoid (for real hardware deployment)
- **Network**: Stable internet connection for AI API access
- **Peripherals**: ReSpeaker USB microphone array, speakers for audio

### Software Prerequisites
```bash
# Install system dependencies
sudo apt update
sudo apt install -y python3-pip python3-dev python3-venv
sudo apt install -y build-essential cmake git
sudo apt install -y ros-humble-desktop
sudo apt install -y nvidia-driver-535 nvidia-utils-535
```

## Installation Steps

### 1. Clone the Repository

```bash
# Create workspace
mkdir -p ~/physical_ai_ws/src
cd ~/physical_ai_ws

# Clone the Physical AI repository
git clone https://github.com/your-org/physical-ai-humanoid.git src/physical_ai
```

### 2. Install NVIDIA Isaac Sim

```bash
# Download Isaac Sim from NVIDIA Developer Portal
# Follow NVIDIA's installation guide for Isaac Sim 4.0+
# Or use the Docker approach:
docker run --gpus all --rm -it \
  --env "ACCEPT_EULA=Y" \
  --env "PRIVACY_CONSENT=Y" \
  nvcr.io/nvidia/isaac-sim:4.0.0
```

### 3. Set Up ROS 2 Packages

```bash
# Navigate to workspace
cd ~/physical_ai_ws

# Install Python dependencies
pip3 install -r src/physical_ai/requirements.txt

# Build ROS 2 packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### 4. Configure API Keys

```bash
# Create environment file
cat << EOF > ~/physical_ai_ws/api_keys.env
OPENAI_API_KEY=your_openai_api_key_here
EOF

# Source the API keys
source ~/physical_ai_ws/api_keys.env
```

## Running the System

### 1. Launch Simulation Environment

```bash
# Terminal 1: Start Isaac Sim
# Run Isaac Sim from your installation

# Terminal 2: Launch the robot simulation
cd ~/physical_ai_ws
source install/setup.bash
ros2 launch physical_ai_simulation humanoid_bringup.launch.py
```

### 2. Start the AI Brain

```bash
# Terminal 3: Launch the AI commander
cd ~/physical_ai_ws
source install/setup.bash
ros2 launch physical_ai_brain llm_commander.launch.py
```

### 3. Start Voice Interface

```bash
# Terminal 4: Start voice processing
cd ~/physical_ai_ws
source install/setup.bash
ros2 run physical_ai_brain voice_listener
```

## Basic Commands

Once the system is running, you can issue voice commands like:

- "Go to the kitchen and find the red mug"
- "Navigate to the living room"
- "Perform a dance routine"
- "Stop all actions"
- "Return to home position"

## The Butler Test

To run the complete "Butler Test" scenario:

```bash
# In a new terminal
cd ~/physical_ai_ws
source install/setup.bash
ros2 run physical_ai_brain butler_test_scenario
```

This will execute the complete scenario:
1. Navigate to kitchen
2. Use VLA (Vision-Language-Action) to identify the red mug
3. Approach and interact with the identified object

## Docker Deployment

For easier deployment, use the provided Docker Compose:

```bash
# Navigate to deployment directory
cd ~/physical_ai_ws/src/physical_ai/deployment

# Start the complete system
docker-compose up -d

# View logs
docker-compose logs -f
```

## Troubleshooting

### Common Issues

1. **CUDA Not Found**:
   ```bash
   # Check CUDA installation
   nvidia-smi
   nvcc --version
   ```

2. **ROS 2 Packages Not Found**:
   ```bash
   # Source the workspace
   cd ~/physical_ai_ws
   source install/setup.bash
   ```

3. **API Key Issues**:
   ```bash
   # Verify API key is set
   echo $OPENAI_API_KEY
   ```

4. **Microphone Not Detected**:
   ```bash
   # Check audio devices
   arecord -l
   # Test microphone
   arecord -D sysdefault:CARD=1 -f cd test.wav
   ```

## Next Steps

### 1. Customize Your Robot
- Modify the URDF in `physical_ai_description/urdf/` for your specific robot
- Adjust controller parameters in `physical_ai_control/config/`
- Tune navigation parameters in `physical_ai_navigation/config/`

### 2. Extend Functionality
- Add new VLA capabilities in `physical_ai_brain/nodes/`
- Create custom behaviors in `physical_ai_behavior/`
- Implement new sensors in `physical_ai_perception/`

### 3. Train Custom Models
- Use the provided training scripts in `physical_ai_training/`
- Collect data with the simulation environment
- Fine-tune models for your specific use cases

## Development Workflow

### Adding New Features

1. Create a new branch:
   ```bash
   git checkout -b feature/new-feature
   ```

2. Develop your feature in the appropriate package

3. Test with simulation:
   ```bash
   # Test your changes
   ros2 launch physical_ai_simulation test_my_feature.launch.py
   ```

4. Create a pull request when ready

### Testing Changes

```bash
# Run unit tests
colcon test
colcon test-result --all

# Test with Isaac Sim
ros2 launch physical_ai_simulation integration_test.launch.py

# Run the Butler Test to verify full functionality
ros2 run physical_ai_brain butler_test_scenario
```

## Performance Optimization

### GPU Utilization
- Monitor GPU usage: `nvidia-smi`
- Ensure CUDA is properly configured
- Use TensorRT for optimized inference when possible

### Real-time Performance
- Use appropriate control frequencies (50-100Hz for control)
- Optimize perception pipelines for your use case
- Consider edge deployment for latency-sensitive applications

## Safety Considerations

⚠️ **IMPORTANT SAFETY NOTES**:
- Always supervise robot operation initially
- Ensure clear operational area before autonomous operation
- Implement emergency stop procedures
- Test extensively in simulation before real-world deployment
- Monitor robot behavior for unexpected actions

## Getting Help

- **Documentation**: Full documentation at `docs/`
- **Issues**: Report problems at the GitHub repository
- **Community**: Join our Discord for real-time help
- **Support**: Enterprise support available for commercial deployments

## Resources

- [Official ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [OpenAI API Documentation](https://platform.openai.com/docs/)
- [Physical AI Community Forum](https://community.physical-ai-book.com)

---

🎉 **Congratulations!** You now have a Physical AI-enabled humanoid robot system operational. Experiment with different voice commands and explore the extensive documentation to customize the system for your specific needs.

For advanced usage and customization, continue with the detailed documentation in the following sections.