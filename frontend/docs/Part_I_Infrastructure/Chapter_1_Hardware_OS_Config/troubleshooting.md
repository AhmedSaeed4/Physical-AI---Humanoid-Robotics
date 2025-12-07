---
sidebar_position: 3
title: "Troubleshooting Guide"
---

# Troubleshooting Guide

## Overview

This guide addresses common issues encountered during the setup of the Physical AI development environment. Each section includes the problem description, potential causes, and step-by-step solutions.

## NVIDIA Driver Issues

### Problem: `nvidia-smi` command not found
**Symptoms**: Command returns "command not found" error

**Causes**:
- NVIDIA drivers not installed
- Drivers installed but not in PATH
- Incompatible driver version

**Solutions**:
1. Check if NVIDIA GPU is detected:
   ```bash
   lspci | grep -i nvidia
   ```
2. If GPU is detected but no driver, install drivers:
   ```bash
   sudo apt install nvidia-driver-535
   sudo reboot
   ```
3. If driver was installed but still not found, check if Secure Boot is enabled in BIOS and disable it.

### Problem: GPU not detected after driver installation
**Symptoms**: `nvidia-smi` shows no devices or system uses integrated graphics

**Causes**:
- Secure Boot enabled
- Nouveau driver conflicts
- Hardware issues

**Solutions**:
1. Disable Secure Boot in BIOS settings
2. If Nouveau driver is loaded, blacklist it:
   ```bash
   echo 'blacklist nouveau' | sudo tee -a /etc/modprobe.d/blacklist-nouveau.conf
   echo 'options nouveau modeset=0' | sudo tee -a /etc/modprobe.d/blacklist-nouveau.conf
   sudo update-initramfs -u
   sudo reboot
   ```
3. Check hardware connections and power supply

### Problem: Driver installation fails
**Symptoms**: apt install command fails with dependency errors

**Solutions**:
1. Update system packages:
   ```bash
   sudo apt update && sudo apt upgrade
   ```
2. Clean up previous installations:
   ```bash
   sudo apt remove --purge nvidia-*
   sudo apt autoremove
   sudo apt autoclean
   ```
3. Reinstall:
   ```bash
   sudo apt install nvidia-driver-535
   ```

## CUDA Issues

### Problem: CUDA samples compilation fails
**Symptoms**: `make` command in CUDA samples directory fails

**Causes**:
- GCC version incompatibility
- Missing development tools
- Incorrect CUDA installation

**Solutions**:
1. Check GCC version compatibility:
   ```bash
   gcc --version
   ```
   CUDA 12.2 supports GCC up to version 11.
2. Install build essentials:
   ```bash
   sudo apt install build-essential
   ```
3. Verify CUDA installation:
   ```bash
   nvcc --version
   which nvcc
   ```

### Problem: CUDA runtime error
**Symptoms**: Applications fail with CUDA runtime errors

**Solutions**:
1. Check CUDA and driver compatibility:
   ```bash
   nvidia-smi
   nvcc --version
   ```
   Ensure CUDA version is supported by your driver version.
2. Set proper environment variables:
   ```bash
   export PATH=/usr/local/cuda/bin:$PATH
   export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
   ```
3. Add these to `~/.bashrc` to make permanent:
   ```bash
   echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
   echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
   source ~/.bashrc
   ```

## Docker Issues

### Problem: Docker commands require sudo
**Symptoms**: Docker commands fail without sudo, but work with sudo

**Causes**: User not in docker group

**Solutions**:
1. Add user to docker group:
   ```bash
   sudo usermod -aG docker $USER
   ```
2. Log out and log back in, or run:
   ```bash
   newgrp docker
   ```
3. Test Docker without sudo:
   ```bash
   docker run hello-world
   ```

### Problem: Docker daemon not starting
**Symptoms**: `systemctl status docker` shows failed status

**Solutions**:
1. Restart Docker service:
   ```bash
   sudo systemctl restart docker
   ```
2. Check if service is enabled:
   ```bash
   sudo systemctl enable docker
   ```
3. If issues persist, reinstall Docker:
   ```bash
   sudo apt remove docker docker-engine docker.io containerd runc
   # Then reinstall following the installation guide
   ```

## ROS 2 Installation Issues

### Problem: ROS 2 packages not found
**Symptoms**: `ros2` command not found or packages not recognized

**Solutions**:
1. Source the ROS 2 setup file:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. Add to `~/.bashrc` for persistence:
   ```bash
   echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
   source ~/.bashrc
   ```
3. Check installation:
   ```bash
   apt list --installed | grep ros-humble
   ```

### Problem: Colcon build fails
**Symptoms**: `colcon build` command fails with compilation errors

**Solutions**:
1. Check ROS 2 environment:
   ```bash
   printenv | grep -i ros
   ```
2. Ensure workspace is properly set up:
   ```bash
   cd ~/physical_ai_ws
   source /opt/ros/humble/setup.bash
   ```
3. Install missing dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

## Network and Connectivity Issues

### Problem: Cannot connect to robot
**Symptoms**: Network timeouts when trying to connect to robot

**Solutions**:
1. Check network configuration:
   ```bash
   ip addr show
   ```
2. Verify robot is on same network:
   ```bash
   ping <robot_ip_address>
   ```
3. Check firewall settings:
   ```bash
   sudo ufw status
   ```
4. If using ROS 2 over network, ensure RMW implementation is consistent between machines.

### Problem: SSH connection to robot fails
**Symptoms**: SSH hangs or returns "connection refused"

**Solutions**:
1. Check if SSH server is running on robot:
   ```bash
   sudo systemctl status ssh
   ```
2. Verify IP address and SSH port (default 22):
   ```bash
   ssh -v -p 22 user@robot_ip
   ```
3. Check for IP conflicts or network configuration issues.

## Performance Issues

### Problem: GPU not being utilized
**Symptoms**: GPU usage shows 0% during AI workloads

**Solutions**:
1. Check if CUDA is properly detecting GPU:
   ```bash
   nvidia-smi
   ```
2. Verify application is configured to use GPU:
   ```bash
   # For PyTorch
   python3 -c "import torch; print(torch.cuda.is_available()); print(torch.cuda.device_count())"
   ```
3. Check for memory issues:
   ```bash
   nvidia-smi -q -d MEMORY
   ```

### Problem: High CPU usage during simple tasks
**Symptoms**: CPU usage remains high during operations that should be GPU-accelerated

**Solutions**:
1. Verify GPU is being used instead of CPU:
   ```bash
   # Monitor GPU usage while running application
   watch -n 1 nvidia-smi
   ```
2. Check application configuration for hardware acceleration settings
3. Update drivers and CUDA toolkit to latest versions

## Simulation Environment Issues

### Problem: Gazebo fails to start
**Symptoms**: Gazebo crashes immediately or shows black screen

**Solutions**:
1. Check graphics drivers:
   ```bash
   glxinfo | grep -i nvidia
   ```
2. Set proper environment variables:
   ```bash
   export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:$GAZEBO_RESOURCE_PATH
   export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH
   ```
3. Run with software rendering if needed:
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   gz sim
   ```

### Problem: Isaac Sim crashes on startup
**Symptoms**: Isaac Sim application closes immediately after launch

**Solutions**:
1. Check system requirements are met
2. Verify NVIDIA drivers support Isaac Sim
3. Run from command line to see error messages:
   ```bash
   /opt/isaac-sim/python.sh
   ```
4. Check available VRAM and system memory

## Common Error Messages and Solutions

### "No module named ros2"
**Solution**: Source ROS 2 setup script or reinstall ROS 2 packages

### "Permission denied" for ROS 2 commands
**Solution**: Check user permissions and ensure proper environment setup

### "ImportError: No module named torch"
**Solution**: Install PyTorch with CUDA support:
```bash
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

### "Could not load dynamic library 'libcudart.so'"
**Solution**: Add CUDA libraries to LD_LIBRARY_PATH or reinstall CUDA

## System Resource Issues

### Problem: Out of memory errors
**Symptoms**: Applications crash with memory allocation errors

**Solutions**:
1. Check available memory:
   ```bash
   free -h
   htop
   ```
2. Increase swap space if needed:
   ```bash
   sudo fallocate -l 4G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   ```
3. Close unnecessary applications to free memory

### Problem: High disk usage
**Symptoms**: System reports low disk space

**Solutions**:
1. Check disk usage:
   ```bash
   df -h
   ```
2. Clean Docker:
   ```bash
   docker system prune -a
   ```
3. Remove unused packages:
   ```bash
   sudo apt autoremove
   sudo apt autoclean
   ```

## Getting Help

If you encounter issues not covered in this guide:

1. Check the [ROS 2 documentation](https://docs.ros.org/en/humble/)
2. Visit the [NVIDIA Developer Forums](https://forums.developer.nvidia.com/)
3. Search for your error message on [ROS Answers](https://answers.ros.org/questions/)
4. Create a detailed issue on the [Physical AI GitHub repository](https://github.com/AhmedSaeed4) with:
   - System specifications
   - Exact error messages
   - Steps to reproduce
   - What you've already tried

Remember to include your system information when seeking help:
```bash
# Collect system information
echo "OS: $(lsb_release -d)"
echo "Kernel: $(uname -r)"
nvidia-smi
nvcc --version
docker --version
```