---
sidebar_position: 1
title: "Chapter 1: Hardware & OS Configuration"
---

# Chapter 1: Hardware & OS Configuration

## Overview

This chapter covers the essential hardware and operating system requirements for implementing Physical AI and humanoid robotics applications. We'll set up your development environment with NVIDIA drivers, CUDA, and ROS 2, following best practices for both simulation and real hardware deployment.

## Learning Objectives

By the end of this chapter, you will be able to:
- Configure a workstation with NVIDIA GPU for AI and robotics development
- Install and configure Ubuntu 22.04 LTS with necessary drivers
- Set up CUDA 12.2+ for GPU acceleration
- Install Docker for containerized development
- Verify your system is ready for the subsequent chapters

## Hardware Requirements

### Workstation Specifications
- **CPU**: Intel i7 or AMD Ryzen 7 (8+ cores recommended)
- **RAM**: 32GB+ (64GB recommended for complex simulations)
- **GPU**: NVIDIA RTX 4070 Ti (12GB VRAM) minimum
  - Recommended: RTX 4080/4090 or RTX A4000/A5000 for professional use
  - For cloud deployment: AWS g5.2xlarge or equivalent
- **Storage**: 1TB+ SSD (NVMe recommended)
- **Network**: Gigabit Ethernet for robot communication

### Alternative Platforms
- **NVIDIA Jetson Orin**: For edge deployment and embedded applications
- **Cloud GPU Instance**: For development without local hardware
- **Robot Platforms**: Unitree G1 (humanoid) or Go2 (quadruped) for real hardware testing

## Ubuntu 22.04 LTS Setup

### Initial System Configuration

First, ensure your system is updated:

```bash
sudo apt update && sudo apt upgrade -y
```

### NVIDIA Driver Installation

For systems with NVIDIA GPUs:

```bash
# Check if NVIDIA GPU is detected
lspci | grep -i nvidia

# Install NVIDIA drivers (version 535 or higher)
sudo apt install nvidia-driver-535

# Reboot to apply changes
sudo reboot
```

After reboot, verify the driver installation:

```bash
nvidia-smi
```

You should see output showing your GPU and the driver version.

### CUDA Toolkit Installation

Download and install CUDA 12.2+:

```bash
# Download CUDA 12.2 installer
wget https://developer.download.nvidia.com/compute/cuda/12.2.0/local_installers/cuda_12.2.0_535.54.03_linux.run

# Run the installer
sudo sh cuda_12.2.0_535.54.03_linux.run
```

Follow the interactive prompts, selecting to install CUDA toolkit but not the driver if you've already installed it.

Add CUDA to your PATH by adding these lines to `~/.bashrc`:

```bash
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

Then reload your environment:

```bash
source ~/.bashrc
```

Verify CUDA installation:

```bash
nvcc --version
```

## Docker Installation

Docker is essential for containerized development and deployment:

```bash
# Update package database
sudo apt update

# Install required packages
sudo apt install ca-certificates curl gnupg lsb-release

# Add Docker's official GPG key
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Set up the Docker repository
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Update package database again
sudo apt update

# Install Docker Engine
sudo apt install docker-ce docker-ce-cli containerd.io docker-compose-plugin

# Add your user to the docker group
sudo usermod -aG docker $USER
```

After adding your user to the docker group, log out and log back in, or run:

```bash
newgrp docker
```

## Verification Steps

Let's verify all components are working:

```bash
# Check NVIDIA driver
nvidia-smi

# Check CUDA
nvcc --version

# Check Docker
docker --version
docker run hello-world
```

## Troubleshooting Common Issues

### NVIDIA Driver Issues
- If `nvidia-smi` fails, ensure Secure Boot is disabled in BIOS
- Check for conflicting drivers: `sudo apt remove nvidia-*` and reinstall

### CUDA Issues
- If CUDA samples fail to compile, ensure gcc version compatibility
- Check CUDA installation path: `ls /usr/local/cuda*`

### Docker Permission Issues
- If Docker commands fail without sudo, ensure you've added your user to the docker group
- Log out and log back in after adding to the group

## Next Steps

With your hardware and OS configured, you're ready to proceed to Chapter 2: The Edge Ecosystem, where we'll explore NVIDIA Jetson platforms for edge AI deployment.

For students using cloud GPU instances, ensure you have SSH access and can forward X11 for GUI applications if needed.