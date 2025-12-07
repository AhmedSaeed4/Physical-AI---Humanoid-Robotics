#!/bin/bash

# setup_lab.sh - Automated Ubuntu 22.04 LTS setup for Physical AI & Humanoid Robotics
# This script automates the setup process described in Chapter 1 of the Physical AI book

set -e  # Exit on any error

echo "==========================================="
echo "Physical AI & Humanoid Robotics Lab Setup"
echo "Chapter 1: Hardware & OS Configuration"
echo "==========================================="

# Function to check if running as root
check_root() {
    if [ "$EUID" -ne 0 ]; then
        echo "Please run as root or with sudo"
        exit 1
    fi
}

# Function to check if running on Ubuntu 22.04
check_ubuntu_version() {
    if ! grep -q "Ubuntu 22.04" /etc/os-release; then
        echo "Warning: This script is designed for Ubuntu 22.04 LTS"
        echo "Current OS: $(cat /etc/os-release | grep PRETTY_NAME)"
        read -p "Continue anyway? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
}

# Function to update system
update_system() {
    echo "Updating system packages..."
    apt update && apt upgrade -y
    apt install -y software-properties-common curl wget gnupg lsb-release
}

# Function to install NVIDIA drivers
install_nvidia_drivers() {
    echo "Installing NVIDIA drivers (535 or higher)..."

    # Check if NVIDIA GPU is present
    if lspci | grep -i nvidia > /dev/null; then
        echo "NVIDIA GPU detected, installing drivers..."
        apt install -y nvidia-driver-535
        echo "NVIDIA drivers installed. A reboot will be required."
    else
        echo "No NVIDIA GPU detected. Skipping driver installation."
    fi
}

# Function to install CUDA
install_cuda() {
    echo "Installing CUDA toolkit..."

    # Check if NVIDIA GPU is present
    if lspci | grep -i nvidia > /dev/null; then
        # Download and install CUDA 12.2
        CUDA_FILE="cuda_12.2.0_535.54.03_linux.run"
        if [ ! -f "$CUDA_FILE" ]; then
            echo "Downloading CUDA toolkit..."
            wget https://developer.download.nvidia.com/compute/cuda/12.2.0/local_installers/$CUDA_FILE
        fi

        # Run installer (non-interactive, install only CUDA, not driver)
        sh $CUDA_FILE --silent --toolkit --toolkitpath=/usr/local/cuda-12.2
        rm $CUDA_FILE

        # Add to profile
        echo 'export PATH=/usr/local/cuda-12.2/bin:$PATH' >> /etc/environment
        echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.2/lib64:$LD_LIBRARY_PATH' >> /etc/environment
        echo "CUDA toolkit installed."
    else
        echo "No NVIDIA GPU detected. Skipping CUDA installation."
    fi
}

# Function to install Docker
install_docker() {
    echo "Installing Docker..."

    # Add Docker's official GPG key
    mkdir -p /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg

    # Set up Docker repository
    echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
      $(lsb_release -cs) stable" | tee /etc/apt/sources.list.d/docker.list > /dev/null

    # Install Docker
    apt update
    apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

    # Add current user to docker group
    usermod -aG docker $SUDO_USER
    echo "Docker installed and user added to docker group."
}

# Function to install basic development tools
install_dev_tools() {
    echo "Installing development tools..."

    # Install common development packages
    apt install -y build-essential cmake git vim htop iotop
    apt install -y python3 python3-pip python3-dev python3-venv
    apt install -y openssh-server net-tools
}

# Function to create verification script
create_verification_script() {
    echo "Creating verification script..."

    cat > /tmp/verify_setup.sh << 'EOF'
#!/bin/bash

echo "==========================================="
echo "Physical AI Lab Setup Verification"
echo "==========================================="

echo "Checking NVIDIA driver..."
if command -v nvidia-smi &> /dev/null; then
    nvidia-smi
    echo "✓ NVIDIA driver working"
else
    echo "✗ NVIDIA driver not found"
fi

echo
echo "Checking CUDA..."
if command -v nvcc &> /dev/null; then
    nvcc --version
    echo "✓ CUDA toolkit working"
else
    echo "✗ CUDA toolkit not found"
fi

echo
echo "Checking Docker..."
if command -v docker &> /dev/null; then
    docker --version
    docker run hello-world
    echo "✓ Docker working"
else
    echo "✗ Docker not found"
fi

echo
echo "Checking Python..."
python3 --version
pip3 --version
echo "✓ Python available"

echo
echo "Setup verification complete!"
EOF

    chmod +x /tmp/verify_setup.sh
    mv /tmp/verify_setup.sh /usr/local/bin/verify_physical_ai_setup
    echo "Verification script created: /usr/local/bin/verify_physical_ai_setup"
}

# Main execution
main() {
    echo "Starting Physical AI lab setup..."

    check_root
    check_ubuntu_version
    update_system
    install_nvidia_drivers
    install_cuda
    install_docker
    install_dev_tools
    create_verification_script

    echo
    echo "==========================================="
    echo "Setup complete!"
    echo "==========================================="
    echo
    echo "Important: If NVIDIA drivers were installed, please reboot your system:"
    echo "  sudo reboot"
    echo
    echo "After reboot (or if no drivers were installed), verify the setup:"
    echo "  verify_physical_ai_setup"
    echo
    echo "Then continue with Chapter 2: The Edge Ecosystem"
}

# Run main function if script is executed directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main
fi