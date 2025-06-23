# VLA-LeRobot Workspace

## Overview

This workspace integrates **LeRobot** and **Isaac-GR00T** for Vision-Language-Action (VLA) model development, inference and deployment on NVIDIA Jetson Orin AGX.

## Quick Start

### 1. Create Workspace

```bash
mkdir vla-lerobot && cd vla-lerobot
```

### 2. Clone Required Repositories

Clone the LeRobot and Isaac-GR00T repositories into the current directory:

```bash
# Clone LeRobot
git clone https://github.com/huggingface/lerobot.git

# Clone Isaac-GR00T  
git clone https://github.com/NVlabs/Isaac-GR00T.git
```

Your directory structure should look like:
```
vla-lerobot/
├── lerobot/
├── Isaac-GR00T/
└── pyproject.toml
```

### 3. Install the Workspace

#### Installation (Recommended)

```bash
pip install -e .
```

#### Specific Robot Hardware Support

```bash
# For SO-101 arm
pip install -e .[feetech]

# For all hardware support
pip install -e .[all]
```

## System Requirements

### Hardware
- **NVIDIA Jetson Orin AGX** 
- **CUDA 12.8** 
- **100GB+ storage space**

### Software
- **Python 3.12**
- **CUDA 12.8 Toolkit**

## Available Features

### Robot Hardware Support
- **Feetech Servos**: `pip install -e .[feetech]`
- **Dynamixel Motors**: `pip install -e .[dynamixel]`
- **Aloha Robot**: `pip install -e .[aloha]`
- **xArm Robot**: `pip install -e .[xarm]`
- **Stretch Robot**: `pip install -e .[stretch]`

### Camera Support
- **Intel RealSense**: `pip install -e .[intelrealsense]`

### Development Tools
- **Development Environment**: `pip install -e .[dev]`
- **Documentation**: `pip install -e .[docs]`
- **Testing**: `pip install -e .[test]`

## Support

For issues and questions:
- **LeRobot**: [GitHub Issues](https://github.com/huggingface/lerobot/issues)
- **Isaac-GR00T**: [GitHub Issues](https://github.com/NVlabs/Isaac-GR00T/issues)