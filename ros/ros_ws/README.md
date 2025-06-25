# GR00T ROS Integration

This directory contains ROS 2 packages for integrating NVIDIA Isaac GR00T VLA inference with ROS systems on Jetson platforms.

## Overview

The integration provides:
- **gr00t_msgs**: Custom ROS messages for robot observations and actions
- **gr00t_inference**: ROS nodes for running GR00T inference server and client

## Architecture

```
┌─────────────────┐    ROS Topics    ┌─────────────────┐    ZMQ/ROS     ┌─────────────────┐
│  Robot Sensors  │ ──────────────── │ Inference Client │ ─────────────  │ Inference Server │
│                 │                  │     Node         │                │     Node         │
│ - Cameras       │                  │                  │                │                  │
│ - Joint States  │                  │ - Observation    │                │ - GR00T Policy  │
│ - Language      │                  │   Collection     │                │ - Model Loading │
└─────────────────┘                  │ - Action Exec.   │                │ - GPU Inference │
                                     └─────────────────┘                └─────────────────┘
                                              │                                   │
                                              └─────── Action Commands ──────────┘
```

## Packages

### gr00t_msgs

Custom ROS 2 message definitions:
- `RobotObservation.msg`: Encapsulates multi-modal observations (images, joint states, language)
- `RobotAction.msg`: Contains action chunks with joint and gripper commands
- `GetAction.srv`: Service for synchronous action inference

### gr00t_inference

ROS 2 nodes:
- **Server Node** (`gr00t_inference_server`): Loads GR00T model and serves inference requests
- **Client Node** (`gr00t_inference_client`): Collects observations and executes actions

## Setup

### Prerequisites

- ROS 2 Humble
- NVIDIA Jetson with CUDA support
- Isaac GR00T repository

### Building

1. **Build the Docker image (recommended for Jetson):**
   ```bash
   docker build -f orin.Dockerfile -t gr00t-jetson .
   ```

2. **Or build natively:**
   ```bash
   cd ros_ws
   ./build.sh
   ```

### Environment Setup

Source the workspace:
```bash
source ros_ws/install/setup.bash
```

## Usage

### Quick Start

1. **Launch the inference server:**
   ```bash
   # Using launch script with your finetuned model
   ./ros_ws/launch_server.sh --model-path /path/to/your/finetuned/model
   
   # Or with ros2 launch directly
   ros2 launch gr00t_inference server.launch.py model_path:=/path/to/your/model
   ```

2. **Launch the inference client:**
   ```bash
   # Using launch script
   ./ros_ws/launch_client.sh --server-host localhost --server-port 5555
   
   # Or with ros2 launch directly  
   ros2 launch gr00t_inference client.launch.py
   ```

### Server Configuration

The server node accepts these parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_path` | `nvidia/GR00T-N1.5-3B` | Path to GR00T model or HuggingFace model ID |
| `embodiment_tag` | `gr1` | Robot embodiment tag |
| `data_config` | `fourier_gr1_arms_only` | Data configuration for modality setup |
| `denoising_steps` | `4` | Number of denoising steps for inference |
| `server_port` | `5555` | ZMQ server port |
| `device` | `cuda` | Device for inference (cuda/cpu) |

**Example with your finetuned model:**
```bash
ros2 launch gr00t_inference server.launch.py \
    model_path:=/workspace/checkpoints/my_finetuned_model \
    embodiment_tag:=my_robot \
    data_config:=my_robot_config \
    denoising_steps:=4
```

### Client Configuration

The client node accepts these parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `server_host` | `localhost` | Inference server hostname/IP |
| `server_port` | `5555` | Inference server port |
| `camera_topics` | `["/camera/image_raw"]` | List of camera topics |
| `camera_names` | `["ego_view"]` | Corresponding camera names for GR00T |
| `joint_state_topic` | `/joint_states` | Robot joint state topic |
| `language_instruction_topic` | `/language_instruction` | Language instruction topic |
| `action_topic` | `/robot_action` | Topic to publish actions |
| `action_horizon` | `8` | Number of actions to execute from each chunk |
| `control_frequency` | `50.0` | Control loop frequency (Hz) |

**Example for dual-camera setup:**
```bash
ros2 launch gr00t_inference client.launch.py \
    camera_topics:='["/camera/front/image_raw", "/camera/wrist/image_raw"]' \
    camera_names:='["front", "wrist"]' \
    joint_state_topic:=/robot/joint_states \
    action_topic:=/robot/command
```

## ROS Topic Interface

### Subscribed Topics (Client Node)

- `/camera/image_raw` (sensor_msgs/Image): Camera images
- `/joint_states` (sensor_msgs/JointState): Robot joint positions and velocities  
- `/language_instruction` (std_msgs/String): Task instructions

### Published Topics (Client Node)

- `/robot_action` (gr00t_msgs/RobotAction): Robot action commands

### Services

- `/get_action` (gr00t_msgs/GetAction): Synchronous action inference service

## Integration Examples

### With Your Robot Driver

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gr00t_msgs.msg import RobotAction
from sensor_msgs.msg import JointState
from std_msgs.msg import String

class YourRobotDriver(Node):
    def __init__(self):
        super().__init__('your_robot_driver')
        
        # Subscribe to GR00T actions
        self.action_sub = self.create_subscription(
            RobotAction, '/robot_action', self.action_callback, 10)
        
        # Publish joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Publish language instructions
        self.lang_pub = self.create_publisher(String, '/language_instruction', 10)
    
    def action_callback(self, msg):
        # Convert GR00T action to your robot commands
        if not msg.is_action_chunk:  # Single timestep action
            joint_commands = msg.joint_actions
            gripper_commands = msg.gripper_actions
            # Send to your robot...
```

### Camera Integration

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()
        
        # Publishers for multiple cameras
        self.front_pub = self.create_publisher(Image, '/camera/front/image_raw', 10)
        self.wrist_pub = self.create_publisher(Image, '/camera/wrist/image_raw', 10)
        
        # Your camera capture logic...
```

## Performance Optimization

### For Jetson Deployment

1. **Use TensorRT optimization:**
   - Build TensorRT engines using the deployment scripts
   - Set appropriate batch sizes for your use case

2. **Memory management:**
   - Monitor GPU memory usage
   - Consider reducing image resolution if needed

3. **Network optimization:**
   - Use localhost connection for server/client on same device
   - Consider shared memory for high-frequency data

## Troubleshooting

### Common Issues

1. **Server fails to load model:**
   - Check CUDA availability: `nvidia-smi`
   - Verify model path exists
   - Check GPU memory: `nvidia-smi`

2. **Client cannot connect to server:**
   - Verify server is running: `ros2 node list`
   - Check network connectivity
   - Ensure ports are not blocked

3. **Image conversion errors:**
   - Verify image topic formats match expected
   - Check cv_bridge installation
   - Ensure camera publishers are active

### Debugging

Enable debug logging:
```bash
ros2 launch gr00t_inference server.launch.py --ros-args --log-level DEBUG
```

Monitor topics:
```bash
# List active topics
ros2 topic list

# Monitor action messages
ros2 topic echo /robot_action

# Check joint states
ros2 topic echo /joint_states
```

## Performance Metrics

Expected performance on Jetson AGX Orin:
- **Inference latency**: ~100-200ms (depending on model size)
- **Control frequency**: Up to 50Hz with action chunking
- **Memory usage**: ~4-8GB GPU memory (model dependent)

## Next Steps

1. **Integrate with your robot hardware**
2. **Configure camera topics and joint mappings**
3. **Test with your finetuned model**
4. **Optimize performance for your use case**
5. **Add safety mechanisms and error handling**

## Support

For issues specific to:
- **GR00T model**: Check the main Isaac GR00T repository
- **ROS integration**: Check this ROS workspace
- **Jetson deployment**: Check NVIDIA Jetson documentation
