# Quick Start Guide: GR00T ROS Integration on Jetson

## 1. Build and Setup

### Using Docker (Recommended)
```bash
# Build the Jetson-optimized image
docker build -f orin.Dockerfile -t gr00t-jetson .

# Run the container
docker run --gpus all -it --rm \
    --network host \
    -v /dev:/dev \
    gr00t-jetson bash
```

### Native Build
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Build the workspace
cd ros_ws
./build.sh

# Source the workspace
source install/setup.bash
```

## 2. Launch the System

### Terminal 1: Start the inference server
```bash
# With your finetuned model
./launch_server.sh --model-path /path/to/your/finetuned/model

# Or with default model
./launch_server.sh
```

### Terminal 2: Start the client
```bash
# Basic setup
./launch_client.sh

# Custom camera setup
./launch_client.sh \
    --camera-topics '["/front_cam/image", "/wrist_cam/image"]' \
    --camera-names '["front", "wrist"]'
```

### Terminal 3: Test with fake data (optional)
```bash
# Publish test sensor data
python3 test_gr00t_ros.py
```

## 3. Integration with Your Robot

Replace the test data with your actual robot topics:

1. **Camera topics**: Update camera topic names in launch_client.sh
2. **Joint states**: Ensure your robot publishes to `/joint_states`
3. **Language instructions**: Publish instructions to `/language_instruction`
4. **Actions**: Subscribe to `/robot_action` to control your robot

## 4. Configuration for Your Robot

### For a dual-arm robot:
```bash
./launch_server.sh \
    --model-path /path/to/dual_arm_model \
    --embodiment-tag dual_arm \
    --data-config dual_arm_config

./launch_client.sh \
    --camera-topics '["/head_camera/image", "/left_wrist/image", "/right_wrist/image"]' \
    --camera-names '["head", "left_wrist", "right_wrist"]'
```

### For a mobile manipulator:
```bash
./launch_server.sh \
    --model-path /path/to/mobile_manipulator_model \
    --embodiment-tag mobile_manipulator

./launch_client.sh \
    --camera-topics '["/navigation/camera", "/arm/camera"]' \
    --camera-names '["nav", "manipulation"]'
```

## 5. Monitoring and Debugging

```bash
# Check active nodes
ros2 node list

# Monitor topics
ros2 topic list
ros2 topic echo /robot_action
ros2 topic hz /camera/image_raw

# Check logs
ros2 log list
```

## 6. Performance Tips

- Use `--device cuda` for GPU inference (default)
- Adjust `--control-frequency` based on your robot's capabilities
- Consider `--action-horizon` for smooth execution
- Monitor GPU memory with `nvidia-smi`

## 7. Safety

- Always test in simulation first
- Implement safety stops in your robot driver
- Monitor action commands before executing
- Set appropriate joint limits
