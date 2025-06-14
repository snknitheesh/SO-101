# SO 101 Arm - Quick Start Guide

This repository contains the setup for working with the SO 101 robotic arm in Isaac Sim and RViz using MoveIt.

## Prerequisites

- ROS2 Jazzy
- Isaac Sim


## Running the SO 101 Arm

### In Isaac Sim

1. Launch Isaac Sim
2. Load the SO 101 arm simulation:
```bash
ros2 launch so101_bringup isaac_sim.launch.py
```

### In RViz with MoveIt

1. Launch the MoveIt configuration:
```bash
ros2 launch so101_moveit_config moveit.launch.py
```

2. For motion planning and control:
```bash
ros2 launch so101_moveit_config move_group.launch.py
```

## Testing Motion Planning

1. Use the MoveIt Rviz plugin to:
   - Set target poses
   - Plan trajectories
   - Execute movements

2. Monitor joint states:
```bash
ros2 topic echo /joint_states
```


