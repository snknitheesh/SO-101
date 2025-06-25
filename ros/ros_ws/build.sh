#!/bin/bash

# Build script for GR00T ROS workspace

set -e

echo "Building GR00T ROS workspace..."

# Check if we're in a ROS environment
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS environment..."
    source /opt/ros/humble/setup.bash
fi

# Navigate to workspace
cd "$(dirname "$0")"

# Build the workspace
echo "Building packages..."
colcon build --packages-select gr00t_msgs gr00t_inference

# Source the built workspace
echo "Sourcing workspace..."
source install/setup.bash

echo "Build complete!"
echo ""
echo "To use the workspace, run:"
echo "source install/setup.bash"
echo ""
echo "Then you can launch:"
echo "# Server:"
echo "ros2 launch gr00t_inference server.launch.py"
echo ""
echo "# Client:"
echo "ros2 launch gr00t_inference client.launch.py"
