# Robot observation data
std_msgs/Header header

# Camera images (multiple cameras supported)
sensor_msgs/Image[] images
string[] camera_names

# Robot state (joint positions, gripper state, etc.)
float64[] joint_positions
float64[] joint_velocities
float64[] gripper_positions

# Language instruction
string language_instruction

# Additional state information
float64[] additional_state
string[] state_keys
