# Robot action data
std_msgs/Header header

# Action chunk (sequence of actions)
float64[] joint_actions
float64[] gripper_actions
float64[] additional_actions

# Action metadata
int32 action_horizon
string[] action_keys
bool is_action_chunk
