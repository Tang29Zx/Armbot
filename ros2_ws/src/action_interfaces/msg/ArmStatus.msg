# Extended arm status with force-feedback fields.
# Complements the existing ArmState.msg with richer gripper/force data.

std_msgs/Header header

# Whether the arm is ready to accept a new command.
bool arm_ready

# Current gripper position in canonical range [0.0, 1.0].
float32 gripper_position

# Whether pressure/force is detected at the gripper (grasp verification).
bool force_detected

# Raw force sensor reading (units TBD based on hardware).
float32 force_raw

# Whether servo torque is enabled.
bool torque_enabled

# Diagnostic summary string (human-readable, do not parse).
string status_message
