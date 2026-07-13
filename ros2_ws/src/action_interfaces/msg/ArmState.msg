# Arm state feedback (stable ROS2 interface contract, sec 3.3)
# https://.../docs/arm-control-interface.md

uint8 STATE_IDLE=0
uint8 STATE_MOVING=1
uint8 STATE_SUCCEEDED=2
uint8 STATE_ERROR=3
uint8 STATE_ESTOP=4

std_msgs/Header header

uint8 state

# sequence_id of the most recently accepted or completed command.
uint32 sequence_id

# Stable joint feedback (rad). Only meaningful when position_valid is true.
float32[5] joint_position

# Gripper feedback, same 0=open, 1=closed range as ArmCommand.gripper_position.
float32 gripper_position

# If false, callers must NOT treat the position arrays as real feedback.
bool position_valid

# Stable machine interface for errors.
uint16 error_code

# Human-readable diagnostic text only; never parse this.
string error_message
