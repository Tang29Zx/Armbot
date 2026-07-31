# Arm command (stable ROS2 interface contract, sec 3.1)
# https://.../docs/arm-control-interface.md

uint8 MODE_STOP=0
uint8 MODE_END_EFFECTOR=1
uint8 MODE_JOINT=2
uint8 MODE_GRIPPER=3
uint8 MODE_GRIPPER_STOP=4
uint8 MODE_CARTESIAN_SERVO=5
uint8 MODE_CARTESIAN_SERVO_END=6
uint8 MODE_WRIST_ROLL=7

std_msgs/Header header

uint8 mode

# MODE_END_EFFECTOR uses x/y/z in centimeters (cm); pitch follows firmware config.
float32 x
float32 y
float32 z
float32 pitch

# MODE_JOINT uses joint_position; unit rad. Disabled until servo mapping confirmed.
# MODE_WRIST_ROLL uses joint_position[4] in rad; the other entries are ignored.
float32[5] joint_position

# MODE_GRIPPER uses gripper_position; 0=open, 1=closed.
float32 gripper_position

# Expected execution time for point target modes (seconds). For
# MODE_CARTESIAN_SERVO this is the stream watchdog. Ignored by STOP modes and
# MODE_CARTESIAN_SERVO_END.
float32 duration_sec

# Caller-generated monotonically increasing id to correlate command/state/error.
uint32 sequence_id
