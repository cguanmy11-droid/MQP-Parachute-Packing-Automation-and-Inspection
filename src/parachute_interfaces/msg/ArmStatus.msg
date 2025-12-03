# Status of main robot arm

std_msgs/Header header
uint8 state              # Current arm state
geometry_msgs/Pose current_pose
bool is_moving
bool goal_reached
string error_message

# State constants
uint8 STATE_IDLE = 0
uint8 STATE_PLANNING = 1
uint8 STATE_EXECUTING = 2
uint8 STATE_SUCCESS = 3
uint8 STATE_ERROR = 4