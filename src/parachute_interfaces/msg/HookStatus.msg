# Status of the side arm hook

std_msgs/Header header
uint8 state              # Current state of hook
geometry_msgs/Point position  # Current hook position
bool is_inserted         # Whether hook is through a loop
bool is_retracted        # Whether hook is fully retracted

# State constants
uint8 STATE_IDLE = 0
uint8 STATE_MOVING = 1
uint8 STATE_INSERTED = 2
uint8 STATE_HANDOFF_READY = 3
uint8 STATE_STOW_READY = 4
uint8 STATE_RETRACTING = 5
uint8 STATE_ERROR = 6