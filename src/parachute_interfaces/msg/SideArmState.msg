# Parsed state from ESP32 side arm controller with mm coordinates
std_msgs/Header header

# Limit switch states
bool limit_horizontal      # l2 - stepper2 (horizontal belt) limit
bool limit_vertical        # l3 - stepper1 (vertical lead screw) limit
bool limit_depth           # l1 - DC motor (depth) limit

# Raw step positions
int64 steps_horizontal     # s2 - stepper2 position in steps
int64 steps_vertical       # s1 - stepper1 position in steps
int32 dc_percent           # dc - DC motor duty cycle percent

# Computed positions in mm
float64 x_mm               # Horizontal position (stepper2)
float64 y_mm               # Vertical position (stepper1)
float64 z_mm               # Depth position (DC motor, estimated)

# Homing status
bool is_homed              # True if all axes have been homed
