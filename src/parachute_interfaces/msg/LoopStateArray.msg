# Array of all detected loop states from top camera

std_msgs/Header header
LoopState[] loops
bool changed          # True if any state changed since last publish
int32 total_count     # Total number of detected loops
