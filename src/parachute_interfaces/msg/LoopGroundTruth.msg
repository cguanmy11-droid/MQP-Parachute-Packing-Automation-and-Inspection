# Ground truth loop positions for simulation
# These represent the actual physical locations of loops in the world frame

std_msgs/Header header
geometry_msgs/Point[] positions    # Loop centers in header.frame_id (typically 'world')
float32[] radii                    # Optional: radius of each loop (meters)
int32[] loop_ids                   # Unique identifier for each loop
int32 count                        # Total number of loops
