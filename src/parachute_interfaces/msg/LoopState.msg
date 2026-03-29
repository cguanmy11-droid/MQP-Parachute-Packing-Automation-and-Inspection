# Single loop with position-based ID and classification state

string loop_id        # e.g. "R1", "R2", "L1", "L2"
string state          # "fully", "partial", "not", "unknown"
float32 confidence    # classification confidence (0.0-1.0)
float32 center_x      # normalized center x in image (0.0-1.0)
float32 center_y      # normalized center y in image (0.0-1.0)
