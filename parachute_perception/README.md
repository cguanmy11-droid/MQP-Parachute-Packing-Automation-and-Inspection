# Parachute Perception

Handles loop detection data from camera processing node for parachute packing automation and camera based verification for hook location and successful stows. 

## Nodes
- `target_selector_node` - Selects which loop is next target 
- [Future] `hook_verification_node` - Verifies hook insertion through loop
- `stow_verification_node` - Verifies a completed stow with line

## Topics
- Publishes: `/target_loop`
- Subscribes: `/detected_loops` (from Yolo_detect)

## Services
- `/request_next_target` - Request next loop to target
- [Future] `/verify_hook_insertion` - Verify hook went through loop
- [Future] `/verify_successful_stow` - Verify a successfully completed stow