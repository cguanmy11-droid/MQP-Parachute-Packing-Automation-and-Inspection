# Side Arm Control

Controls the ESP32-based side arm. Stepper motors for horizontal and vertical motion along the bag, DC motor to insert/retract hook arm into loop and servo motor for hook rotation.

## Nodes
- `side_arm_interface_node` - ROS 2 ↔ ESP32 communication bridge

## Actions
- `/side_arm/insert_hook` - Insert hook through target loop

## Services
- `/side_arm/retract_hook` - Retract hook after stowing

## Topics
- Publishes: `/side_arm/status`