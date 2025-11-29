# Main Arm Control

Controls the WidowX-200 robot arm for parachute line stowing.

## Nodes
- `main_arm_interface_node` - Low-level arm control interface
- `main_arm_planner_node` - Motion planning for stowing sequence

## Actions
- `/main_arm/execute_trajectory` - Execute planned stowing motion

## Services
- `/main_arm/plan_to_hook` - Plan trajectory relative to hook position

## Topics
- Publishes: `/main_arm/status`