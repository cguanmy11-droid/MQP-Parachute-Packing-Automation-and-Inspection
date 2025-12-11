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




The WidowX-200 uses a **right-handed coordinate system** with the base frame at the robot's base:
```
         Z (up)
         |
         |
         |_______ Y (left when facing robot)
        /
       /
      X (forward from robot base)


Top View:
     
        Y (left)
        ^
        |
        |
        |______> X (forward)
       /
      /
    base


Side View:

    Z (up)
    ^
    |     gripper
    |      /
    |     /
    |    /_____ end-effector
    |
    |________> X (forward)
   base
```