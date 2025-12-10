# Parachute Coordinator

High-level coordination and state machine for the parachute packing system.

## Nodes
- `packing_coordinator_node` - Main orchestration state machine

## States:
1. IDLE
2. DETECT_LOOPS (request perception)
3. WAIT_FOR_TARGET (wait for target selection)
4. POSITION_SIDE_ARM (call side arm action)
5. VERIFY_HOOK_INSERTION (check hook went through loop)
6. PLAN_MAIN_ARM_PATH (based on verified hook position)
7. EXECUTE_MAIN_ARM_MOTION (stowing sequence)
8. VERIFY_STOW_COMPLETE (check if line is secured)
9. RETRACT_HOOK
10. VERIFY_HOOK_CLEAR
11. REPEAT (or COMPLETE if all loops done)

## Possible future added states
1. Slack generation
2. Handoff verification
3. Stow verification

Coordinates between perception, side arm, and main arm packages.