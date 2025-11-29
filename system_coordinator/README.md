# Parachute Coordinator

High-level coordination and state machine for the parachute packing system.

## Nodes
- `packing_coordinator_node` - Main orchestration state machine

## States
1. Detect loops
2. Position side arm
3. Verify line to hook handoff
4. Plan main arm path
5. Execute stowing motion
6. Retract hook
7. Remove hook  
8. Error / Stop 

## Possible future added states
1. Slack generation
2. Handoff verification
3. Stow verification

Coordinates between perception, side arm, and main arm packages.