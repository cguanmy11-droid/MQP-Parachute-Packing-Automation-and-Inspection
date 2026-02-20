# State Machine Architecture & Implementation Plan

## Overview

This document outlines the plan for implementing a proper ROS 2 state machine coordinator for the automated parachute line stowing system. It serves as a reference for development priorities, architectural decisions, and the target interface contracts between packages.

The state machine replaces the linear step-sequence approach in `full_stow_demo_node.py` (archived as a working reference) with an event-driven coordinator that matches the system design described in the MQP paper.

---

## Current State (What Works)

The `full_stow_demo_node.py` (renamed to `stow_demo_legacy_node.py`) demonstrates:

- Loading and applying motion patterns from JSON files
- Sending trajectory goals to the main arm via `ExecuteTrajectory` action
- Moving the side arm via `MoveToCoordinate` action and `MoveToPosition` service
- Rotating the hook via `RotateHook` service
- Sequential step execution with timer-based delays

**Known limitations of the legacy node:**

- Linear step list, not a state machine — no branching, no recovery
- Timer-driven sequencing instead of event-driven transitions
- Coordinator builds `Pose` messages with hardcoded orientation (`w=1.0`), causing IK failures
- Motion pattern waypoints and trajectory details managed by the coordinator instead of the arm node
- Side arm position parameters are disconnected from trajectory generation
- No error handling, retry logic, or operator intervention
- No perception integration — uses hardcoded target positions
- No verification at any stage

---

## Target Architecture

### State Machine States

These match Figure 6 from the paper:

| State | Description | Entry Condition |
|-------|-------------|-----------------|
| **IDLE** | System ready, arms homed | Startup / reset |
| **AT_LOOP** | Vision-guided positioning at next loop | Target loop selected |
| **INSERT** | Hook insertion through loop with collision detection | Both arms positioned |
| **HANDOFF** | Synchronized dual-arm line transfer | Hook inserted and verified |
| **RETRACT** | Hook withdrawal with line seating | Handoff complete |
| **RELEASE** | Cycle completion and stow verification | Retraction complete |
| **COMPLETE** | All loops stowed | No remaining loops |
| **ERROR** | Fault handling with operator recovery options | Any failure condition |

### Transition Table

```
IDLE
  → AT_LOOP          (operator starts / loops remaining)

AT_LOOP
  → INSERT           (both arms report positioned successfully)
  → ERROR            (vision failure, IK failure, timeout)

INSERT
  → HANDOFF          (hook depth verified)
  → INSERT           (collision detected, retries remaining — apply offset)
  → ERROR            (max retries exceeded, depth verification failed)

HANDOFF
  → RETRACT          (trajectory complete, alignment verified)
  → AT_LOOP          (alignment lost — recoverable, retry from positioning)
  → ERROR            (unrecoverable failure)

RETRACT
  → RELEASE          (hook fully retracted, line position verified)
  → ERROR            (excessive force / current spike, position verification failed)

RELEASE
  → AT_LOOP          (more loops remain)
  → COMPLETE         (all loops stowed)
  → ERROR            (quality verification failed)

ERROR
  → (origin state)   (operator selects retry)
  → AT_LOOP          (operator selects skip loop)
  → IDLE             (operator selects abort)
```

---

## Package Responsibilities

### parachute_coordinator (this is the state machine)

**Owns:** State transitions, sequencing, operator interface, progress tracking.

**Does NOT own:** IK solving, orientation selection, trajectory feasibility, motor commands, perception processing.

The coordinator sends high-level goals and reacts to results. It should read like the paper's state diagram.

**Key node:** `stow_coordinator_node.py`

**Publishes:**
- `/stow/status` — current state, progress, loop count
- `/stow/error` — error details for operator interface

**Subscribes:**
- `/main_arm/status` — arm state enum + pose
- `/side_arm/status` — hook state enum + position (already exists as `HookStatus`)
- `/target_loop` — next loop to stow (from perception)
- `/detected_loops` — all visible loops (for verification)

**Action clients:**
- `/main_arm/execute_stow` — send target point + pattern, arm handles the rest
- `/side_arm/insert_hook` — already exists
- `/side_arm/move_to_coordinate` — already exists

**Service clients:**
- `/side_arm/rotate_hook` — already exists
- `/perception/request_next_target` — get next loop to stow

**Subscribes for commands:**
- `/stow/command` — operator input (start, stop, retry, skip, abort)

### main_arm_control

**Owns:** IK solving, end-effector orientation, trajectory feasibility, motion execution, workspace limits.

**Changes needed:**

1. **New action: `ExecuteStowTrajectory`** — accepts a target `Point` (not `Pose`) and a pattern name. The arm node internally resolves orientation per waypoint, checks IK feasibility, and executes. Returns success/failure with diagnostics.

2. **New service: `CheckTrajectoryFeasibility`** — accepts waypoints as `Point[]`, returns which (if any) are unreachable. Allows the coordinator to fail fast before committing to execution.

3. **Status topic: `/main_arm/status`** — publish an enum state (IDLE, MOVING, HOMING, ERROR) plus current end-effector pose. The coordinator subscribes to this for transition decisions.

4. **Orientation handling** — the `_make_pose` helper (or equivalent) should select appropriate orientation based on position and task phase. For low-Z stowing work, the gripper likely needs to point downward. This logic lives here, not in the coordinator.

### side_arm_control

**Mostly correct already.** The `InsertHook` action and `MoveToPosition`/`MoveToWorldPose` services are the right abstraction level.

**Changes needed:**

1. **Motor current feedback** — if the ESP32 reports current draw, include it in `InsertHook` action feedback so the coordinator can react to collisions.

2. **Homing robustness** — firmware update (done) adds proper back-off-and-approach homing. The coordinate node should expose a `Home` service that the coordinator calls and waits for confirmation.

3. **Status topic enrichment** — `HookStatus` already publishes state. Consider adding a `homing_complete` field or ensuring the state enum covers HOMING.

### side_arm_motor_control_bridge (firmware)

**Changes needed:**

1. **Homing state machine** — done (back off if on limit, slow approach, zero on trigger).

2. **Current sensing** — if ADC is available on the motor driver, add current reading to the `STATE` JSON payload. This enables collision detection without additional hardware.

3. **Homing confirmation event** — publish `EVENT HOME_COMPLETE` (or per-axis) so the ROS side knows when homing finishes rather than guessing.

### parachute_perception

**Changes needed:**

1. **`/target_loop` publisher** — a target selector node that picks the next loop to stow based on ordering logic (left-to-right, closest, etc.) and publishes it.

2. **`RequestNextTarget` service** — coordinator calls this to trigger target selection, then subscribes to `/target_loop` for the result.

3. **Camera frame correction** — fix the URDF camera orientation (currently 180° pitch causing inverted view). The detection simulator's FOV and projection math need to match the corrected frame.

---

## Coordinator Node Structure

The coordinator should be structured as a single node with clean state management:

```
stow_coordinator_node.py
├── State enum (StowState)
├── Transition table (dict of state → {event: next_state})
├── State handlers (enter/execute/exit per state)
├── Action/service clients (to arm nodes and perception)
├── Subscriber callbacks (status, detection, operator commands)
└── Transition method (event-driven, logs all transitions)
```

Each state handler:

- **AT_LOOP:** Calls `RequestNextTarget`, waits for `/target_loop`, commands both arms to position, transitions on both-ready or timeout.
- **INSERT:** Sends `InsertHook` goal, monitors feedback for collision/current, handles retry with offset, transitions on depth verification.
- **HANDOFF:** Calls `RotateHook(90)`, sends main arm stow trajectory goal, monitors vision alignment via feedback, transitions on completion.
- **RETRACT:** Sends retraction goal (reversed insertion path), monitors current, calls `RotateHook` oscillation, transitions on full retraction + vision verification.
- **RELEASE:** Calls `RotateHook(0)`, verifies stow quality via vision, increments counter, transitions to AT_LOOP or COMPLETE.
- **ERROR:** Halts all motion, logs diagnostics, waits for operator command on `/stow/command`.

---

## Interface Contracts

### Main Arm Status (`/main_arm/status`)

```
uint8 state          # IDLE=0, MOVING=1, HOMING=2, ERROR=3
geometry_msgs/Pose current_pose
bool is_homed
string error_message  # empty if no error
```

### Stow Status (`/stow/status`)

```
uint8 state           # matches StowState enum
uint32 current_loop   # which loop we're on
uint32 total_loops    # total loops to stow
string state_name     # human-readable state name
string details        # current activity description
```

### Operator Command (`/stow/command`)

```
string command        # start, stop, retry, skip, abort, home
```

---

## Implementation Order

### Phase 1: Foundation (do first)

- [ ] Archive `full_stow_demo_node.py` → `stow_demo_legacy_node.py`
- [ ] Fix main arm orientation handling — move `_make_pose` orientation logic into `main_arm_control`
- [ ] Create new `ExecuteStowTrajectory` action that accepts `Point` + pattern name
- [ ] Fix side arm URDF offsets to match physical dimensions
- [ ] Fix camera frame orientation in URDF
- [ ] Verify `world → side_arm_origin` transform accuracy with physical measurements

### Phase 2: Coordinator Skeleton

- [ ] Create `stow_coordinator_node.py` with state enum, transition table, and logging
- [ ] Implement IDLE → AT_LOOP → INSERT flow with hardcoded positions (no perception yet)
- [ ] Wire up action clients for main arm and side arm
- [ ] Add `/stow/status` publisher
- [ ] Add `/stow/command` subscriber for operator control (start/stop/abort)

### Phase 3: Full State Machine

- [ ] Implement HANDOFF state with dual-arm coordination
- [ ] Implement RETRACT state with reversed trajectory
- [ ] Implement RELEASE state with hook neutral and cycle counting
- [ ] Implement ERROR state with halt-all and operator recovery options
- [ ] Add retry logic in INSERT with position offset

### Phase 4: Perception Integration

- [ ] Wire `/target_loop` into AT_LOOP state
- [ ] Add vision verification callbacks in HANDOFF and RETRACT
- [ ] Implement stow quality check in RELEASE
- [ ] Fix detection simulator projection to match corrected camera frame

### Phase 5: Safety & Polish

- [ ] Add motor current monitoring for collision detection (firmware + ROS)
- [ ] Add `CheckTrajectoryFeasibility` service to main arm
- [ ] Add timeout handling for every action call
- [ ] Test full multi-loop stowing sequence
- [ ] Tune parameters (thresholds, speeds, offsets) on hardware

---

## File Locations

```
src/parachute_coordinator/
├── parachute_coordinator/
│   ├── stow_coordinator_node.py      # NEW — the state machine
│   ├── stow_demo_legacy_node.py      # ARCHIVED — old step-based demo
│   ├── motion_pattern_manager.py     # Keep — pattern loading (used by main_arm later)
│   └── ...
├── config/
│   └── motion_patterns/              # Keep — JSON pattern files
├── launch/
│   ├── stow.launch.py                # NEW — launches coordinator + arms
│   ├── full_stow_demo.launch.py      # Keep for legacy testing
│   └── dual_arm_test.launch.py       # Keep for hardware testing
└── ...

src/main_arm_control/
├── main_arm_control/
│   ├── main_arm_interface_node.py    # UPDATE — add orientation handling, new action
│   └── ...
└── ...

src/side_arm_control/
├── urdf/
│   └── side_arm.urdf                 # UPDATE — fix joint offsets, camera frame
├── side_arm_control/
│   ├── side_arm_interface_node.py    # Minor updates — homing service, current feedback
│   └── ...
└── ...

src/side_arm_motor_control_bridge/
├── firmware/
│   └── src/
│       └── main.cpp                  # UPDATED — homing state machine (done)
└── ...
```

---

## Design Principles

1. **The coordinator reads like the state diagram.** Anyone looking at the transition table should see Figure 6 from the paper.

2. **Push implementation details down.** The coordinator says "stow at this point." The arm node figures out how.

3. **Event-driven, not timer-driven.** State transitions happen because an action completed or a sensor fired, not because a timer elapsed.

4. **Single source of truth for loop position.** One detected (or hardcoded) loop position drives both arms. No independent position parameters.

5. **Fail safely and loudly.** Every action call has a timeout. Every failure transitions to ERROR with diagnostics. No silent failures.

6. **Keep the legacy demo runnable.** The archived node stays functional for quick hardware tests while the state machine is under development.
