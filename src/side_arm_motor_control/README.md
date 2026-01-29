## Side Arm Motor Control (ESP32 + ROS 2)

This directory includes **PlatformIO/Arduino** firmware for ESP32 and a **ROS 2 Python** serial bridge. The firmware follows the verified `motor_control` project and extends it to “two steppers + one DC + three limit switches.” The ROS 2 node publishes/subscribes commands to the ESP32.

```
side_arm_motor_control/
├── firmware/         # PlatformIO project (ESP32)
└── ros2_bridge/      # ROS 2 Python package (serial bridge + topics/services)
```

### Pinout (aligned with `motor_control/include/pins.h`)

#### Limit switches (NO, INPUT_PULLUP)

| Device          | NO → ESP32 | C → GND |
| --------------- | ---------- | ------- |
| Limit Switch 1  | GPIO25     | GND     |
| Limit Switch 2  | GPIO26     | GND     |
| Limit Switch 3  | GPIO27     | GND     |

#### Steppers (two A4988 drivers)

| Device            | Pin → ESP32 | Notes                                   |
| ----------------- | ----------- | --------------------------------------- |
| Stepper1 STEP     | GPIO18      | Pulse                                    |
| Stepper1 DIR      | GPIO19      | Direction                                |
| Stepper2 STEP     | GPIO21      | Pulse                                    |
| Stepper2 DIR      | GPIO22      | Direction                                |
| Enable (shared)   | GPIO12*     | Active LOW; tie to GND if always enabled |
| VMOT              | External 12V| Common ground with ESP32                 |
| VDD               | 3.3V / 5V   | Depends on driver logic                  |

> The original `motor_control` project used STEP/DIR/EN = 18/19/12; we keep that and add a second stepper on 21/22.

#### DC motor (BTS7960)

| Device              | Pin → ESP32 | Notes                                     |
| ------------------- | ----------- | ----------------------------------------- |
| R_PWM (IN1/AIN1)    | GPIO14      | Forward PWM                               |
| L_PWM (IN2/AIN2)    | GPIO15      | Reverse PWM                               |
| R_EN / L_EN         | GPIO4       | Both EN tied together, HIGH = enabled     |
| Motor Power         | External 12V| Do not power from ESP32                   |
| Driver GND          | ESP32 GND   | Must share ground                         |

### Serial protocol (ASCII lines ending with `\n`)

| Command                              | Description                                              |
| ------------------------------------ | -------------------------------------------------------- |
| `STEPPER_MOVE,<id>,<steps>,<speed>`  | id=1/2; `steps` signed; `speed` in steps/s               |
| `STEPPER_ENABLE,<0|1>`               | 1=enable (EN LOW), 0=disable                             |
| `HOME,<id>`                          | Move negative until limit switch triggers, then zero pos |
| `DC_SPEED,<percent>`                 | `percent` in [-100,100], sign = direction                |
| `STOP_ALL`                           | Stop all motors (steppers stop + DC PWM=0)               |
| `REQUEST_STATE`                      | Send one `STATE {...}` JSON immediately                  |

State JSON fields: `l1~l3` (limit triggered=1), `s1/s2` (stepper positions in steps), `dc` (DC duty percent).

### Quick start

1) **PlatformIO / firmware**
```bash
cd side_arm_motor_control/firmware
pio run -t upload --upload-port /dev/ttyUSB0
pio device monitor -b 115200 --port /dev/ttyUSB0
```
On boot it stays idle; try `STEPPER_MOVE,1,2000,300` or `DC_SPEED,30` in the monitor.

2) **ROS 2 bridge**
```bash
cd side_arm_motor_control/ros2_bridge
# copy/link this package into your ROS 2 workspace src/ then build
colcon build --packages-select side_arm_motor_control_bridge
source install/setup.bash
ros2 launch side_arm_motor_control_bridge side_arm_serial.launch.py serial_port:=/dev/ttyUSB0
ros2 topic pub --once /side_arm/command std_msgs/msg/String "data: 'STEPPER_MOVE,2,-1000,200'"
ros2 topic echo /side_arm/state
```

```bash
# For testing here are important commands to be able to use with the ros bridge communication
# Stop all motors gracefully (STOP_NOW for immediate)
ros2 topic pub --once /side_arm/command std_msgs/msg/String "data: 'STOP_ALL'"

# Move the side arm left/right (Positive is left, negative is right)
ros2 topic pub --once /side_arm/command std_msgs/msg/String "data: 'STEPPER_MOVE,2,1000,200'"

# Move the hook up and down (positive id down, negative is up) (moves much further in fewer steps)
ros2 topic pub --once /side_arm/command std_msgs/msg/String "data: 'STEPPER_MOVE,1,100,200'"

# DC motor to move hook forward back (positive is back, negative is forward)
ros2 topic pub --once /side_arm/command std_msgs/msg/String "data: 'DC_SPEED,30'"
# You can press the limit switch to stop it (either direction) there is a slight backup once pressed

# THE HOME COMMANDS MOVE IN THE WRONG DIRECTION SO DON'T USE THOSE FOR NOW
```

### Relation to `motor_control`

- `motor_control` is a minimal, hand-coded pulse/accel demo that was tested on hardware. We keep its pinout and PWM settings, then add:
  - `AccelStepper` control for two steppers (serial/ROS friendly),
  - limit switch and homing logic,
  - serial protocol plus ROS 2 bridge.
- If you want the original demo motions, extend `firmware/src/main.cpp` with extra commands or sequences.

### Debugging tips

- If `ros2 topic pub` has no effect, ensure the serial bridge is running and the `serial_port` matches the ESP32.
- If both ids move the same stepper, wiring for STEP/DIR of the second driver is likely not on GPIO21/22.
- BTS7960 EN is held HIGH; stopping relies on PWM. For hardware e‑stop, add a switch inline with EN.
- Pin mappings live in `side_arm_motor_control/firmware/include/pin_config.h` (mirrors `motor_control/include/pins.h`).***

