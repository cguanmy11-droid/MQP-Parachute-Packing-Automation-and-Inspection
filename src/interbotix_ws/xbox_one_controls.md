# Xbox One Controller Mapping Guide

## Completed Updates

I have completed the Xbox One controller integration based on your test results and desired control logic:

### 1. Tested Button Mapping
```
A button = 0        B button = 1        X button = 3        Y button = 4
LB = 6             RB = 7              
LT = axis 5        RT = axis 4
Left stick X = axis 0   Left stick Y = axis 1
Right stick X = axis 2   Right stick Y = axis 3
D-pad X = axis 6   D-pad Y = axis 7
```

### 2. Control Function Mapping

| Xbox One Button/Axis | Function | PS4 Equivalent |
|----------------|------|----------|
| **A Button** | Gripper Close | Square |
| **B Button** | Gripper Open | Circle (O) |
| **X Button** | Decrease Gripper Pressure | X |
| **Y Button** | Increase Gripper Pressure | Triangle |
| **LB** | 🔄 **Pose Toggle** (Home ↔ Sleep) | - |
| **RB** | End-effector -Y Direction Movement | R1 |
| **LT (Trigger)** | Waist Counter-clockwise Rotation | L2 |
| **RT (Trigger)** | Waist Clockwise Rotation | R2 |
| **START** | Robot Move to Home Pose | START/OPTIONS |
| **SELECT** | Robot Move to Sleep Pose | SELECT/SHARE |
| **Left Stick X** | End-effector Horizontal Movement | Left stick Left/Right |
| **Left Stick Y** | End-effector Vertical Movement | Left stick Up/Down |
| **Right Stick X** | End-effector Roll | Right stick Left/Right |
| **Right Stick Y** | End-effector Pitch | Right stick Up/Down |
| **L3 (Left Stick Press)** | Invert Left Stick X-axis Control | L3 |
| **R3 (Right Stick Press)** | Invert Right Stick X-axis Control | R3 |
| **D-pad Up** | Increase Control Loop Rate | D-pad Up |
| **D-pad Down** | Decrease Control Loop Rate | D-pad Down |
| **D-pad Left** | Coarse Control Mode | D-pad Left |
| **D-pad Right** | Fine Control Mode | D-pad Right |
| **Xbox Guide** | Torque Toggle (Hold 3 seconds to disable) | PS |

## Usage

### Launch Command
```bash
ros2 launch interbotix_xsarm_joy xsarm_joy.launch.py robot_model:=wx200 controller:=xboxone
```

### Key Features

1. **🔄 Smart Pose Switching**: LB button has intelligent switching logic
   - **Any non-Home pose** (including after manual movement) → Press LB → **Home pose**
   - **Home pose** → Press LB → **Sleep pose**  
   - **Sleep pose** → Press LB → **Home pose**
   - Automatically detects manual movement, ensures return to Home before switching to Sleep
   - Displays detailed log information showing switching reason and target pose
2. **Special Trigger Handling**: LT/RT triggers now correctly mapped for waist control
3. **Simplified Y-axis Control**: Only RB controls end-effector -Y direction (LB dedicated to pose switching)
4. **Safe Workspace**: Added workspace boundary checks to prevent "No valid pose" errors
5. **Complete Speed Control**: D-pad controls speed and control precision

### Notes

1. **Button Numbers**: Some button numbers are estimated (e.g., START=9, SELECT=8). If they're different in practice, please tell me the actual numbers
2. **Trigger Sensitivity**: LT/RT triggers must be pressed more than 50% to activate waist rotation
3. **Joystick Deadzone**: Joystick threshold set to 0.75, can be adjusted at launch

## Testing Recommendations

Now you can test all functions:

1. **Basic Movement**: Use left stick to control end-effector position
2. **Rotation Control**: Use right stick to control end-effector orientation  
3. **Gripper Control**: A/B to open/close gripper, X/Y to adjust pressure
4. **Waist Control**: LT/RT to control waist rotation
5. **🆕 Smart Pose Switching**: **LB button** intelligently manages pose switching
   - **Scenario 1**: Robot at any position (non-Home) → Press LB → Move to **Home pose**
   - **Scenario 2**: Robot at Home pose → Press LB → Move to **Sleep pose**
   - **Scenario 3**: Robot at Sleep pose → Press LB → Move to **Home pose**
   - **Scenario 4**: After manually moving robot → Press LB → Return to **Home pose** first
   - System displays log information showing switching reason
6. **Traditional Pose Control**: START/SELECT can still directly switch preset poses

If any buttons don't work or are mapped incorrectly, please tell me the actual button numbers and I'll adjust them!