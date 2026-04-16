cmake_minimum_required(VERSION 3.8)
project(parachute_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Declare messages, services, and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedLoop.msg"
  "msg/DetectedLoops.msg"
  "msg/LoopGroundTruth.msg"
  "msg/HookStatus.msg"
  "msg/ArmStatus.msg"
  "msg/SideArmState.msg"
  "msg/LoopState.msg"
  "msg/LoopStateArray.msg"  
  "srv/VerifyHookInsertion.srv"
  "srv/RequestNextTarget.srv"
  "srv/PlanToHook.srv"
  "srv/RotateHook.srv"
  "srv/MoveToPosition.srv"
  "srv/MoveToWorldPose.srv"
  "srv/GetCalibratedLoops.srv"
  "srv/CaptureLoops.srv"
  "action/InsertHook.action"
  "action/ExecuteTrajectory.action"
  "action/MoveToCoordinate.action"
  "action/VisualServo.action"
  "action/ReleaseHook.action"
  DEPENDENCIES geometry_msgs std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()