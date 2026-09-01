cmake_minimum_required(VERSION 3.8)
project(rover_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmIKCommand.msg"
  "msg/ArmIKEnabledReport.msg"
  "msg/CameraCommand.msg"
  "msg/CameraFrameReport.msg"
  "msg/CameraStreamReport.msg"
  "msg/CameraStreamState.msg"
  "msg/CameraStreamTarget.msg"
  "msg/CommandHeartbeat.msg"
  "msg/CommandSessionStart.msg"
  "msg/DriveCommand.msg"
  "msg/EmergencyStopCommand.msg"
  "msg/JointPositionCommand.msg"
  "msg/JointPositionReport.msg"
  "msg/JointPowerCommand.msg"
  "msg/JointPowerTarget.msg"
  "msg/MountedPeripheralReport.msg"
  "msg/OperationModeCommand.msg"
  "msg/RoverPositionReport.msg"
  "msg/SafetyState.msg"
  "msg/ServoPositionCommand.msg"
  "msg/ServoPositionReport.msg"
  "msg/StepperTurnAngleCommand.msg"
  "msg/TankDriveCommand.msg"
  "msg/WaypointNavCommand.msg"
  "msg/WheelCommand.msg"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
