cmake_minimum_required(VERSION 3.8)
project(hansel_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActiveChain.msg"
  "msg/CameraReceiveStatus.msg"
  "msg/DetachMode.msg"
  "msg/DetachRecommendation.msg"
  "msg/EmergencyStop.msg"
  "msg/HeadAngleState.msg"
  "msg/HeadServoCommand.msg"
  "msg/MotionCommand.msg"
  "msg/NetworkStatus.msg"
  "msg/SurvivorApStatus.msg"
  "msg/SurvivorCommEvent.msg"
  "msg/UnitState.msg"
  "msg/WheelState.msg"
  "srv/DetachUnit.srv"
  "srv/PrepareDetach.srv"
  "srv/SetDriveEnabled.srv"
  "srv/TriggerDetach.srv"
  DEPENDENCIES builtin_interfaces diagnostic_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

