cmake_minimum_required(VERSION 3.8)
project(brain_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BrainAction.msg"
  "msg/BrainTask.msg"
  "msg/ControlMode.msg"
  "msg/ExecutorStatus.msg"
  "msg/ObservationSummary.msg"
  "msg/TrackedObject.msg"
  "msg/TrackedObjectArray.msg"
  "srv/ClearTask.srv"
  "srv/RecordSemanticTarget.srv"
  "srv/SetEmergencyStop.srv"
  "srv/SubmitTask.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

