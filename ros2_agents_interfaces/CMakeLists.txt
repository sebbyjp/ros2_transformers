cmake_minimum_required(VERSION 3.5)
project(ros2_agents_interfaces)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

set(message_files "msg/BaseControl.msg" "msg/EEFControl.msg"
                  "msg/MbodiedBasicControl.msg")

set(action_files "action/SampleGraspPoses.action" "action/Mbodied.action"
                 DEPENDENCIES)

rosidl_generate_interfaces(${PROJECT_NAME} ${message_files} ${action_files}
                           DEPENDENCIES geometry_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
