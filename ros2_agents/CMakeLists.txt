cmake_minimum_required(VERSION 3.8)
project(ros2_agents)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

cmake_policy(SET CMP0074 NEW)
# Use minimum-size optimization for pybind11 bindings
add_compile_options("-Os")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(Python REQUIRED COMPONENTS Interpreter Development)

find_package(pybind11 CONFIG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros2_agents_interfaces REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit REQUIRED)

set(DEPENDENCIES
    rclcpp
    geometry_msgs
    rclcpp_action
    Eigen3
    rviz_visual_tools
    Eigen3
    tf2_ros
    tf2_eigen
    moveit_visual_tools
    moveit
    moveit_ros_planning_interface
    # moveit_msgs
    # moveit_task_constructor_core
    # moveit_ros_planning_interface
    # moveit
    moveit_ros)

include_directories(${Python_INCLUDE_DIRS} ${pybind11_INCLUDE_DIRS})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})


add_library(
  ${PROJECT_NAME}
  INTERFACE
  include/${PROJECT_NAME}/agent.hpp
  include/${PROJECT_NAME}/actor.hpp
  include/${PROJECT_NAME}/observer.hpp
  include/${PROJECT_NAME}/client.hpp
  )
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

ament_target_dependencies(${PROJECT_NAME} INTERFACE ${DEPENDENCIES})
target_include_directories(
  ${PROJECT_NAME}
  INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
            "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

add_executable(rt1 src/rt1.cpp)
ament_target_dependencies(rt1 PUBLIC rclcpp sensor_msgs ros2_agents_interfaces
                          rviz_visual_tools)
target_link_libraries(rt1 PUBLIC ${Python_LIBRARIES} ${pybind11_LIBRARIES})
target_include_directories(
  rt1 PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
             "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

add_executable(octo src/octo.cpp)
ament_target_dependencies(octo PUBLIC rclcpp ros2_agents_interfaces sensor_msgs
                          rviz_visual_tools)
target_link_libraries(octo PUBLIC ${Python_LIBRARIES} ${pybind11_LIBRARIES})
target_include_directories(
  octo PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
              "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

add_executable(demo_app src/demo_app.cpp)
ament_target_dependencies(demo_app PUBLIC rclcpp rclcpp_action
 ros2_agents_interfaces moveit_ros moveit moveit_visual_tools moveit_ros_planning_interface)
target_include_directories(
demo_app PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
            "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(
  TARGETS ${PROJECT_NAME} rt1 octo demo_app
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})

install(DIRECTORY launch tasks config sim robots moveit
        DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
