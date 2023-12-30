cmake_minimum_required(VERSION 3.8)
project(ros2_transformers)

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
find_package(dgl_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(dgl_ros_interfaces REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_ros REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit REQUIRED)
set(DEPENDENCIES
    rclcpp geometry_msgs rclcpp_action Eigen3 rviz_visual_tools Eigen3 tf2_ros
)

include_directories(
    ${Python_INCLUDE_DIRS}
    ${pybind11_INCLUDE_DIRS})
install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

add_executable(rt1
    src/rt1.cpp
)
ament_target_dependencies(rt1 PUBLIC
    rclcpp
    dgl_ros_interfaces
    dgl_ros
    sensor_msgs
    rviz_visual_tools
)
target_link_libraries(rt1 PUBLIC
    ${Python_LIBRARIES}
    ${pybind11_LIBRARIES})
target_include_directories(rt1 PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

add_executable(rt1_demo_scene
    src/rt1_demo_scene.cpp
)
ament_target_dependencies(rt1_demo_scene PUBLIC
    rclcpp
    moveit_visual_tools
    moveit_ros_planning_interface
    )
add_executable(rt1_demo_app
    src/rt1_demo_app.cpp
)
ament_target_dependencies(rt1_demo_app PUBLIC
    rclcpp
    tf2_eigen
    dgl_ros_interfaces
    moveit_visual_tools
    moveit_ros_planning_interface
    moveit_msgs
    moveit_task_constructor_core
    moveit_ros_planning_interface
    moveit
    moveit_ros
    )

target_include_directories(rt1_demo_scene PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
    
install(TARGETS
    rt1 rt1_demo_scene rt1_demo_app
    EXPORT export_${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})


install(DIRECTORY
launch
tasks
config
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# add_executable(rt1_demo src/rt1_demo.cpp)
# target_include_directories(rt1_demo PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(rt1_demo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# install(TARGETS rt1_demo
#   DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
