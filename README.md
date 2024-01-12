# ROS 2 Transformers

## Application Design

You implement the ROS2 control API and provide the following config files for each layer.

### Layers

#### 1. Application

- your application code config files
- task config
- ai agent config

#### 2. MoveIt MoveGroup

- kinematics_config
- ompl_planning_pipeline_config
- trajectory_execution_parameters
- moveit_controllers
- planning_scene_monitor_parameters
- joint_limits
- sensor_parameters


#### 3. ROS

- ROS controllers
  

#### 4. Control

- sim controller interface
- hardware controller interface


#### 5. Hardware

- URDF + SRDF
- mujoco description