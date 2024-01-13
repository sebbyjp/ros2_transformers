# ROS2 Transformers

ROS2 package for deploying and fine-tuning multi-modal generalist agent models. This package provides inference servers as [ROS2 action servers](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html) for the most popular generalist multimodal robotics models (see [Available Models](#available-models). It depends on [robo_transformers](https://github.com/sebbyjp/robo_transformers) to access and run inference for these models.

## Table of Contents

- [ROS2 Transformers](#ros2-transformers)
  - [Table of Contents](#table-of-contents)
  - [Tested Platforms](#tested-platforms)
  - [Available Models](#available-models)
  - [Installation](#installation)
  - [Repo Structure](#repo-structure)
  - [Software Stack](#software-stack)
    - [1. Application Layer (your code)](#1-application-layer-your-code)
    - [2. ROS Agent Layer (called by your code)](#2-ros-agent-layer-called-by-your-code)
    - [2. AI Agent Layer](#2-ai-agent-layer)
    - [3. Kinematics Layer](#3-kinematics-layer)
    - [4. Controller Layer](#4-controller-layer)
  
## Tested Platforms

- :white_check_mark: Ubuntu 22.04 + ROS2 Humble

## [Available Models](#available-models)

| Model Type |  Variants | Observation Space | Action Space | Author |
| ---------- | --------- | ------- | ------- | ------- |
| [RT-1](https://robotics-transformer1.github.io/)     | rt1main, rt1multirobot, rt1simreal | text + head camera | end effector pose delta |  Google Research, 2022 |
| [RT-1-X](https://robotics-transformer-x.github.io/)  | rt1x   | text + head camera | end effector pose delta |  Google Research et al., 2023 |
| [Octo](https://github.com/octo-models/octo) | octo-base, octo-small | text + head camera + Optional[wrist camera] | end effector pose delta |  Octo Model Team et al., 2023 |

## Installation

In a term

## Repo Structure

- ``config``: Contains configuration files for the application, ros_gz bridge, and rviz gui.
- ``moveit``: Contains MoveIt configuration files that are not specific to a robot.
- ``robots``: Contains robot specific files such as urdfs, meshes, ros_controllers, and robot-specific moveit configurations.
- ``tasks``: Contains task specifications (location and properties of objects in the environment) in yaml format.
- ``sim``: Contains simulation assets for tasks and gazebo world files.

## Software Stack

### 1. Application Layer (your code)

- See `src/demo_app.cpp` and `launch/task_launch.py` for an example of how to use this package.

### 2. ROS Agent Layer (called by your code)

- Agent Inference Server (C++ or Python) (See [dgl_ros](https://github.com/sebbyjp/dgl_ros)).
- See `include/rt1.cpp` and `include/octo.cpp`

### 2. AI Agent Layer

- Tensorflow or PyTorch for inference and training (Python) (See [robo_transformers](https://github.com/sebbyjp/robo_transformers))
- ONNX or OpenVino for high performance inference and training (C++)

The following layers are usually part of the user’s robot stack but we include them to support robots out-of-the-box for users who only have actuator driver or ROS control API’s from the manufacturer (Note that ROS control makes calls to the driver APIs). These layers are only required at all because current foundational models for robotics output to action spaces like position or velocity of a robots arms and feet. Once foundational models begin outputting to input spaces for each of these layers (first joint angles then motor torques), they become redundant.

### 3. Kinematics Layer

- MoveIt Inverse Kinematics (C++)
- Open Motion Planning Library (C++)

### 4. Controller Layer

- ROS2 control (C++)
