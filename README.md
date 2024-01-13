# ROS2 Transformers

ROS2 package for deploying and fine-tuning multi-modal generalist agent models. This package provides inference servers as [ROS2 action servers](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html) for the most popular generalist multimodal robotics models (see the next section). It depends on [robo_transformers](https://github.com/sebbyjp/robo_transformers) to access and run inference for these models.

## Available Models

| Model Type |  Variants | Observation Space | Action Space | Author |
| ---------- | --------- | ------- | ------- | ------- |
| [RT-1](https://robotics-transformer1.github.io/)     | rt1main, rt1multirobot, rt1simreal | text + head camera | end effector pose delta |  Google Research, 2022 |
| [RT-1-X](https://robotics-transformer-x.github.io/)  | rt1x   | text + head camera | end effector pose delta |  Google Research et al., 2023 |
| [Octo](https://github.com/octo-models/octo) | octo-base, octo-small | text + head camera + Optional[wrist camera] | end effector pose delta |  Octo Model Team et al., 2023 |

## Code Structure

### 1. Application Layer (called by your code)

- Agent Inference Server (C++ or Python) (See [dgl_ros](https://github.com/sebbyjp/dgl_ros)).
- Dataset Generator Server (C++ or Python)
- Online Trainer (C++ or Python)

### 2. AI Agent Layer

- Tensorflow or PyTorch for inference and training (Python) (See [robo_transformers](https://github.com/sebbyjp/robo_transformers))
- ONNX or OpenVino for high performance inference and training (C++)

The following layers are usually part of the user’s robot stack but we include them to support robots out-of-the-box for users who only have actuator driver or ROS control API’s from the manufacturer (Note that ROS control makes calls to the driver APIs). These layers are only required at all because current foundational models for robotics output to action spaces like position or velocity of a robots arms and feet. Once foundational models begin outputting to input spaces for each of these layers (first joint angles then motor torques), they become redundant.

### 3. Kinematics Layer

- MoveIt Inverse Kinematics (C++)
- Open Motion Planning Library (C++)

### 4. Controller Layer

- ROS2 control (C++)
