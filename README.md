# ROS2 Transformers
ROS2 package for deploying and fine-tuning multi-modal generalist agent models. Pre-trained RT-1 and RT-1-X models released by Google are made available as ROS inference servers. See [RoboTransformers](https://github.com/sebbyjp/robo_transformers) for a pure python implementation. RT-2 and other Vision-Language-Action (VLA) models will be coming soon.

## Code Structure
Our stack currently supports any user-provided AI agent models that run in Tensorflow or PyTorch as well as pre-trained RT-1 and RT-1-X. We will soon support Octo, and custom RT-2 and RT-2-X models with updated vision backbones.

Our core stack includes:

### 1. Application Layer (called by your code)
- Agent Inference Server (C++ or Python) (See [dgl_ros](https://github.com/sebbyjp/dgl_ros)).
- Dataset Generator Server (C++ or Python)
- Online Trainer (C++ or Python)

### 2. AI Agent Layer 
- Tensorflow or PyTorch for inference and training (Python) (See [robo_transformers](https://github.com/sebbyjp/robo_transformers))
- ONNX or OpenVino for high performance inference and training (C++)

The following layers are usually part of the user’s robot stack but we include them to support robots out-of-the-box for users who only have actuator driver or ROS control API’s from the manufacturer (Note that ROS control makes calls to the driver APIs). These layers are only required at all because current foundational models for robotics output to action spaces like position or velocity of a robots arms and feet. Once foundational models begin outputting to input spaces for each of these layers, and ultimately to API calls to the hardware drivers, they become redundant:

### 3. Kinematics Layer
- MoveIt Inverse Kinematics (C++)
- Open Motion Planning Library (C++)

### 4. Controller Layer
- ROS2 control (C++)
