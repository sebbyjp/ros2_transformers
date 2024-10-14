# interbotix_xslocobot_descriptions

[![View Documentation](https://docs.trossenrobotics.com/docs_button.svg)](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/ros2_packages/locobot_descriptions.html)

## Overview

This package contains the URDFs and meshes for the robots in the Interbotix X-Series LoCoBot Family. The STL mesh files for each robot are located in a unique folder inside the [meshes](meshes/) directory. Also in the 'meshes' directory is the [interbotix_black.png](meshes/interbotix_black.png) picture. The appearance and texture of the robots come from this picture. Next, the URDFs for the robot are located in the [urdf](urdf/) directory. They are written in 'xacro' format so that users have the ability to customize what parts of the URDF get loaded to the parameter server. Note that all the other ROS packages in the sub-repo reference this package to launch the robot description.
