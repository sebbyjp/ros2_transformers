# Copyright 2022 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from pathlib import Path
import xacro
from interbotix_xs_modules.xs_common import (
    get_interbotix_xslocobot_models,)
from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xslocobot_robot_description_launch_arguments,)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    ExecuteProcess,
)
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):

    set_gz_resource_path = ExecuteProcess(
        cmd=[
            FindExecutable(name='ign'), 'service', '--service',
            '/gazebo/resource_paths/add', '--reqtype',
            'ignition.msgs.StringMsg_V', '--reptype', 'ignition.msgs.Empty',
            '--timeout', '1000', '-rdata:"' + str(
                Path(
                    FindPackageShare('ros2_transformers').
                    perform(context)).parent.resolve()) + '"'
        ],
        output='screen',
    )
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        # namespace=robot_name_launch_arg,
        output='screen',
        arguments=[
            '-entity', 'robot_description', '-topic', 'robot_description', '-x',
            '0.0', '-y', '0.0', '-z', '0.0', '-Y', '0.0'
        ],
    )


    spawn_joint_state_broadcaster_node = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
        ],
        output='screen')

    spawn_arm_controller_node = Node(
        name='arm_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
        ],
        output='screen')

    spawn_gripper_controller_node = Node(
        name='gripper_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
        ],
        output='screen')

    spawn_camera_controller_node = Node(
        name='camera_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            # '-c',
            # f'{robot_name_launch_arg.perform(context)}/controller_manager',
            'camera_controller',
        ],
        output='screen')

    spawn_diffdrive_controller_node = Node(
        name='diffdrive_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            # '-c',
            # f'{robot_name_launch_arg.perform(context)}/controller_manager',
            'diffdrive_controller',
        ],
        output='screen')

    # spawn joint_state_broadcaster after robot is spawned
    load_joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[spawn_joint_state_broadcaster_node]))

    # spawn diffdrive_controller after joint_state_broadcaster is spawned
    load_diffdrive_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_diffdrive_controller_node]))

    # spawn camera_controller after joint_state_broadcaster is spawned
    load_camera_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_camera_controller_node]))

    # spawn arm_controller controller after joint_state_broadcaster is spawned
    load_arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_arm_controller_node]))

    # spawn gripper_controller controller after joint_state_broadcaster is spawned
    load_gripper_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_gripper_controller_node]))

    return [
        set_gz_resource_path,
        # gz_model_path_env_var,
        # gz_media_path_env_var,
        # gz_model_uri_env_var,
        # gazebo_launch_include,
        # controller_manager_node,
        load_diffdrive_controller_event,
        load_camera_controller_event,
        load_joint_state_broadcaster_event,
        load_arm_controller_event,
        load_gripper_controller_event,
        spawn_robot_node,
    ]


def generate_launch_description():

    return LaunchDescription([OpaqueFunction(function=launch_setup)])
