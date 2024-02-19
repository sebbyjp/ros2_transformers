import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
    Command,
    FindExecutable
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):

#     robot_model_launch_arg = LaunchConfiguration('robot_model')
#     robot_name_launch_arg = LaunchConfiguration('robot_name')
#     arm_model_launch_arg = LaunchConfiguration('arm_model')
#     external_urdf_loc_launch_arg = LaunchConfiguration('external_urdf_loc')
#     mode_configs_launch_arg = LaunchConfiguration('mode_configs')
#     use_moveit_rviz_launch_arg = LaunchConfiguration('use_moveit_rviz')
#     rviz_config_file_launch_arg = LaunchConfiguration('rviz_config_file')
#     world_filepath_launch_arg = LaunchConfiguration('world_filepath')
#     robot_description_launch_arg = LaunchConfiguration('robot_description')
#     hardware_type_launch_arg = LaunchConfiguration('hardware_type')
#     xs_driver_logging_level_launch_arg = LaunchConfiguration(
#         'xs_driver_logging_level')

#     # sets use_sim_time parameter to 'true' if using gazebo hardware
#     use_sim_time_param = determine_use_sim_time_param(
#         context=context, hardware_type_launch_arg=hardware_type_launch_arg)

    robot_description = {
        'robot_description_semantic':
         ParameterValue(value=Command([
        PathJoinSubstitution([
            FindExecutable(name='xacro')
        ]),
        ' ',
        FindPackageShare('ros2_agents'),
        '/robots/locobot_wx250s/description/urdf/locobot.urdf.xacro',
    ]), value_type=str)}

#     config_path = PathJoinSubstitution([
#         FindPackageShare('interbotix_xslocobot_moveit'),
#         'config',
#     ])

    robot_description_semantic = {
        'robot_description_semantic':
         ParameterValue(value=Command([
        PathJoinSubstitution([
            FindExecutable(name='xacro')
        ]),
        ' ',
        FindPackageShare('ros2_agents'),
        '/robots/locobot_wx250s/moveit/locobot.srdf.xacro',
    ]), value_type=str)}

    kinematics_config = PathJoinSubstitution([
        FindPackageShare('ros2_agents'),
        'moveit/kinematics.yaml',
    ])

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters':
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }

    ompl_planning_pipeline_yaml_file = load_yaml('ros2_agents',
                                                 'moveit/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(
        ompl_planning_pipeline_yaml_file)

    controllers_config = load_yaml(
        'ros2_agents',
        'robots/locobot_wx250s/moveit/ros_controller_interfaces.yaml'
    )

    config_joint_limits = load_yaml(
        'ros2_agents',
        'robots/locobot_wx250s/moveit/joint_limits.yaml'
    )

    sensor_parameters = load_yaml('ros2_agents',
                                  'moveit/sensors_3d.yaml')

    joint_limits = {
        'robot_description_planning': config_joint_limits,
    }

    moveit_controllers = {
        'moveit_simple_controller_manager':
            controllers_config,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution_parameters = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.1,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    remappings = [
        ('/get_planning_scene', 'get_planning_scene'),
        ('/arm_controller/follow_joint_trajectory',
         'arm_controller/follow_joint_trajectory'),
        ('/gripper_controller/follow_joint_trajectory',
         'gripper_controller/follow_joint_trajectory'),
    ]
#     # move_group_node = Node(
#     #     package='deep_grasp_task',
#     #     executable='cylinder_segment',
#     #     namespace="move_group",
#     #     parameters=[
#     #         {
#     #             'planning_scene_monitor_options': {
#     #                 'robot_description': 'robot_description',
#     #                 'joint_state_topic': '/joint_states',
#     #             },
#     #             'use_sim_time': use_sim_time_param,
#     #         },
#     #         robot_description,
#     #         robot_description_semantic,
#     #         kinematics_config,
#     #         ompl_planning_pipeline_config,
#     #         trajectory_execution_parameters,
#     #         moveit_controllers,
#     #         planning_scene_monitor_parameters,
#     #         joint_limits,
#     #         sensor_parameters,
#     #     ],
#     #     # remappings=remappings,
#     #     output={'both': 'screen'},
#     # )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        # namespace=robot_name_launch_arg,
        parameters=[
            {
                'planning_scene_monitor_options': {
                    'robot_description': 'robot_description',
                    'joint_state_topic': '/joint_states',
                },
                'use_sim_time': True,
                'capabilities': 'move_group/ExecuteTaskSolutionCapability',
                
            },
            robot_description,
            robot_description_semantic,
            kinematics_config,
            ompl_planning_pipeline_config,
        #     trajectory_execution_parameters,
            moveit_controllers,
        #     planning_scene_monitor_parameters,
        #     joint_limits,
        #     sensor_parameters,
        ],
        remappings=remappings,
        output={'both': 'screen'},
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        # namespace=robot_name_launch_arg,
        parameters=[
            robot_description,
            '',
        ],
        output={'both': 'screen'},
    )

    spawn_arm_controller_node = Node(
        name='arm_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            # '-c',
            # f'{robot_name_launch_arg.perform(context)}/controller_manager',
            'arm_controller',
        ],
        output={'both': 'screen'},
    )

    spawn_gripper_controller_node = Node(
        name='gripper_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            # '-c',
            # f'{robot_name_launch_arg.perform(context)}/controller_manager',
            'gripper_controller',
        ],
        output={'both': 'screen'},
    )

    spawn_joint_state_broadcaster_node = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            # '-c',
            # f'{robot_name_launch_arg.perform(context)}/controller_manager',
            'joint_state_broadcaster',
        ],
        condition=LaunchConfigurationEquals(
            launch_configuration_name='hardware_type',
            expected_value='fake'
        ),
        output={'both': 'screen'},
    )

    robot_description_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros2_agents'), 'launch',
                'robot_description_launch.py'
            ])
        ]),
        launch_arguments={
            'urdf_path':  PathJoinSubstitution([
                FindPackageShare('ros2_agents'), 'robots/locobot_wx250s/description/urdf/locobot.urdf.xacro'
            ])
        }.items()
    )

      # spawn camera_controller after joint_state_broadcaster is spawned
    launch_moveit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_description_node,
            on_exit=[move_group_node]))


    return [robot_description_node,
            launch_moveit,
            controller_manager_node,
            spawn_arm_controller_node,
            spawn_gripper_controller_node,
            spawn_joint_state_broadcaster_node,
            ]



def generate_launch_description():
    rviz_config_file_launch_arg = LaunchConfiguration('rviz_config_file')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    moveit_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        # namespace=robot_name_launch_arg,
        arguments=[
            '-d', rviz_config_file_launch_arg, '-f',
            (robot_name_launch_arg, '_base_link')
        ],
        parameters=[
            {'robot_description_semantic': ParameterValue(value=Command([
        PathJoinSubstitution([
            FindExecutable(name='xacro')
        ]),
        ' ',
        FindPackageShare('ros2_agents'),
        '/robots/locobot_wx250s/moveit/locobot.srdf.xacro',
    ]), value_type=str)},]
    )

    declared_arguments = [
              DeclareLaunchArgument(
            'robot_name',
            default_value='locobot',
            description=
            'name of the robot (could be anything but defaults to `locobot`).',
        ),


        DeclareLaunchArgument(
            'use_moveit_rviz',
            default_value='true',
            choices=('true', 'false'),
            description="launches RViz with MoveIt's RViz configuration.",
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('ros2_agents'), 'config',
                'rviz_gui.rviz'
            ]),
            description='file path to the config file RViz should load.',
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=('true', 'false'),
            description=
            ('tells ROS nodes asking for time to get the Gazebo-published simulation time, '
             'published over the ROS topic /clock; this value is automatically set to `true` if'
             ' using Gazebo hardware.')),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup), moveit_rviz_node])
