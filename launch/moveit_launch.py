import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
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
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


# def launch_setup(context, *args, **kwargs):

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

#     robot_description = {
#         'robot_description':
#             ParameterValue(robot_description_launch_arg, value_type=str)
#     }

#     config_path = PathJoinSubstitution([
#         FindPackageShare('interbotix_xslocobot_moveit'),
#         'config',
#     ])

#     robot_description_semantic = {
#         'robot_description_semantic':
#             construct_interbotix_xslocobot_semantic_robot_description_command(
#                 robot_model=robot_model_launch_arg.perform(context),
#                 config_path=config_path),
#         'publish_robot_description_semantic':
#             True,
#     }

#     kinematics_config = PathJoinSubstitution([
#         FindPackageShare('interbotix_xslocobot_moveit'),
#         'config',
#         'kinematics.yaml',
#     ])

#     ompl_planning_pipeline_config = {
#         'move_group': {
#             'planning_plugin': 'ompl_interface/OMPLPlanner',
#             'request_adapters':
#                 'default_planner_request_adapters/AddTimeOptimalParameterization '
#                 'default_planner_request_adapters/FixWorkspaceBounds '
#                 'default_planner_request_adapters/FixStartStateBounds '
#                 'default_planner_request_adapters/FixStartStateCollision '
#                 'default_planner_request_adapters/FixStartStatePathConstraints',
#             'start_state_max_bounds_error': 0.1,
#         }
#     }

#     ompl_planning_pipeline_yaml_file = load_yaml('interbotix_xslocobot_moveit',
#                                                  'config/ompl_planning.yaml')
#     ompl_planning_pipeline_config['move_group'].update(
#         ompl_planning_pipeline_yaml_file)

#     controllers_config = load_yaml(
#         'interbotix_xslocobot_moveit',
#         f'config/controllers/{arm_model_launch_arg.perform(context)}_controllers.yaml'
#     )

#     config_joint_limits = load_yaml(
#         'interbotix_xslocobot_moveit',
#         f'config/joint_limits/{arm_model_launch_arg.perform(context)}_joint_limits.yaml'
#     )

#     sensor_parameters = load_yaml('interbotix_xslocobot_moveit',
#                                   'config/sensors_3d.yaml')

#     joint_limits = {
#         'robot_description_planning': config_joint_limits,
#     }

#     moveit_controllers = {
#         'moveit_simple_controller_manager':
#             controllers_config,
#         'moveit_controller_manager':
#             'moveit_simple_controller_manager/MoveItSimpleControllerManager',
#     }

#     trajectory_execution_parameters = {
#         'moveit_manage_controllers': True,
#         'trajectory_execution.allowed_execution_duration_scaling': 1.2,
#         'trajectory_execution.allowed_goal_duration_margin': 0.5,
#         'trajectory_execution.allowed_start_tolerance': 0.1,
#     }

#     planning_scene_monitor_parameters = {
#         'publish_planning_scene': True,
#         'publish_geometry_updates': True,
#         'publish_state_updates': True,
#         'publish_transforms_updates': True,
#     }

#     remappings = [
#         ('/get_planning_scene', 'get_planning_scene'),
#         ('/arm_controller/follow_joint_trajectory',
#          'arm_controller/follow_joint_trajectory'),
#         ('/gripper_controller/follow_joint_trajectory',
#          'gripper_controller/follow_joint_trajectory'),
#     ]
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

#     move_group_node = Node(
#         package='moveit_ros_move_group',
#         executable='move_group',
#         # namespace=robot_name_launch_arg,
#         parameters=[
#             {
#                 'planning_scene_monitor_options': {
#                     'robot_description': 'robot_description',
#                     'joint_state_topic': '/joint_states',
#                 },
#                 'use_sim_time': use_sim_time_param,
#                 'capabilities': 'move_group/ExecuteTaskSolutionCapability',
#             },
#             robot_description,
#             robot_description_semantic,
#             kinematics_config,
#             ompl_planning_pipeline_config,
#             trajectory_execution_parameters,
#             moveit_controllers,
#             planning_scene_monitor_parameters,
#             joint_limits,
#             sensor_parameters,
#         ],
#         remappings=remappings,
#         output={'both': 'screen'},
#     )

   
    #         ompl_planning_pipeline_config,
    #         kinematics_config,
    #         {
    #             'use_sim_time': use_sim_time_param
    #         },
    #     ],
    #     # remappings=remappings,
    #     output={'both': 'screen'},
    # )



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
        FindPackageShare('ros2_transformers'),
        f'/robots/locobot_wx250s/moveit/locobot.srdf.xacro', ' ',
        'robot_name:=', LaunchConfiguration('robot_name'), ' ',
    ]), value_type=str)},])

    return LaunchDescription([
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
                FindPackageShare('ros2_transformers'), 'config',
                'rviz_gui.rviz'
            ]),
            description='file path to the config file RViz should load.',
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=('true', 'false'),
            description=
            ('tells ROS nodes asking for time to get the Gazebo-published simulation time, '
             'published over the ROS topic /clock; this value is automatically set to `true` if'
             ' using Gazebo hardware.')),
                moveit_rviz_node])
