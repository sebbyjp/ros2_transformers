from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



# def launch_setup(context, *args, **kwargs):

#     robot_state_publisher_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         # namespace=robot_name_launch_arg,
#         parameters=[{
#             'robot_description':
#                 ParameterValue(
#                     Command(FindExecutable('xacro'), ' ', LaunchConfiguration('xacro_path')
#                     , value_type=str),
#             'use_sim_time':
#                 use_sim_time_param,
#         }],
#         output={'both': 'screen'},
#     )

#     world_broadcaser_node = Node(
#         package="tf2_ros",
#         executable="static_transform_publisher",
#         arguments=['0', '0', '0', ' 0', '0', '0', 'odom', 'world'],
#         output="screen")

#     return [
#         world_broadcaser_node,
#         robot_state_publisher_node,
#     ]


def generate_launch_description():


    return LaunchDescription([
           DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=('true', 'false'),
            description=
            ('tells ROS nodes asking for time to get the Gazebo-published simulation time, '
             'published over the ROS topic /clock.')),
        DeclareLaunchArgument(  'urdf_path'),
        Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description':
                ParameterValue(value=
                    Command([FindExecutable(name='xacro'), ' ', LaunchConfiguration('urdf_path')]
                   ), value_type=str),
            'use_sim_time':
                LaunchConfiguration('use_sim_time'),
        }],
        output={'both': 'screen'},
    )]
    )
