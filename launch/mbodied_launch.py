from os import path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    RegisterEventHandler,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    FindExecutable,
    Command,
    LaunchConfiguration,
    EnvironmentVariable
)
from ament_index_python.packages import get_package_share_directory
import yaml
from os import path

# Launch robot description, start controllers, start moveit, start sim
def generate_launch_description():
    # Get the launch directory
    ros2_transformers_dir = get_package_share_directory('ros2_transformers')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Declare the launch arguments
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_urdf_path_argument = DeclareLaunchArgument(
        'urdf',
        default_value=path.join(ros2_transformers_dir, 'urdf', '/robots/locobot_wx250s/description/urdf/locobot.urdf.xacro'),
        description='Absolute path to robot urdf file'
    )
    
    

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output={'both','screen'},
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command([FindExecutable(name='xacro'), ' ', LaunchConfiguration('urdf')])}]
    )

    # Include launch sim
    gz_launch_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ros2_transformers_dir, '/launch/gz_launch.py'])
    )

    # Spawn controllers
    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_urdf_path_argument,
        robot_state_publisher_node,
        robot_state_publisher_node,
        gz_launch_node,
    ])