from os import path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    FindExecutable,
    Command,
    LaunchConfiguration,
)
from ament_index_python.packages import get_package_share_directory
import yaml


def spawn_create_item_node(
    name: str,
    urdf_file: str,
    x: float,
    y: float,
    z: float,
    xacro_args: str = '',
) -> Node:
    '''Creates a ROS node that spawns a URDF model in Gazebo.

    Args:
        name (str): Reference for gazebo model
        urdf_file (str): Absolute path to URDF file
        x (float):  
        y (float):  
        z (float):  

    Returns:
        Node: 
    '''
    return Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string',
            Command([FindExecutable(name='xacro'), ' ', urdf_file, xacro_args]),
            '-name', name, '-x',
            str(x), '-y',
            str(y), '-z',
            str(z)
        ])


def generate_launch_description() -> LaunchDescription:
    ros2_transformers_dir = get_package_share_directory('ros2_transformers')
    with open(path.join(ros2_transformers_dir, 'config/block_config.yaml')) as f:
        task_config = yaml.load(f, Loader=yaml.FullLoader)

    start_gz_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        ]),
        launch_arguments=[('gz_args', [
            LaunchConfiguration('gz_args'), ' ',
            LaunchConfiguration('world_filepath')
        ])],
    )

    x, y, z, _, _, _ = task_config['table_pose']
    spawn_table = spawn_create_item_node(
        'table',
        PathJoinSubstitution([
            FindPackageShare('ros2_transformers'),
            'urdf/objects/table.urdf.xacro ',
        ]), x, y, z)

    x, y, z, _, _, _ = task_config['block1_pose']
    spawn_block1 = spawn_create_item_node(
        'block1',
        PathJoinSubstitution([
            FindPackageShare('ros2_transformers'),
            'urdf/objects/block.urdf.xacro ',
        ]),
        x,
        y,
        z,
        xacro_args='color:=red')

    x, y, z, _, _, _ = task_config['block2_pose']
    spawn_block2 = spawn_create_item_node(
        'block2',
        PathJoinSubstitution([
            FindPackageShare('ros2_transformers'),
            'urdf/objects/block.urdf.xacro ',
        ]),
        x,
        y,
        z,
        xacro_args='color:=green')
    x, y, z, _, _, _ = task_config['block3_pose']
    spawn_block3 = spawn_create_item_node(
        'block3',
        PathJoinSubstitution([
            FindPackageShare('ros2_transformers'),
            'urdf/objects/block.urdf.xacro ',
        ]),
        x,
        y,
        z,
        xacro_args='color:=yellow')

    # Examples: https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge
    bridge = Node(      
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[
            {
                'config_file':
                    path.join(ros2_transformers_dir, 'config/ros_gz_bridge.yaml')
            },
            # {
            #     'qos_overrides': {
            #         '/rgbd_camera/points': {
            #             'publisher': {
            #                 'depth': 9,
            #                 'durability': 'transient_local',
            #                 'history': 'keep_last',
            #                 'reliability': 'reliable'
            #             },
            #             # 'subscription': {
            #             #     'depth': 10,
            #             #     'durability': 'transient_local',
            #             #     'history': 'keep_last',
            #             #     'reliability': 'reliable'
            #             # }
            #         }
            #     }
            # },
            {
                'use_sim_time': True,
                'subscription_heartbeat': 100
            }
        ],
    )
    # moveit_rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     # namespace=robot_name_launch_arg,
    #     arguments=[
    #         '-d', ' /third_party/moveit_task_constructor/demo/config/mtc.rviz',
    #         '-f', ('locobot', '_base_link')
    #     ],
    #     # remappings=remappings,
    #     output={'both': 'screen'},
    # )

    rt1_demo_node = Node(
        name="rt1_demo_node",
        package='ros2_transformers',
        executable='rt1_demo_scene',
        output='screen',
        parameters=[
            task_config,
            {
                'deep_grasps': LaunchConfiguration('deep_grasps'),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_filepath',
            default_value=PathJoinSubstitution([
                FindPackageShare('ros2_transformers'),
                'worlds/empty.sdf',
            ]),
            description='the file path to the Gazebo world file to load.',
        ),
        DeclareLaunchArgument(
            'deep_grasps',
            default_value='false',
            description='Should use deep grasp algorithm.',
        ),
        DeclareLaunchArgument(
            'gz_args',
            default_value='-r',
            description='Arguments to pass to Gazebo.',
        ),
        bridge,
        start_gz_world,
        spawn_table,
        spawn_block1,
        spawn_block2,
        spawn_block3,
        # rt1_demo_node,
        # moveit_rviz_node,
    ])
