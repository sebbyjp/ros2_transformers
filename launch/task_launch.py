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


def spawn_create_item_node(
    name: str,
    model_description: str,
    x: float,
    y: float,
    z: float,
) -> Node:
    """Creates a ROS node that spawns a URDF model in Gazebo.

    Args:
        name (str): Name
        model_description (str): Either URDF or SDF string
        x (float): X position
        y (float): Y position
        z (float): Z position


    Returns:
        Node:
    """
    return Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            model_description,
            "-name",
            name,
            "-x",
            str(x),
            "-y",
            str(y),
            "-z",
            str(z),
        ],
    )


def generate_ros_param_args(app_config: dict, exclude: list[str] = ["task_name"]):
    launch_args = []
    for k, v in app_config.items():
        if k in exclude:
            continue
        launch_args.append(DeclareLaunchArgument(
            k,
            default_value=str(v),
        ),)
    return launch_args


def get_launch_arg_overrides(config: dict):
    return {k: LaunchConfiguration(k) for k, _ in config.items()}

def resolve_model_description(model_path,pkg_dir, name,color,lwh):
    if model_path.endswith(".urdf.xacro"):
            return Command([
                FindExecutable(name="xacro"), " ",
                path.join(pkg_dir, model_path),
                f" color:={color} name:={name}  l:={lwh[0]} w:={lwh[1]} h:={lwh[2]}"
            ])
    elif model_path.endswith(".sdf"):
       return Command([
            "cat",
            " ",
            path.join(pkg_dir, model_path),
        ])
    else:
        raise ValueError(
            "Model path must end with .urdf.xacro or .sdf")


def generate_launch_description() -> LaunchDescription:
    ros2_transformers_dir = get_package_share_directory("ros2_transformers")
    with open(path.join(ros2_transformers_dir, "config/app_config.yaml")) as f:
        app_config = yaml.load(f, Loader=yaml.FullLoader)
    with open(path.join(ros2_transformers_dir, "tasks/" + app_config['task_name'] + ".yaml")) as f:
        task_config = yaml.load(f, Loader=yaml.FullLoader)

    kinematics_config = PathJoinSubstitution([
        FindPackageShare("ros2_transformers"),
        "config",
        "kinematics.yaml",
    ])

    set_gz_resource_path = SetEnvironmentVariable(
        name='IGN_FILE_PATH',
        value=[
            EnvironmentVariable('IGN_FILE_PATH', default_value=''),
            ':', path.join(ros2_transformers_dir, 'sim/third_party')])
    set_gz_sdf_path = SetEnvironmentVariable(
        name='SDF_PATH',
        value=[
            EnvironmentVariable('SDF_PATH', default_value=''),
            ':', path.join(ros2_transformers_dir, 'sim/third_party')])

    start_gz_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]),
        ]),
        launch_arguments=[(
            "gz_args",
            [
                LaunchConfiguration("gz_args"),
                " ",
                LaunchConfiguration("world_filepath"),
            ],
        )],
    )
    spawn_item_nodes = []
    for item in task_config:
        lwh = item["lwh"]
        for i in range(item["count"]):
            name = item["name"] + str(i)
            x, y, z, _, _, _ = item["poses"][i]
            color = item["colors"][i]
            spawn_node = spawn_create_item_node(
                name,
                resolve_model_description(item['path'], ros2_transformers_dir, name, color, lwh),
                x,
                y,
                z,
            )
            spawn_item_nodes.append(spawn_node)

    # Examples: https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[
            {
                "config_file":
                    path.join(ros2_transformers_dir,
                              "config/ros_gz_bridge.yaml")
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
                "use_sim_time": True,
                "subscription_heartbeat": 100
            },
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

    rt1_demo_app_node = Node(
        package="ros2_transformers",
        executable="rt1_demo_app",
        output="screen",
        parameters=[
            get_launch_arg_overrides(app_config),
            # kinematics_config,
        ],
        ros_arguments=["--log-level",
                       LaunchConfiguration("log-level")],
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            "world_filepath",
            default_value=PathJoinSubstitution([
                FindPackageShare("ros2_transformers"),
                "sim/worlds/empty.sdf",
            ]),
            description="the file path to the Gazebo world file to load.",
        ),
        DeclareLaunchArgument(
            "gz_args",
            default_value="-r",
            description="Arguments to pass to Gazebo.",
        ),
        DeclareLaunchArgument(
            "task_name",
            default_value="stack_blocks",
            description="Task name of yaml file in tasks directory.",
        ),
        DeclareLaunchArgument("log-level", default_value="info"),
        *generate_ros_param_args(app_config),
        set_gz_resource_path,
        set_gz_sdf_path,
        bridge,
        start_gz_world,
        *spawn_item_nodes,
        # spawn_block2,
        # spawn_block3,
        rt1_demo_app_node,
        # RegisterEventHandler(
        # OnProcessStart(
        #     target_action=rt1_demo_app,
        #     on_start=[
        #         LogInfo(msg='App started, launching moveit'),
        #         spawn_turtle
        #     ]
        # )
        # ),
        # moveit_rviz_node,
    ])
