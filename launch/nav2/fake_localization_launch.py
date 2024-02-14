import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration


def generate_launch_description():
    # Get the launch directory
    this_pgk_dir = get_package_share_directory("tuw_nav2")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    launch_lifecycle_manager = LaunchConfiguration("launch_lifecycle_manager")

    # Declare the launch arguments
    namespace = LaunchConfiguration("namespace")
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description=(
            "Top-level namespace. The value will be used to replace the "
            "<robot_namespace> keyword on the rviz config file."
        ),
    )

    lifecycle_nodes = ["fake_localization_node"]

    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("odom_ground_truth", "ground_truth"),
    ]

    fake_localization_yaml_arg = DeclareLaunchArgument(
        "fake_localization_yaml",
        default_value="fake_localization.yaml",
        description="fake_localization parameter file name",
    )

    init_pose_yaml_arg = DeclareLaunchArgument(
        "init_pose_yaml",
        default_value="init_pose.yaml",
        description="init pose parameter file name",
    )

    vehilce_arg = DeclareLaunchArgument(
        "vehilce",
        default_value="pioneer3dx",
        description="Robot used and configuration folder used: ./fake_localization/$vehilce/$parameters_used/..",
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="", description="Used namespace"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the localization stack",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    launch_lifecycle_manager_arg = DeclareLaunchArgument(
        "launch_lifecycle_manager",
        default_value="True",
        description="launches lifecycle manager",
    )

    def create_full_path_configurations(context):
        param_file_path = os.path.join(
            this_pgk_dir,
            "config",
            "fake_localization",
            context.launch_configurations["vehilce"],
            context.launch_configurations["fake_localization_yaml"],
        )
        print(param_file_path)
        init_param_file_path = os.path.join(
            this_pgk_dir,
            "config",
            "fake_localization",
            context.launch_configurations["vehilce"],
            context.launch_configurations["init_pose_yaml"],
        )
        print(init_param_file_path)
        return [
            SetLaunchConfiguration("param_file_path", param_file_path),
            SetLaunchConfiguration("init_param_file_path", init_param_file_path),
        ]

    create_full_path_configurations_arg = OpaqueFunction(
        function=create_full_path_configurations
    )

    load_nodes = GroupAction(
        actions=[
            Node(
                package="tuw_fake_localization",
                executable="fake_localization_node",
                name="fake_localization_node",
                namespace=namespace,
                output="screen",
                respawn_delay=2.0,
                parameters=[
                    LaunchConfiguration("param_file_path"),
                    LaunchConfiguration("init_param_file_path"),
                    {"use_sim_time": use_sim_time},
                ],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                condition=IfCondition(launch_lifecycle_manager),
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                namespace=namespace,
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(fake_localization_yaml_arg)
    ld.add_action(init_pose_yaml_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(namespace_arg)
    ld.add_action(autostart_arg)
    ld.add_action(log_level_arg)
    ld.add_action(launch_lifecycle_manager_arg)
    ld.add_action(vehilce_arg)
    # Opaque function call
    ld.add_action(create_full_path_configurations_arg)

    # Add the actions to launch all of the localization nodes
    ld.add_action(load_nodes)

    return ld
