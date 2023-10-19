
import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch_ros.actions import Node
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    this_pgk = 'tuw_nav2'
    this_pgk_dir = get_package_share_directory(this_pgk)

    use_sim_time_cfg = LaunchConfiguration('use_sim_time')
    autostart_cfg = LaunchConfiguration('autostart')
    log_level_cfg = LaunchConfiguration('log_level')

    lifecycle_nodes = ['map_server']

    # Declare the launch arguments
    namespace_cfg = LaunchConfiguration('namespace')
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description=('Top-level namespace. The value will be used to replace the '
                     '<robot_namespace> keyword on the rviz config file.'))

    use_sim_time_cfg = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo/Stage) clock if true')
    
    environment_arg = DeclareLaunchArgument(
        'environment',
        description=('The enviroment defines the map folder used '
                     '(cave, hallway, ...).'))
    
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')
    
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    
    def create_full_path_configurations(context):
        map_file_path = os.path.join(
            this_pgk_dir,
            'config','maps',
            context.launch_configurations['environment'],
            'map.yaml')
        print(map_file_path)
        return [SetLaunchConfiguration('yaml_filename', map_file_path)]

    create_map_yaml_filename_arg = OpaqueFunction(function=create_full_path_configurations)

    return LaunchDescription([
        use_sim_time_arg,
        namespace_arg,
        autostart_arg,
        log_level_arg,
        environment_arg,
        create_map_yaml_filename_arg,
        Node(
            package='nav2_map_server',
            executable='map_server',
            namespace=namespace_cfg,
            output='screen',
            parameters=[{'yaml_filename': LaunchConfiguration('yaml_filename')}]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            namespace=namespace_cfg,
            name='lifecycle_manager_map',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level_cfg],
            parameters=[{'use_sim_time': use_sim_time_cfg},
                        {'autostart': autostart_cfg},
                        {'node_names': lifecycle_nodes}])
    ])
