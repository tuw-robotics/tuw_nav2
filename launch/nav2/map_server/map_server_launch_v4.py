
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
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_filename = LaunchConfiguration('map_yaml_filename')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    
    lifecycle_nodes = ['map_server']
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo/Stage) clock if true')
    
    environment_arg = DeclareLaunchArgument(
        'use_environment',
        default_value='cave',
        description='Map file used: /maps/$use_environment/map.yaml')
    
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
            context.launch_configurations['use_environment'],
            'map.yaml')
        print(map_file_path)
        return [SetLaunchConfiguration('map_file_path', map_file_path)]

    create_map_file_path_arg = OpaqueFunction(function=create_full_path_configurations)
    
    return LaunchDescription([
        use_sim_time_arg,
        namespace_arg,
        environment_arg,
        autostart_arg,
        log_level_arg,
        create_map_file_path_arg,
        Node(
            namespace = namespace,
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            respawn_delay=2.0,
            parameters=[{'use_sim_time': use_sim_time},
            		{'frame_id':"map"},
                        {'yaml_filename': LaunchConfiguration('map_file_path')}],
            arguments=['--ros-args', '--log-level', log_level]),
        Node(
            namespace = namespace,
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'bond_timeout':0.0},
                        {'node_names': lifecycle_nodes}])
    ])
