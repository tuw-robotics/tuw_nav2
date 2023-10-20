import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    # Get the launch directory
    this_pgk_dir = get_package_share_directory('tuw_nav2')
    launch_dir = os.path.join(this_pgk_dir, 'launch', 'nav2')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    namespace = LaunchConfiguration('namespace')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Used namespace')
    
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    
    environment_arg = DeclareLaunchArgument(
        'environment',
        description=('The enviroment defines the map folder used '
                     '(cave, hallway, ...).'))
    
    vehilce_arg = DeclareLaunchArgument(
        'vehilce',
        default_value='pioneer3dx',
        description='Robot used and configuration folder used: ./amcl/$vehilce/$parameters_used/..')
    
    init_pose_yaml_arg = DeclareLaunchArgument(
        'init_pose_yaml',
        default_value='init_pose_cave.yaml',
        description='amcl init pose parameter file name')
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='nav2',
        description='RViz Configuration used without \".rviz\" ending')
    
    scan_src_arg = DeclareLaunchArgument(
        'scan_src',
        default_value='base_scan',
        description='laser scan source')
    
    scan_des_arg = DeclareLaunchArgument(
        'scan_des',
        default_value='scan',
        description='laser scan destination')
    
    # Specify the actions
    group_action = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'config':  LaunchConfiguration('rviz_config')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'laser_filter_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'filter_yaml': 'shadow_filter_stage.yaml',
                              'scan_src': LaunchConfiguration('scan_src'),
                              'scan_des': LaunchConfiguration('scan_des'),
                              'vehilce': LaunchConfiguration('vehilce')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'map_server_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'log_level': log_level,
                              'environment': LaunchConfiguration('environment')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'log_level': log_level,
                              'vehilce': LaunchConfiguration('vehilce'),
                              'init_pose_yaml': LaunchConfiguration('init_pose_yaml')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav2_minimal_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'log_level': log_level}.items())
    ])

    return LaunchDescription([
        use_sim_time_arg,
        namespace_arg,
        autostart_arg,
        log_level_arg,
        environment_arg,
        vehilce_arg,
        init_pose_yaml_arg,
        rviz_config_arg,
        scan_src_arg,
        scan_des_arg,
        group_action
    ])
