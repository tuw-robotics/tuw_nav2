
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    this_pgk = 'tuw_nav2'
    this_pgk_dir = get_package_share_directory(this_pgk)

    use_robot = LaunchConfiguration('use_robot')
    declare_use_robot_cmd = DeclareLaunchArgument(
        'use_robot',
        default_value='pioneer3dx',
        description='Robot used and configuration folder used: ./nav2/$use_robot/$use_version/..')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    scan_src = LaunchConfiguration('scan_src')
    declare_scan_src_cmd = DeclareLaunchArgument(
        'scan_src',
        default_value='scan_raw',
        description='laser scan source')
    
    scan_des = LaunchConfiguration('scan_des')
    declare_scan_des_cmd = DeclareLaunchArgument(
        'scan_des',
        default_value='scan',
        description='laser scan destination')
    
    filter_yaml = LaunchConfiguration('filter_yaml')
    declare_filter_yaml_cmd  = DeclareLaunchArgument( 
        'filter_yaml',  
        default_value='shadow_filter.yaml',
        description='filter configuration')

    namespace = LaunchConfiguration('namespace')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Used namespace')
    
    def create_full_path_configurations(context):
        filter_file_path = os.path.join(
            this_pgk_dir,
            'config', 'laser_filter',
            context.launch_configurations['use_robot'],
            context.launch_configurations['filter_yaml'])
        print(filter_file_path)
        return [SetLaunchConfiguration('filter_file_path', filter_file_path)]

    create_full_path_configurations_arg = OpaqueFunction(function=create_full_path_configurations)

    return LaunchDescription([
        declare_use_sim_time_cmd, 
        declare_use_robot_cmd,
        declare_filter_yaml_cmd,
        declare_scan_src_cmd, 
        declare_scan_des_cmd, 
        declare_namespace_cmd, 
        create_full_path_configurations_arg,
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            namespace=namespace,
            parameters=[LaunchConfiguration('filter_file_path')],
            remappings=[
                ("scan", scan_src),
                ("scan_filtered", scan_des),]

        )
    ])
