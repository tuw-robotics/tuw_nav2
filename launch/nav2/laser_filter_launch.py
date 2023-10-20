
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

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    scan_src_arg = DeclareLaunchArgument(
        'scan_src',
        default_value='scan_raw',
        description='laser scan source')
    
    scan_des_arg = DeclareLaunchArgument(
        'scan_des',
        default_value='scan',
        description='laser scan destination')
    
    vehilce_arg = DeclareLaunchArgument(
        'vehilce',
        default_value='pioneer3dx',
        description='Robot used and configuration folder used: ./laser_filter/$vehilce/$filter_yaml')
    
    filter_yaml_arg  = DeclareLaunchArgument( 
        'filter_yaml',  
        default_value='shadow_filter.yaml',
        description='filter configuration')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Used namespace')
    
    def create_full_path_configurations(context):
        filter_file_path = os.path.join(
            this_pgk_dir,
            'config', 'laser_filter',
            context.launch_configurations['vehilce'],
            context.launch_configurations['filter_yaml'])
        print(filter_file_path)
        return [SetLaunchConfiguration('filter_file_path', filter_file_path)]

    create_full_path_configurations_arg = OpaqueFunction(function=create_full_path_configurations)

    return LaunchDescription([
        use_sim_time_arg, 
        vehilce_arg,
        filter_yaml_arg,
        scan_src_arg, 
        scan_des_arg, 
        namespace_arg, 
        create_full_path_configurations_arg,
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",                              
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('filter_file_path'),
                        {'use_sim_time': use_sim_time}],
            remappings=[
                ("scan", LaunchConfiguration('scan_src')),
                ("scan_filtered", LaunchConfiguration('scan_des')),]

        )
    ])
