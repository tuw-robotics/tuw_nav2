from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    this_pgk = 'tuw_nav2'
    this_pgk_dir = get_package_share_directory(this_pgk)
    
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_src = LaunchConfiguration('scan_src')
    scan_des = LaunchConfiguration('scan_des')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        description='Top-level namespace')
      
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_scan_src_cmd = DeclareLaunchArgument(
        'scan_src',
        default_value='scan_raw',
        description='laser scan source')
    
    declare_scan_des_cmd = DeclareLaunchArgument(
        'scan_des',
        default_value='scan',
        description='laser scan destination')
    
    return LaunchDescription([
    	declare_namespace_cmd,
        declare_use_sim_time_cmd, 
        declare_scan_src_cmd, 
        declare_scan_des_cmd, 
        Node(
            namespace=namespace,
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[PathJoinSubstitution([
                    this_pgk_dir, "config/laser_filter/p3dx", "shadow_filter_example.yaml",
                ])],
            remappings=[
                ("scan", scan_src),
                ("scan_filtered", scan_des),]

        ),
    ])
