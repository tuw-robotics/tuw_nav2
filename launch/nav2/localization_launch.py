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
    this_pgk_dir = get_package_share_directory('tuw_nav2')
  
    use_sim_time_cfg = LaunchConfiguration('use_sim_time')
    autostart_cfg = LaunchConfiguration('autostart')
    log_level_cfg = LaunchConfiguration('log_level')

    # Declare the launch arguments
    namespace_cfg = LaunchConfiguration('namespace')
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description=('Top-level namespace. The value will be used to replace the '
                     '<robot_namespace> keyword on the rviz config file.'))

    lifecycle_nodes = ['amcl']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/initialpose', 'initialpose'),
                  ('/scan', 'scan')]

    amcl_yaml_arg = DeclareLaunchArgument(
        'amcl_yaml',
        default_value='amcl.yaml',
        description='amcl parameter file name')

    init_pose_yaml_arg = DeclareLaunchArgument(
        'init_pose_yaml',
        default_value='init_pose.yaml',
        description='amcl init pose parameter file name')
    
    vehilce_arg = DeclareLaunchArgument(
        'vehilce',
        default_value='pioneer3dx',
        description='Robot used and configuration folder used: ./amcl/$vehilce/$parameters_used/..')
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Used namespace')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the localization stack')

    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')


    def create_full_path_configurations(context):
        amcl_param_file_path = os.path.join(
            this_pgk_dir,
            'config', 'amcl',
            context.launch_configurations['vehilce'],
            context.launch_configurations['amcl_yaml'])
        print(amcl_param_file_path)
        amcl_init_param_file_path = os.path.join(
            this_pgk_dir,
            'config', 'amcl',
            context.launch_configurations['vehilce'],
            context.launch_configurations['init_pose_yaml'])
        print(amcl_init_param_file_path)
        return [SetLaunchConfiguration('amcl_param_file_path', amcl_param_file_path),
                SetLaunchConfiguration('amcl_init_param_file_path', amcl_init_param_file_path)]

    create_full_path_configurations_arg = OpaqueFunction(function=create_full_path_configurations)

    load_nodes = GroupAction(
        actions=[
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                namespace=namespace_cfg,
                output='screen',
                respawn_delay=2.0,
                parameters=[LaunchConfiguration('amcl_param_file_path'),
                            LaunchConfiguration('amcl_init_param_file_path'),
                            {'use_sim_time': use_sim_time_cfg}],
                arguments=['--ros-args', '--log-level', log_level_cfg],
                remappings=remappings),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                namespace=namespace_cfg,
                output='screen',
                arguments=['--ros-args', '--log-level', log_level_cfg],
                parameters=[{'use_sim_time': use_sim_time_cfg},
                            {'autostart': autostart_cfg},
                            {'node_names': lifecycle_nodes}])
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(amcl_yaml_arg)
    ld.add_action(init_pose_yaml_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(namespace_arg)
    ld.add_action(autostart_arg)
    ld.add_action(log_level_arg)
    ld.add_action(vehilce_arg)
    #Opaque function call
    ld.add_action(create_full_path_configurations_arg)
    
    # Add the actions to launch all of the localiztion nodes
    ld.add_action(load_nodes)

    return ld
