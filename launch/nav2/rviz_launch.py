import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    this_directory = get_package_share_directory('tuw_nav2')

    # Create the launch configuration variables

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
        
    rviz_config_cfg = LaunchConfiguration('config')
    rviz_config_arg = DeclareLaunchArgument(
        'config',
        default_value=TextSubstitution(text='nav2'),
        description='Use empty, cave or roblab to load a TUW enviroment')
    
    def rviz_launch_configuration(context):
        file = os.path.join(
            this_directory,
            #'..', '..', '..', '..', 'src', 'tuw_nav2', 'config', 'rviz',
            'config', 'rviz',
            context.launch_configurations['config'] + '.rviz')
        return [SetLaunchConfiguration('config', file)]

    rviz_launch_configuration_arg = OpaqueFunction(function=rviz_launch_configuration)

    start_rviz_multiple_tf_trees_arg = Node(
        package='rviz2',
        executable='rviz2',
        namespace=namespace_cfg,
        arguments=['-d', rviz_config_cfg, '-t', '{NAMESPACE} - {CONFIG_PATH}/{CONFIG_FILENAME} - RViz2'],
        output='screen',
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/camera_info', 'camera_info'),
                    ('/goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose')],
        parameters=[{
                "use_sim_time": use_sim_time_cfg}])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(use_sim_time_arg)
    ld.add_action(namespace_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(rviz_launch_configuration_arg)

    # Add any conditioned actions
    ld.add_action(start_rviz_multiple_tf_trees_arg)

    return ld
