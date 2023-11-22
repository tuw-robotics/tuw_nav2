# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from nav2_common.launch import ReplaceString

def generate_launch_description():
    # Get the launch directory
    this_pgk_dir = get_package_share_directory('tuw_nav2')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    
    declare_controller_server_yaml  = DeclareLaunchArgument( 'controller_server_yaml',  default_value='controller_server_purepursuite.yaml')
    argument_controller_server_yaml = LaunchConfiguration('controller_server_yaml')
    declare_bt_navigator_yaml       = DeclareLaunchArgument( 'bt_navigator_yaml',       default_value='bt_navigator.yaml')
    argument_bt_navigator_yaml      = LaunchConfiguration('bt_navigator_yaml')
    declare_smoother_server_yaml    = DeclareLaunchArgument( 'smoother_server_yaml',    default_value='smoother_server.yaml')
    argument_smoother_server_yaml   = LaunchConfiguration('smoother_server_yaml')
    declare_behavior_server_yaml    = DeclareLaunchArgument( 'behavior_server_yaml',    default_value='behavior_server.yaml')
    argument_behavior_server_yaml   = LaunchConfiguration('behavior_server_yaml')
    declare_waypoint_follower_yaml  = DeclareLaunchArgument( 'waypoint_follower_yaml',  default_value='waypoint_follower.yaml')
    argument_waypoint_follower_yaml = LaunchConfiguration('waypoint_follower_yaml')
    declare_planner_server_yaml     = DeclareLaunchArgument( 'planner_server_yaml',     default_value='planner_server.yaml')    
    argument_planner_server_yaml    = LaunchConfiguration('planner_server_yaml')
    declare_velocity_smoother_yaml  = DeclareLaunchArgument( 'velocity_smoother_yaml',  default_value='velocity_smoother.yaml')
    argument_velocity_smoother_yaml = LaunchConfiguration('velocity_smoother_yaml')
   
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        description='Top-level namespace')

    declare_use_robot = DeclareLaunchArgument(
        'use_robot',
        default_value='pioneer3dx',
        description='Robot used and configuration folder used: ./nav2/$use_robot/$use_version/..')

    declare_use_version = DeclareLaunchArgument(
        'use_version',
        default_value='v3',
        description='Configuration folder used: ./nav2/$use_robot/$use_version/..')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    def create_full_path_configurations(context):
        controller_server_param_file_path = os.path.join(
            this_pgk_dir,
            'config', 'nav2',
            context.launch_configurations['use_robot'],
            context.launch_configurations['use_version'],
            context.launch_configurations['controller_server_yaml'])
        print(controller_server_param_file_path)
        bt_navigator_yaml_param_file_path = os.path.join(
            this_pgk_dir,
            'config', 'nav2',
            context.launch_configurations['use_robot'],
            context.launch_configurations['use_version'],
            context.launch_configurations['bt_navigator_yaml'])
        print(bt_navigator_yaml_param_file_path)
        behavior_server_param_file_path = os.path.join(
            this_pgk_dir,
            'config', 'nav2',
            context.launch_configurations['use_robot'],
            context.launch_configurations['use_version'],
            context.launch_configurations['behavior_server_yaml'])
        print(behavior_server_param_file_path)
        smoother_server_param_file_path = os.path.join(
            this_pgk_dir,
            'config', 'nav2',
            context.launch_configurations['use_robot'],
            context.launch_configurations['use_version'],
            context.launch_configurations['smoother_server_yaml'])
        print(smoother_server_param_file_path)
        waypoint_follower_param_file_path = os.path.join(
            this_pgk_dir,
            'config', 'nav2',
            context.launch_configurations['use_robot'],
            context.launch_configurations['use_version'],
            context.launch_configurations['waypoint_follower_yaml'])
        print(waypoint_follower_param_file_path)
        planner_server_param_file_path = os.path.join(
            this_pgk_dir,
            'config', 'nav2',
            context.launch_configurations['use_robot'],
            context.launch_configurations['use_version'],
            context.launch_configurations['planner_server_yaml'])
        print(planner_server_param_file_path)
        velocity_smoother_param_file_path = os.path.join(
            this_pgk_dir,
            'config', 'nav2',
            context.launch_configurations['use_robot'],
            context.launch_configurations['use_version'],
            context.launch_configurations['velocity_smoother_yaml'])
        print(velocity_smoother_param_file_path)
 	
     
        
        return [SetLaunchConfiguration('controller_server_param_file_path', controller_server_param_file_path),
                SetLaunchConfiguration('bt_navigator_yaml_param_file_path', bt_navigator_yaml_param_file_path),
                SetLaunchConfiguration('behavior_server_param_file_path', behavior_server_param_file_path),
                SetLaunchConfiguration('smoother_server_param_file_path', smoother_server_param_file_path),
                SetLaunchConfiguration('waypoint_follower_param_file_path', waypoint_follower_param_file_path),
                SetLaunchConfiguration('planner_server_param_file_path', planner_server_param_file_path),
                SetLaunchConfiguration('velocity_smoother_param_file_path', velocity_smoother_param_file_path)]

    create_full_path_configurations_arg = OpaqueFunction(function=create_full_path_configurations)


    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']


    namespaced_controller_server_yaml = ReplaceString(
            source_file=LaunchConfiguration('controller_server_param_file_path'),
            replacements={'<robot_namespace>': (namespace)})
    namespaced_bt_navigator_yaml = ReplaceString(
            source_file=LaunchConfiguration('bt_navigator_yaml_param_file_path'),
            replacements={'<robot_namespace>': (namespace)})
    namespaced_smoother_server_yaml = ReplaceString(
            source_file=LaunchConfiguration('smoother_server_param_file_path'),
            replacements={'<robot_namespace>': (namespace)})     
    namespaced_behavior_server_yaml = ReplaceString(
            source_file=LaunchConfiguration('behavior_server_param_file_path'),
            replacements={'<robot_namespace>': (namespace)})
    namespaced_waypoint_follower_yaml = ReplaceString(
            source_file=LaunchConfiguration('waypoint_follower_param_file_path'),
            replacements={'<robot_namespace>': (namespace)})
    namespaced_planner_server_yaml = ReplaceString(
            source_file=LaunchConfiguration('planner_server_param_file_path'),
            replacements={'<robot_namespace>': (namespace)})
    namespaced_velocity_smoother_yaml = ReplaceString(
            source_file=LaunchConfiguration('velocity_smoother_param_file_path'),
            replacements={'<robot_namespace>': (namespace)})
            
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    controller_remappings = remappings + [('cmd_vel', 'cmd_vel_nav')]
    velocity_smoother_remappings = remappings + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]

    load_nodes = GroupAction(
        actions=[
            Node(
                condition=IfCondition(PythonExpression( ["'", argument_controller_server_yaml, "' != 'empty'"]  )),
                namespace= namespace,
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn_delay=2.0,
                parameters=[namespaced_controller_server_yaml,
                            {'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=controller_remappings),
            Node(
                condition=IfCondition(PythonExpression( ["'", argument_smoother_server_yaml, "' != 'empty'"]  )),
                namespace= namespace,
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn_delay=2.0,
                remappings=remappings,
                parameters=[namespaced_smoother_server_yaml,
                            {'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level]),
            Node(
                condition=IfCondition(PythonExpression( ["'", argument_planner_server_yaml, "' != 'empty'"]  )),
                namespace= namespace,
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn_delay=2.0,
                remappings=remappings,
                parameters=[namespaced_planner_server_yaml,
                            {'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level]),
            Node(
                condition=IfCondition(PythonExpression( ["'", argument_behavior_server_yaml, "' != 'empty'"]  )),
                namespace= namespace,
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn_delay=2.0,
                remappings=remappings,
                parameters=[namespaced_behavior_server_yaml,
                            {'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level]),
            Node(
                condition=IfCondition(PythonExpression( ["'", argument_bt_navigator_yaml, "' != 'empty'"]  )),
                namespace= namespace,
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn_delay=2.0,
                remappings=remappings,
                parameters=[namespaced_bt_navigator_yaml,
                            {'use_sim_time': use_sim_time,
                             'autostart': autostart}],
                arguments=['--ros-args', '--log-level', log_level]),
            Node(
                condition=IfCondition(PythonExpression( ["'", argument_waypoint_follower_yaml, "' != 'empty'"]  )),
                namespace= namespace,
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn_delay=2.0,
                remappings=remappings,
                parameters=[namespaced_waypoint_follower_yaml,
                            {'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level]),
            Node(
                condition=IfCondition(PythonExpression( ["'", argument_velocity_smoother_yaml, "' != 'empty'"]  )),
                namespace= namespace,
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn_delay=2.0,
                parameters=[namespaced_velocity_smoother_yaml,
                            {'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings= velocity_smoother_remappings),
            Node(
            	namespace = namespace,
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_robot)
    ld.add_action(declare_use_version)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_controller_server_yaml)
    ld.add_action(declare_bt_navigator_yaml)
    ld.add_action(declare_behavior_server_yaml)
    ld.add_action(declare_smoother_server_yaml)
    ld.add_action(declare_waypoint_follower_yaml)
    ld.add_action(declare_planner_server_yaml)
    ld.add_action(declare_velocity_smoother_yaml)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)

    
    #Opaque function call
    ld.add_action(create_full_path_configurations_arg)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)

    return ld
