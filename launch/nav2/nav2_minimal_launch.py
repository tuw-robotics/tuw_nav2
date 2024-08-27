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
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.launch_context import LaunchContext
from tuw_common.launch import RewrittenYaml
from nav2_common.launch import ReplaceString


def generate_launch_description():
    # Get the launch directory
    this_pgk_dir = get_package_share_directory('tuw_nav2')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    namespace = LaunchConfiguration('namespace')
    launch_lifecycle_manager = LaunchConfiguration('launch_lifecycle_manager')
     

    controller_server_yaml_arg  = DeclareLaunchArgument( 'controller_server_yaml',  default_value='controller_server_purepursuite.yaml')
    controller_server_yaml_cfg  = LaunchConfiguration('controller_server_yaml')
    bt_navigator_yaml_arg       = DeclareLaunchArgument( 'bt_navigator_yaml',       default_value='bt_navigator.yaml')
    bt_navigator_yaml_cfg       = LaunchConfiguration('bt_navigator_yaml')
    behavior_server_yaml_arg    = DeclareLaunchArgument( 'behavior_server_yaml',    default_value='behavior_server.yaml')
    behavior_server_yaml_cfg    = LaunchConfiguration('behavior_server_yaml')
    planner_server_yaml_arg     = DeclareLaunchArgument( 'planner_server_yaml',     default_value='planner_server.yaml')    
    planner_server_yaml_cfg     = LaunchConfiguration('planner_server_yaml')

    vehilce_arg = DeclareLaunchArgument(
        'vehilce',
        default_value='pioneer3dx',
        description='Vehilce used and configuration folder used: ./nav2/$vehilce/$version/..')

    version_arg = DeclareLaunchArgument(
        'version',
        default_value='v1',
        description='Vehilce used and configuration folder used: ./nav2/$vehilce/$version/..')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

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
    
    launch_lifecycle_manager_arg = DeclareLaunchArgument(
        'launch_lifecycle_manager',
        default_value='True',
        description='launches lifecycle manager')
    
    def create_full_path_configurations(context: LaunchContext):
        
        def create_rewritten_yaml(context: LaunchContext, source_file_name):
            #'topic': '/' + context.launch_configurations['namespace'] + '/scan',
            param_substitutions = {
                'use_sim_time': use_sim_time,
                'autostart': autostart
            }
            configuration_yaml = os.path.join(
                this_pgk_dir,
                'config', 'nav2',
                context.launch_configurations['vehilce'],
                context.launch_configurations['version'],
                source_file_name)
            if context.launch_configurations['namespace']:
                tmp_file = ReplaceString(
                        source_file=configuration_yaml,
                        replacements={'/scan': ('/', namespace, '/scan')})
                configuration_yaml = RewrittenYaml(
                        source_file=tmp_file,
                        oginal_file=configuration_yaml,
                        root_key=namespace,
                        param_rewrites=param_substitutions,
                        convert_types=True)
            
            return configuration_yaml;
    

        controller_server_param_rewritten = create_rewritten_yaml(context, context.launch_configurations['controller_server_yaml'])
        bt_navigator_yaml_param_rewritten = create_rewritten_yaml(context, context.launch_configurations['bt_navigator_yaml'])
        behavior_server_param_rewritten = create_rewritten_yaml(context, context.launch_configurations['behavior_server_yaml'])
        planner_server_param_rewritten = create_rewritten_yaml(context, context.launch_configurations['planner_server_yaml'])
    
        return [SetLaunchConfiguration('controller_server_param_file_path', controller_server_param_rewritten),
                SetLaunchConfiguration('bt_navigator_yaml_param_file_path', bt_navigator_yaml_param_rewritten),
                SetLaunchConfiguration('behavior_server_param_file_path', behavior_server_param_rewritten),
                SetLaunchConfiguration('planner_server_param_file_path', planner_server_param_rewritten),
            ]

    create_full_path_configurations_arg = OpaqueFunction(function=create_full_path_configurations)

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    load_nodes = GroupAction(
        actions=[
            Node(
                condition=IfCondition(PythonExpression( ["'", controller_server_yaml_cfg, "' != 'empty'"]  )),
                package='nav2_controller',
                executable='controller_server',
                namespace=namespace,
                output='screen',
                respawn_delay=2.0,
                parameters=[LaunchConfiguration('controller_server_param_file_path'),
                            {'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                condition=IfCondition(PythonExpression( ["'", planner_server_yaml_cfg, "' != 'empty'"]  )),
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                namespace=namespace,
                output='screen',
                respawn_delay=2.0,
                parameters=[LaunchConfiguration('planner_server_param_file_path'),
                            {'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                condition=IfCondition(PythonExpression( ["'", behavior_server_yaml_cfg, "' != 'empty'"]  )),
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                namespace=namespace,
                output='screen',
                respawn_delay=2.0,
                parameters=[LaunchConfiguration('behavior_server_param_file_path'),
                            {'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                condition=IfCondition(PythonExpression( ["'", bt_navigator_yaml_cfg, "' != 'empty'"]  )),
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                namespace=namespace,
                output='screen',
                respawn_delay=2.0,
                parameters=[LaunchConfiguration('bt_navigator_yaml_param_file_path'),
                            {'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'default_nav_through_poses_bt_xml':get_package_share_directory('tuw_nav2') + "/config/nav2_bt_navigator/navigate_through_poses_w_replanning_and_recovery_v3.xml"},
                            {'default_nav_to_pose_bt_xml': get_package_share_directory('tuw_nav2') + "/config/nav2_bt_navigator/navigate_to_pose_w_replanning_and_recovery.xml"}
                            ],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                condition=IfCondition(launch_lifecycle_manager),
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                namespace=namespace,
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
    ld.add_action(vehilce_arg)
    ld.add_action(version_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(namespace_arg)
    ld.add_action(controller_server_yaml_arg)
    ld.add_action(bt_navigator_yaml_arg)
    ld.add_action(behavior_server_yaml_arg)
    ld.add_action(planner_server_yaml_arg)
    ld.add_action(autostart_arg)
    ld.add_action(log_level_arg)
    ld.add_action(launch_lifecycle_manager_arg)
    
    #Opaque function call
    ld.add_action(create_full_path_configurations_arg)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)

    return ld
