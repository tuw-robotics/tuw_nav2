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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.launch_context import LaunchContext
from launch import utilities
from launch_ros.descriptions import ParameterFile
from tuw_common.launch import RewrittenYaml
from nav2_common.launch import ReplaceString



def generate_launch_description():
    # Get the launch directory
    this_pgk_dir = get_package_share_directory('tuw_nav2')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    namespace = LaunchConfiguration('namespace')
     

    declare_controller_server_yaml  = DeclareLaunchArgument( 'controller_server_yaml',  default_value='controller_server_purepursuite.yaml')
    argument_controller_server_yaml = LaunchConfiguration('controller_server_yaml')
    declare_bt_navigator_yaml       = DeclareLaunchArgument( 'bt_navigator_yaml',       default_value='bt_navigator.yaml')
    argument_bt_navigator_yaml      = LaunchConfiguration('bt_navigator_yaml')
    declare_behavior_server_yaml    = DeclareLaunchArgument( 'behavior_server_yaml',    default_value='behavior_server.yaml')
    argument_behavior_server_yaml   = LaunchConfiguration('behavior_server_yaml')
    declare_planner_server_yaml     = DeclareLaunchArgument( 'planner_server_yaml',     default_value='planner_server.yaml')    
    argument_planner_server_yaml    = LaunchConfiguration('planner_server_yaml')
    
    declare_use_robot = DeclareLaunchArgument(
        'use_robot',
        default_value='pioneer3dx',
        description='Robot used and configuration folder used: ./nav2/$use_robot/$use_version/..')

    declare_use_version = DeclareLaunchArgument(
        'use_version',
        default_value='v1',
        description='Robot used and configuration folder used: ./nav2/$use_robot/$use_version/..')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Used namespace')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    



    def create_full_path_configurations(context: LaunchContext):
        
        def create_rewritten_yaml(context: LaunchContext, source_file_name):
            #'topic': '/' + context.launch_configurations['namespace'] + '/scan',
            param_substitutions = {
                'use_sim_time': use_sim_time,
                'autostart': autostart}
            source_file_path = os.path.join(
                this_pgk_dir,
                'config', 'nav2',
                context.launch_configurations['use_robot'],
                context.launch_configurations['use_version'],
                source_file_name)
            tmp_file = ReplaceString(
                    source_file=source_file_path,
                    replacements={'/scan': ('/', namespace, '/scan')})
            tmp_file2 = RewrittenYaml(
                    source_file=tmp_file,
                    oginal_file=source_file_path,
                    root_key=namespace,
                    param_rewrites=param_substitutions,
                    convert_types=True)
            
            # destination_file_rewritten = os.path.join('tmp', source_file_name)
            # os.rename(source_file_rewritten, destination_file_rewritten)
            return tmp_file2;
    

        controller_server_param_rewritten = create_rewritten_yaml(context, context.launch_configurations['controller_server_yaml'])
        bt_navigator_yaml_param_rewritten = create_rewritten_yaml(context, context.launch_configurations['bt_navigator_yaml'])
        behavior_server_param_rewritten = create_rewritten_yaml(context, context.launch_configurations['behavior_server_yaml'])
        planner_server_param_rewritten = create_rewritten_yaml(context, context.launch_configurations['planner_server_yaml'])
    
        return [SetLaunchConfiguration('controller_server_param_file_path', controller_server_param_rewritten),
                SetLaunchConfiguration('bt_navigator_yaml_param_file_path', bt_navigator_yaml_param_rewritten),
                SetLaunchConfiguration('behavior_server_param_file_path', behavior_server_param_rewritten),
                SetLaunchConfiguration('planner_server_param_file_path', planner_server_param_rewritten)]

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
                condition=IfCondition(PythonExpression( ["'", argument_controller_server_yaml, "' != 'empty'"]  )),
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
                condition=IfCondition(PythonExpression( ["'", argument_planner_server_yaml, "' != 'empty'"]  )),
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
                condition=IfCondition(PythonExpression( ["'", argument_behavior_server_yaml, "' != 'empty'"]  )),
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
                condition=IfCondition(PythonExpression( ["'", argument_bt_navigator_yaml, "' != 'empty'"]  )),
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                namespace=namespace,
                output='screen',
                respawn_delay=2.0,
                parameters=[LaunchConfiguration('bt_navigator_yaml_param_file_path'),
                            {'use_sim_time': use_sim_time,
                             'autostart': autostart}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
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
    ld.add_action(declare_use_robot)
    ld.add_action(declare_use_version)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_controller_server_yaml)
    ld.add_action(declare_bt_navigator_yaml)
    ld.add_action(declare_behavior_server_yaml)
    ld.add_action(declare_planner_server_yaml)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_log_level_cmd)
    
    #Opaque function call
    ld.add_action(create_full_path_configurations_arg)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)

    return ld
