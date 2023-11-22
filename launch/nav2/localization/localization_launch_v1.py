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
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from nav2_common.launch import ReplaceString





def generate_launch_description():
    # Get the launch directory
    this_pgk_dir = get_package_share_directory('tuw_nav2')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    amcl_yaml = LaunchConfiguration('amcl_yaml')
    init_pose_yaml = LaunchConfiguration('init_pose_yaml')
    
    
    
    lifecycle_nodes = ['amcl']

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        description='Top-level namespace')
        
    declare_amcl_yaml = DeclareLaunchArgument(
        'amcl_yaml',
        default_value='amcl.yaml',
        description='amcl parameter file name of robot')

    declare_init_pose_yaml = DeclareLaunchArgument(
        'init_pose_yaml',
        description='amcl init pose parameter file name of robot')
    
    declare_use_robot = DeclareLaunchArgument(
        'use_robot',
        default_value='pioneer3dx',
        description='Robot used and configuration folder used: ./amcl/$use_robot/$parameters_used/..')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the localization stack')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')


    def create_full_path_configurations(context):
        amcl_param_file_path = os.path.join(
            this_pgk_dir,
            'config', 'amcl',
            context.launch_configurations['use_robot'],
            'v1',
            context.launch_configurations['amcl_yaml'])
        print(amcl_param_file_path)
        
        amcl_init_param_file_path = os.path.join(
            this_pgk_dir,
            'config', 'amcl',
            context.launch_configurations['use_robot'],
            context.launch_configurations['init_pose_yaml'])
        print(amcl_init_param_file_path)
     
        return [SetLaunchConfiguration('amcl_param_file_path', amcl_param_file_path),
                SetLaunchConfiguration('amcl_init_param_file_path', amcl_init_param_file_path)]

    create_full_path_configurations_arg = OpaqueFunction(function=create_full_path_configurations)

    namespaced_amcl_yaml = ReplaceString(
            source_file=LaunchConfiguration('amcl_param_file_path'),
            replacements={'<robot_namespace>': (namespace)})
    
    
    load_nodes = GroupAction(
        actions=[
            Node(
            	namespace = namespace,
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn_delay=2.0,
                parameters=[namespaced_amcl_yaml,
                            LaunchConfiguration('amcl_init_param_file_path'),
                            {'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level]),
            Node(
            	namespace= namespace,
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}])
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_amcl_yaml)
    ld.add_action(declare_init_pose_yaml)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_robot)

    #Opaque function call
    ld.add_action(create_full_path_configurations_arg)
    
    # Add the actions to launch all of the localiztion nodes
    ld.add_action(load_nodes)

    return ld
