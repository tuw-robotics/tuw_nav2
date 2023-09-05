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



def generate_launch_description():
    # Get the launch directory
    this_pgk_dir = get_package_share_directory('tuw_nav2')
  
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['map_server', 'amcl']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    declare_amcl_yaml = DeclareLaunchArgument(
        'amcl_yaml',
        default_value='amcl.yaml',
        description='amcl parameter file name')

    declare_init_pose_yaml = DeclareLaunchArgument(
        'init_pose_yaml',
        default_value='init_pose.yaml',
        description='amcl init pose parameter file name')
    
    declare_map_yaml = DeclareLaunchArgument(
        'map_yaml',
        default_value='map.yaml',
        description='map yaml file name')
    
    declare_use_robot = DeclareLaunchArgument(
        'use_robot',
        default_value='pioneer3dx',
        description='Robot used and configuration folder used: ./amcl/$use_robot/$parameters_used/..')
    
    declare_use_environment = DeclareLaunchArgument(
        'use_environment',
        default_value='cave',
        description='Map file used: /maps/$use_environment/map.yaml')

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
            context.launch_configurations['amcl_yaml'])
        print(amcl_param_file_path)
        amcl_init_param_file_path = os.path.join(
            this_pgk_dir,
            'config', 'amcl',
            context.launch_configurations['use_robot'],
            context.launch_configurations['init_pose_yaml'])
        print(amcl_init_param_file_path)
        particle_filter_map_file_path = os.path.join(
            this_pgk_dir,
            'config','maps',
            context.launch_configurations['use_environment'],
            context.launch_configurations['map_yaml'])
        print(particle_filter_map_file_path)
        return [SetLaunchConfiguration('amcl_param_file_path', amcl_param_file_path),
                SetLaunchConfiguration('amcl_init_param_file_path', amcl_init_param_file_path),
                SetLaunchConfiguration('particle_filter_map_file_path', particle_filter_map_file_path)]

    create_full_path_configurations_arg = OpaqueFunction(function=create_full_path_configurations)

    load_nodes = GroupAction(
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn_delay=2.0,
                parameters=[{'use_sim_time': use_sim_time},
                            {'yaml_filename': LaunchConfiguration('particle_filter_map_file_path')}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn_delay=2.0,
                parameters=[LaunchConfiguration('amcl_param_file_path'),
                            LaunchConfiguration('amcl_init_param_file_path'),
                            {'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
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
    ld.add_action(declare_amcl_yaml)
    ld.add_action(declare_init_pose_yaml)
    ld.add_action(declare_map_yaml)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_robot)
    ld.add_action(declare_use_environment)

    #Opaque function call
    ld.add_action(create_full_path_configurations_arg)
    
    # Add the actions to launch all of the localiztion nodes
    ld.add_action(load_nodes)

    return ld
