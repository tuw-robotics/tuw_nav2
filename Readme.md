# tuw_nav2 

## Command Hints

### teleop joy 
```
cd $WS
ros2 launch teleop_twist_joy teleop-launch.py  config_filepath:=$WS/src/tuw_nav2/config/teleop_twist_joy/f710.yaml
```

### slam
```
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$WS/src/tuw_nav2/config/slam_toolbox/mapper_params_online_async.yaml
```

### map save 
```
mkdir config/map
ros2 run nav2_map_server map_saver_cli -f config/map/cave
```

### map server
```
ros2 run nav2_map_server map_server --ros-args --params-file map_server_params.yaml
##
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```
### mouse_teleop
```
ros2 run mouse_teleop mouse_teleop --ros-args --remap mouse_vel:=cmd_vel
```

### nav2
```
ros2 launch tuw_nav2 rviz_launch.py
ros2 launch tuw_nav2 laser_filter.launch.py
ros2 launch tuw_nav2 localization_launch.py use_sim_time:=true init_pose_yaml:=init_pose_cave.yaml use_environment:=cave use_robot:=pioneer3dx
ros2 launch tuw_nav2 nav2_default_launch.py
```

#### debugging
config for the controller_server
```
ros2 launch tuw_nav2 nav2_minimal_launch.py  controller_server_yaml:=controller_server_purepursuite.yaml 
```

Not starting the node controller_server
```
ros2 launch tuw_nav2 nav2_minimal_launch.py  controller_server_yaml:=empty
```
Starting the controller server only
```
ros2 run nav2_controller controller_server --ros-args --params-file $WS/src/tuw_nav2/config/nav2/pioneer3dx/v1/controller_server_purepursuite.yaml
ros2 run nav2_controller controller_server --ros-args --params-file $WS/src/tuw_nav2/config/nav2/pioneer3dx/v1/controller_server_mppi.yaml
```

## tmuxinator

```
tmuxinator start -p $WS/src/tuw_nav2/tmux_cave_gazebo.yml
tmuxinator start -p $WS/src/tuw_nav2/tmux_cave_stage.yml 
```

## tmuxinator

```
tmuxinator start -p $WS/src/tuw_nav2/tmux_cave_gazebo.yml
tmuxinator start -p $WS/src/tuw_nav2/tmux_cave_stage.yml 
tmuxinator start -p ws02/src/tuw_nav2/tmux_cave_stage_multi_robot.yml
```