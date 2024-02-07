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
##### controller_server
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

##### planner_server
straight line planner
```
ros2 launch tuw_nav2 nav2_default_launch.py controller_server_yaml:=controller_server_mppi.yaml planner_server_yaml:=planner_server_line.yaml
```

starting the node planner_server extra
```
ros2 launch tuw_nav2 nav2_minimal_launch.py controller_server_yaml:=controller_server_mppi.yaml planner_server_yaml:=empty
ros2 run nav2_planner planner_server --ros-args --params-file ./ws02/src/tuw_nav2/config/nav2/pioneer3dx/v1/planner_server_line.yaml
```
## tmuxinator

```
cd $WS
tmuxinator start -p ./ws02/src/tuw_nav2/tmux/roblab_gazebo.yml
tmuxinator start -p ./ws02/src/tuw_nav2/tmux/cave_gazebo.yml
tmuxinator start -p ./ws02/src/tuw_nav2/tmux/cave_stage_ns.yml
tmuxinator start -p ./ws02/src/tuw_nav2/tmux/cave_stage_three_robots_one_wnd.yml 
tmuxinator start -p ./ws02/src/tuw_nav2/tmux/straden_stage.yml
```

## view on all vehicles
```
ros2 launch tuw_nav2 rviz_launch.py config:=multi_robot
```

