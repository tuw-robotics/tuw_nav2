# tuw_nav2 

## Command Hints

### teleop joy 
```
cd $WORKSPACE
ros2 launch teleop_twist_joy teleop-launch.py  config_filepath:=$WORKSPACE/src/tuw_nav2/config/teleop_twist_joy/f710.yaml
```

### slam
```
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$WORKSPACE/src/tuw_nav2/config/slam_toolbox/mapper_params_online_async.yaml
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
ros2 launch tuw_nav2 localization_launch.py map:=$WORKSPACE/install/tuw_nav2/share/tuw_nav2/config/maps/cave/map.yaml
ros2 launch tuw_nav2 navigation_launch.py
```