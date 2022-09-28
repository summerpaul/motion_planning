run rviz

```
roslaunch motion_planning_ros rviz.launch
```

test grid_map

```
roslaunch motion_planning_ros gridmap_test.launch
```

test astar with grid map

```
roslaunch motion_planning_ros astar_search_grid_map_test.launch
```

test esdf_map

```
roslaunch motion_planning_ros esdfmap_test.launch
```

test polynomial_traj

```
roslaunch motion_planning_ros polynomial_traj_test.launch
```

reference

```
https://github.com/tuw-robotics/tuw_multi_robot
```

```
https://github.com/amslabtech/dwa_planner
```


生成complie_commands.json

`catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1 `

添加complie_commands.json

`"compileCommands": "${workspaceFolder}/build/compile_commands.json"`
