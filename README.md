# software-frameworks

## Mini-Project 1

In order to view the solution see either `proj1_run.mp4` or `proj1_run_speed.mp4` or run the following code from `catkin_ws_project_1`
```
roslaunch jaco_on_table jaco_on_table_gazebo_controlled.launch load_grasp_fix := true
roslaunch jaco_on_table_moveit jaco_on_table_moveit.launch
roslaunch jaco_on_table_moveit jaco_on_table_rviz.launch
rosrun proj1 cube_spawn.py
rosurn proj1 move_cubes_to_bucket.py
``` 

## Final Project

In order to view the solution see either `proj2.mp4` or `proj2.mp4` or run the following code from `catkin_ws_project_2`
```
roslaunch proj2 run_all.launch
rosurn proj2 track_qr_codes.py
``` 
