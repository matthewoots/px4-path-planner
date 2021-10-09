# PX4 Path Planner
Offboard/Onboard development for PX4 Autopilot with
1. **Autonomous navigation** with **Trajectory planning** 
2. **Added** cubic spline library from https://github.com/ttk592/spline/blob/master/src/spline.h (Includes first and second derivatives that can be use in sending to PX4, however currently only velocity is being used currently)
3. **Added** CSV parser to C++ (https://github.com/ben-strasser/fast-cpp-csv-parser) this is used for passing waypoints in a certain format to generate the trajectory
4. **Included** Eigen3 library (`Vector3d` and `MatrixXd`)
5. Using `v1.12.3` of PX4-Autopilot, run with `make px4_sitl gazebo HEADLESS=1`
6. Launch with `roslaunch px4_path_planner main.launch`

- `full_test_trajectory.ulg` has the full log for a test trajectory. This includes `takeoff`, `mission`, `home` and `landing`, which all uses the **spline** trajectory and it seems quite promising. To test `home` function, `wp_away_from_home.csv` is used.

## Notes
1. Example of a csv file format in text that can be found in `path/wp.csv`.
```csv
time,xpos,ypos,zpos
0,0,0,5
1,0.2,0.8,5
2,1,0.5,5
3,1.5,1.0,5
4,0.5,0.5,5
5,0,0.0,5
```
---
2. Do not use `~/` as home reference, as the `error` below will appear.
```bash
[trajectory.h] Trying to open ~/offb_ws/src/px4-path-planner/path/wp.csv 
[trajectory.h] File not present!
```
---
3. The `yaml` common settings are below, there are `@brief` of the definitions in some of the code so you can have a look at them.
```yaml
debug: true
control_points_interval: 0.1
unique_id_range: 100
trajectory_interval: 0.1
takeoff_height: 5.0
takeoff_velocity: 0.05
setpoint_raw_mode: true
wp_file_location: '/home/USER/offb_ws/src/px4-path-planner/path/wp.csv'
```
---
4. PX4 `master` branch does not allow offboard arming, which causes the drone to arm and disarm immediately, the code will keep forcing it and the request will be rejected. Hence use `v1.12.3` branch of `PX4-Autopilot`.
```git
HEAD detached at v1.12.3
```

## Current Progress
1. Tracking performance with spline provided, using `path/wp_test_tracking.csv`
```
You can find the ulog file inside the media folder, media/wp_test_tracking.ulg
```

|![](media/local_position_x.png) | ![](media/local_position_x.png) | 
|---|---|


|![](media/velocity.png) | ![](media/local_position_x.png) |
|---|---|

## References 
1. **Offboard Node Example** : https://docs.px4.io/master/en/ros/mavros_offboard.html
1. **Cubic Spline Library Reference** : https://kluge.in-chemnitz.de/opensource/spline/