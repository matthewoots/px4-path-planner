# PX4 Path Planner
Offboard/Onboard development for PX4 Autopilot with
1. **Autonomous navigation** with **Trajectory planning** 
2. **Added-But-Not-Using** cubic spline library from https://github.com/ttk592/spline/blob/master/src/spline.h (Includes first and second derivatives that can be use in sending to PX4, however currently only velocity is being used currently) 
```
Cubic spline library is not used and is replaced by a Custom Bspline Trajectory Generation, which is able to constrain agent path within control points 
```
3. **Added** CSV parser to C++ (https://github.com/ben-strasser/fast-cpp-csv-parser) this is used for passing waypoints in a certain format to generate the trajectory

3. **Added** LBFGS++ (https://github.com/yixuan/LBFGSpp) using an unconstrained nonlinear optimization to compute a safe corridor and reciprocal avoidance of other agents

4. **Included** Eigen3 library (`Vector3d` and `MatrixXd`)

5. Using `v1.12.3` of PX4-Autopilot, run with `make px4_sitl gazebo HEADLESS=1`

6. Launch with `roslaunch px4_path_planner main.launch`

- `trajectory_representation.ulg` has the full log for a test trajectory. This includes `takeoff`, `mission`, `home` and `landing`, which all uses the **Bspline** trajectory.

## Setup
**Step 1** Setup workspace
```bash
# cd to <your_ws>/src
git clone https://github.com/matthewoots/px4-path-planner.git
catkin build # at <your_ws>
```
**Step 2** Add to `devel/setup.bash`
```bash
SRC_DIR=<PX4-Autopilot-directory>
BUILD_DIR=<PX4-Autopilot-directory>/build/px4_sitl_default

# setup Gazebo env and update package path
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:${BUILD_DIR}/build_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${SRC_DIR}/Tools/sitl_gazebo/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${BUILD_DIR}/build_gazebo

echo -e "GAZEBO_PLUGIN_PATH $GAZEBO_PLUGIN_PATH"
echo -e "GAZEBO_MODEL_PATH $GAZEBO_MODEL_PATH"
echo -e "LD_LIBRARY_PATH $LD_LIBRARY_PATH"

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$SRC_DIR:$SRC_DIR/Tools/sitl_gazebo
echo $ROS_PACKAGE_PATH
```
And then source the file again, this will give you access to PX4-Autopilot launch files and Gazebo interface. You can refer to `setup_bash_file` for a sample

**Step 3** Test launch
```bash
# To launch 1 agent
roslaunch px4_path_planner main_visualization.launch
# To launch 3 agents
roslaunch px4_path_planner multi_sitl.launch
```

## Notes
0. Using Bspline Generic Recursive Representation by `K. Qin,General matrix representations forb-splines, Vis. Comput., 16 (2000),pp. 177–186.`

1. Example of a csv file format in text that can be found in `path/wp0.csv`.
    - `mode` represents `0. bypass` or `1. bspline` or `2. bspline_avoid` or `3. bspline_avoid_opt`
```csv
mode,xpos,ypos,zpos
0,0,-3.0,1.3
1,3,-3.0,1.3
2,3,3.0,1.3
3,-3.0,3,1.3
```
**[Updated]** Trajectory generation has factored the `maximum velocity limits` into the calculation, hence `time` is taken out.

---
2. Do not use `~/` as home reference, as the `error` below will appear.
```bash
[trajectory.h] Trying to open ~/offb_ws/src/px4-path-planner/path/wp0.csv 
[trajectory.h] File not present!
```

---
3. The `yaml` common settings are below, there are `@brief` of the definitions in some of the code so you can have a look at them.
```yaml
# Command Parameters
setpoint_raw_mode: true

# Spline Parameters
spline_order: 5
control_points_division: 10
trajectory_pub_rate: 20

# Total number of Agents
unique_id_range: 20

# Agent Parameters
takeoff_height: 1.3
common_max_vel: 1.5
common_min_vel: 0.3
```

---
4. PX4 `master` branch does not allow offboard arming, which causes the drone to arm and disarm immediately, the code will keep forcing it and the request will be rejected. Hence use `v1.12.3` branch of `PX4-Autopilot`.
```git
HEAD detached at v1.12.3
```

---
5. Since this module would be used on lightweight platforms in GPS denied environments, we should add all the `# Other Addons` to the blacklist when launching `Mavros`
```yaml
plugin_blacklist:
# common
- safety_area
# extras
- image_pub
- vibration
- distance_sensor
- rangefinder
- wheel_odometry
# Other Addons
- landing_target
- px4flow
- vision_speed_estimate
- fake_gps
- global_position
- vfr_hud
- setpoint_attitude
- setpoint_accel
- setpoint_velocity
- mission
- rc_io
- param
- setpoint_trajectory
- gpsstatus
- debug_value
- adsb
- gps_rtk
- manual_control
- trajectory
- wind_estimation
- actuator_control
```

## Current Progress
0. Working for all tasks, and using `setpoint_raw` target of `position`, `velocity`, `acceleration` and `yaw`. `yaw` is mostly enabled during **mission** tasks only.

1. Tracking performance with spline provided, using `path/trajectory_representation.csv`
```
You can find the ulog file inside the media folder, media/trajectory_representation.ulg
```

|![](media/local_position_x.png) | ![](media/local_position_x.png) | 
|---|---|


|![](media/velocity.png) | ![](media/local_position_x.png) |
|---|---|

## References 
1. **Offboard Node Example** : https://docs.px4.io/master/en/ros/mavros_offboard.html
1. **Cubic Spline Library Reference - Not Used** : https://kluge.in-chemnitz.de/opensource/spline/