# Concert Nav2 Real Robot Bringup

This package is the real-robot Nav2 stack for Concert. It assumes the real
XBot2/omnisteering stack is already running and that the robot exposes:

- `/omnisteering/cmd_vel` as `geometry_msgs/msg/Twist`
- `/base_link/odom` as odometry
- `/scan` as the fused 2D laser scan
- TF frames `map`, `odom`, and `base_link`

The command chain is:

```text
Nav2 controller/recoveries -> cmd_vel_nav
cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
cmd_vel_smoothed -> collision_monitor -> /omnisteering/cmd_vel
```

Only `collision_monitor` should publish to `/omnisteering/cmd_vel`.

## 0. Forest Grow And Source

From the Forest workspace containing this package:

```bash
cd ~/data/forest_ws
forest grow concert_nav2 --no-deps --force-reconfigure -j 4
source install/setup.bash
export USE_SIM_TIME=false
```

If the Forest project name in your workspace is different, replace
`concert_nav2` with that project name. For the larger Concert/XBot2 workspace,
the usual build command is:

```bash
cd /home/user/xbot2_ws
forest grow iit-concert-ros-pkg --no-deps --force-reconfigure -j 4
source install/setup.bash
export USE_SIM_TIME=false
```

## 1. Start The Real Robot Base

Terminal 1, EtherCAT master:

```bash
reset
ecat_master
```

Terminal 2, XBot2:

```bash
xbot2-core --hw ec_imp -V
```

Wait until XBot2 is fully running before enabling navigation.

## 2. Enable And Test Omnisteering

Terminal 3, source ROS and check XBot2:

```bash
source ~/data/forest_ws/install/setup.bash
export USE_SIM_TIME=false

ros2 service list | grep xbotcore
ros2 topic list | grep omnisteering
```

Enable omnisteering:

```bash
ros2 service call /xbotcore/omnisteering/switch std_srvs/srv/SetBool "{data: true}"
ros2 service call /xbotcore/omnisteering/state xbot_msgs/srv/PluginStatus "{}"
```

Low-speed manual tests:

```bash
ros2 topic pub -r 10 /omnisteering/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

```bash
ros2 topic pub -r 10 /omnisteering/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.05, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

```bash
ros2 topic pub -r 10 /omnisteering/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

Stop after every test:

```bash
ros2 topic pub --once /omnisteering/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Do not start Nav2 until forward, lateral, and yaw commands all move correctly.

## 3. Start Odometry

Terminal 4:

```bash
source ~/data/forest_ws/install/setup.bash
export USE_SIM_TIME=false
ros2 launch concert_odometry_ros2 concert_odometry.launch.py use_sim_time:=false
```

Check odometry and TF:

```bash
ros2 topic echo --once /base_link/odom
ros2 run tf2_ros tf2_echo odom base_link
```

Both must work before Nav2 starts.

## 4. Start LiDAR Fusion To `/scan`

Terminal 5:

```bash
source ~/data/forest_ws/install/setup.bash
export USE_SIM_TIME=false
ros2 launch concert_navigation master_lidar_conversion_fuse.launch.py
```

Check scan:

```bash
ros2 topic hz /scan
ros2 topic echo --once /scan
```

## 5. Navigation With A Saved Map

Terminal 6, start localization. Replace the map path with the map you actually
want to use:

```bash
source ~/data/forest_ws/install/setup.bash
export USE_SIM_TIME=false
ros2 launch concert_localization localization.launch.py map_file:=/absolute/path/to/map.yaml use_sim_time:=false
```

Check localization TF:

```bash
ros2 run tf2_ros tf2_echo map odom
```

Terminal 7, start Nav2:

```bash
source ~/data/forest_ws/install/setup.bash
export USE_SIM_TIME=false
ros2 launch concert_navigation path_planner.launch.py use_sim_time:=false
```

Check Nav2 command topics:

```bash
ros2 topic list | grep -E "cmd_vel_nav|cmd_vel_smoothed|collision_monitor|omnisteering"
ros2 topic info /cmd_vel_nav
ros2 topic info /cmd_vel_smoothed
ros2 topic info /omnisteering/cmd_vel
```

Expected behavior:

- `controller_server` and `recoveries_server` publish toward `cmd_vel_nav`
- `velocity_smoother` publishes `cmd_vel_smoothed`
- `collision_monitor` publishes `/omnisteering/cmd_vel`

## 6. Send A Goal

RViz is the safest normal workflow:

```bash
rviz2 --ros-args -p use_sim_time:=false
```

In RViz:

1. Set fixed frame to `map`.
2. Use `2D Pose Estimate` if AMCL needs the initial pose.
3. Use `Nav2 Goal` to send a nearby goal first.

For a small CLI smoke test:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

Start with a short straight goal in open space. Then test lateral goals and
rotations.

## 7. Mapping / SLAM Mode

For mapping, start odometry and `/scan` first, then:

```bash
source ~/data/forest_ws/install/setup.bash
export USE_SIM_TIME=false
ros2 launch concert_navigation master_mapping_slam_saver.launch.py
```

Save a map:

```bash
ros2 run nav2_map_server map_saver_cli -f maps/myworld
```

The current planner config uses `allow_unknown: false`, which is safer for
saved-map navigation. If you want live SLAM navigation through unknown space,
set `allow_unknown: true` in:

```text
concert_navigation/config/planner_server.yaml
```

## 8. Stop Safely

Preferred stop order:

1. Cancel the Nav2 goal in RViz or stop the Nav2 launch.
2. Publish one zero command:

```bash
ros2 topic pub --once /omnisteering/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

3. Stop localization, lidar, odometry, then XBot2/EtherCAT.

Always keep the physical emergency stop available during first tests.

## Troubleshooting

If the robot does not move:

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel_smoothed
ros2 topic echo /omnisteering/cmd_vel
```

If `/cmd_vel_nav` has commands but `/cmd_vel_smoothed` does not, check
`velocity_smoother`. If `/cmd_vel_smoothed` has commands but
`/omnisteering/cmd_vel` does not, check `collision_monitor`.

If Nav2 cannot transform:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

If the local costmap is empty:

```bash
ros2 topic hz /scan
ros2 topic echo --once /local_costmap/published_footprint
```

If the robot tries to rotate like a differential drive, check that:

```bash
grep -n "motion_model" concert_navigation/config/controller.yaml
grep -n "max_velocity" concert_navigation/config/velocity_smoother.yaml
```

Expected values are `motion_model: "Omni"` and nonzero Y velocity limits.
