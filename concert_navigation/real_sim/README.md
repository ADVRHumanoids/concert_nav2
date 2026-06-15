# Real Nav2 Profile Adapted For Simulation

This folder contains the Nav2 configuration, behavior trees, and launch files
copied from the real-robot `concert_nav2_real` implementation and adapted only
where the simulator integration differs.

Topic and frame adaptations:

- Real lidar clouds: `/VLP16_lidar_front/velodyne_points`,
  `/VLP16_lidar_back/velodyne_points`
- Sim lidar clouds: `/VLP16_lidar_front/points`,
  `/VLP16_lidar_back/points`
- Real lidar conversion frames: `VLP16_lidar_front_base_link`,
  `VLP16_lidar_back_base_link`
- Sim lidar conversion frames: `VLP16_lidar_front`, `VLP16_lidar_back`
- Real Nav2 base frame: `base_link`
- Sim Nav2 base frame: `base_link_projected`

Topics kept identical because they are already shared by the sim stack:

- `/scan`
- `/base_link/odom`
- `/omnisteering/cmd_vel`
- `/cmd_vel_nav`
- `/cmd_vel_smoothed`

The public launch files are installed in the normal `launch/` directory with
`*_real_sim.launch.py` names. They are direct adapted launchers, not wrapper
layers around the original simulated launch files.

Launch examples after rebuilding/installing the package:

```bash
ros2 launch concert_navigation master_lidar_conversion_fuse_real_sim.launch.py
ros2 launch concert_navigation path_planner_real_sim.launch.py
ros2 launch concert_navigation path_planner_tuned_real_sim.launch.py
```
