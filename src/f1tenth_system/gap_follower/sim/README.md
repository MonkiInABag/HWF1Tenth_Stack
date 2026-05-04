# Fake Track Simulation

This folder contains the lightweight simulation setup for testing the path
follower without the physical F1TENTH car.

It starts:

- `nav2_map_server` for `/map`
- `global_planner_map_node` for `/global_centerline`
- `fake_vehicle_sim` for `/odometry/filtered`, `/odom`, `/scan`, and TF
- `path_follower_node` for `/drive`

Build and run from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch gap_follower fake_track_sim.launch.py
```

Useful checks:

```bash
ros2 topic echo /drive
ros2 topic hz /odometry/filtered
ros2 topic hz /scan
ros2 topic echo /global_centerline --once
```

For RViz, add displays for `/map`, `/global_centerline`, `/scan`,
`/odometry/filtered`, `/fake_vehicle_marker`, and TF.

To test blocker behaviour, set `enable_front_blocker: true` in
`fake_vehicle_sim.yaml`.
