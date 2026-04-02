#!/bin/bash
cd ~/szymon_ws/HWF1Tenth_Stack
source install/setup.bash
echo "=== Starting SLAM Mapping ==="
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=src/f1tenth_system/f1tenth_stack/config/tiny_mapper_params_online_async.yaml
