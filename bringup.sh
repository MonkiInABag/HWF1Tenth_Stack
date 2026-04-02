#!/bin/bash
cd ~/szymon_ws/HWF1Tenth_Stack
source install/setup.bash
echo "=== Starting Bringup ==="
ros2 launch f1tenth_stack bringup_launch.py
