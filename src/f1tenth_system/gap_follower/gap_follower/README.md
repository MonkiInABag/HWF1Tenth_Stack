This is used to run the path follower code. These are what need to be running before the path follower.


ros2 launch f1tenth_stack bringup_launch.py

ros2 launch f1tenth_stack localization_mode_slam_launch.py map_config:=tiny

ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph "{filename: '/home/hwf1tenth/szymon_ws/HWF1Tenth_Stack/quick_map_10', match_type: 1}"

ros2 launch global_planner global_planner_launch.py

