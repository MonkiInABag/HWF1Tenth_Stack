import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gap_share = get_package_share_directory("gap_follower")
    stack_share = get_package_share_directory("f1tenth_stack")
    planner_share = get_package_share_directory("global_planner")

    workspace_root = "/home/ethan/ros2_ws/src/HWF1Tenth_Stack"
    default_posegraph = os.path.join(workspace_root, "test_map")

    fake_vehicle_params = os.path.join(gap_share, "sim", "test_map_fake_vehicle_sim.yaml")
    path_follower_params = os.path.join(gap_share, "config", "gap_follower_params.yaml")
    slam_params = os.path.join(
        stack_share,
        "config",
        "tiny_mapper_params_localization.yaml",
    )
    global_planner_launch = os.path.join(
        planner_share,
        "launch",
        "global_planner_launch.py",
    )

    posegraph_arg = DeclareLaunchArgument(
        "posegraph",
        default_value=default_posegraph,
        description="Saved slam_toolbox posegraph base path without .posegraph/.data.",
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params,
            {
                # In this sim, slam_toolbox loads/publishes test_map while the
                # fake vehicle owns the stable map->odom TF.
                "transform_publish_period": 0.0,
            },
        ],
    )

    load_posegraph = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/slam_toolbox/deserialize_map",
                    "slam_toolbox/srv/DeserializePoseGraph",
                    [
                        "{filename: '",
                        LaunchConfiguration("posegraph"),
                        "', match_type: 1}",
                    ],
                ],
                output="screen",
            )
        ],
    )

    fake_vehicle = Node(
        package="gap_follower",
        executable="fake_vehicle_sim",
        name="fake_vehicle_sim",
        output="screen",
        parameters=[fake_vehicle_params],
    )

    global_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(global_planner_launch)
    )

    path_follower = Node(
        package="gap_follower",
        executable="path_follower_node",
        name="path_follower_node",
        output="screen",
        parameters=[
            path_follower_params,
            {
                # This launch is for validating map-frame global path tracking.
                # The raycast scan sees track walls very close, so local takeover
                # would otherwise stop the fake car immediately.
                "enable_local_planner": True,
                "local_steering_blend": 0.35,
                "wall_avoidance_gain": 0.18,
                "enable_boundary_safety": True,
                "boundary_safety_distance": 0.9,
                "boundary_safety_speed": 0.08,
                "boundary_steering_blend": 1.0,
                "boundary_escape_steering": 0.5,
                "force_closed_path": True,
                "reverse_path": True,
                # Keep the saved-map sim gentle so pure pursuit does not orbit a
                # single target point on tight or noisy centerline sections.
                "lookahead_distance": 1.2,
                "max_lookahead_target_distance": 5.0,
                "closest_search_back_points": 5,
                "closest_search_ahead_points": 120,
                "enable_path_error_correction": True,
                "heading_error_gain": 0.25,
                "crosstrack_error_gain": 0.35,
                "crosstrack_softening_speed": 0.45,
                "max_speed": 0.50,
                "min_speed": 0.22,
                "local_planner_speed": 0.18,
                "steering_smoothing": 0.55,
                "avoid_distance": 0.7,
                "blocker_distance": 0.18,
                "stop_on_blocker": False,
                "depth_threshold": 0.55,
                "front_fov_degrees": 22.0,
            },
        ],
    )

    return LaunchDescription(
        [
            posegraph_arg,
            slam_toolbox,
            load_posegraph,
            fake_vehicle,
            global_planner,
            path_follower,
        ]
    )
