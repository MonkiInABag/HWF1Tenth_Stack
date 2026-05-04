import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gap_share = get_package_share_directory("gap_follower")
    planner_share = get_package_share_directory("global_planner")

    default_map = os.path.join(planner_share, "maps", "Austin_map.yaml")
    fake_vehicle_params = os.path.join(gap_share, "sim", "fake_vehicle_sim.yaml")
    path_follower_params = os.path.join(gap_share, "config", "gap_follower_params.yaml")
    global_planner_launch = os.path.join(
        planner_share,
        "launch",
        "global_planner_launch.py",
    )

    map_file_arg = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Occupancy-grid map yaml used by nav2_map_server.",
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": LaunchConfiguration("map")}],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {
                "autostart": True,
                "node_names": ["map_server"],
            }
        ],
    )

    global_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(global_planner_launch)
    )

    fake_vehicle = Node(
        package="gap_follower",
        executable="fake_vehicle_sim",
        name="fake_vehicle_sim",
        output="screen",
        parameters=[fake_vehicle_params],
    )

    path_follower = Node(
        package="gap_follower",
        executable="path_follower_node",
        name="path_follower_node",
        output="screen",
        parameters=[path_follower_params],
    )

    return LaunchDescription(
        [
            map_file_arg,
            map_server,
            lifecycle_manager,
            global_planner,
            fake_vehicle,
            path_follower,
        ]
    )
