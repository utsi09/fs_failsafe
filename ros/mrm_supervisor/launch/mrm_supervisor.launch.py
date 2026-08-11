from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "map_path", default_value="/home/taewook/awcl_maps/Town07/lanelet2_map.osm"),
        DeclareLaunchArgument("baseline", default_value="false"),
        DeclareLaunchArgument("demo_no_shoulder", default_value="false"),
        Node(
            package="mrm_supervisor",
            executable="supervisor",
            name="mrm_supervisor",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "map_path": LaunchConfiguration("map_path"),
                "baseline": LaunchConfiguration("baseline"),
                "demo_no_shoulder": LaunchConfiguration("demo_no_shoulder"),
            }],
        ),
    ])
