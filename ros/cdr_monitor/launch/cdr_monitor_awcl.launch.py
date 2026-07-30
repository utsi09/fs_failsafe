import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sensors_yaml = os.path.join(
        get_package_share_directory("cdr_monitor"), "config", "sensors_awcl.yaml")

    return LaunchDescription([
        Node(
            package="cdr_monitor",
            executable="cdr_monitor_node",
            name="cdr_monitor_node",
            output="screen",
            parameters=[sensors_yaml, {"use_sim_time": True}],
        ),
        Node(
            package="cdr_monitor",
            executable="awcl_adapter.py",
            name="awcl_adapter",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ])
