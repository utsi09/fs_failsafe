import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sensors_yaml = os.path.join(
        get_package_share_directory("cdr_monitor"), "config", "sensors_awcl.yaml")

    return LaunchDescription([
        # 데드라인 상한(ms)과 min(fresh, TTC) 동적 조임 on/off
        DeclareLaunchArgument(
            "fresh_ms", default_value="850.0",
            description="Sensor freshness deadline upper bound in ms"),
        DeclareLaunchArgument(
            "ttc_dynamic", default_value="true",
            description="Tighten deadlines to min(fresh_ms, TTC) when true"),
        Node(
            package="cdr_monitor",
            executable="cdr_monitor_node",
            name="cdr_monitor_node",
            output="screen",
            parameters=[sensors_yaml, {
                "use_sim_time": True,
                "fresh_ms": ParameterValue(
                    LaunchConfiguration("fresh_ms"), value_type=float),
                "ttc_dynamic": ParameterValue(
                    LaunchConfiguration("ttc_dynamic"), value_type=bool),
            }],
        ),
        Node(
            package="cdr_monitor",
            executable="awcl_adapter.py",
            name="awcl_adapter",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ])
