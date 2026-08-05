import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_dir = get_package_share_directory('cdr_monitor')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'vehicle.urdf.xml')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    sensors_yaml = os.path.join(pkg_dir, 'config', 'sensors.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # 데드라인 상한(ms)과 min(fresh, TTC) 동적 조임 on/off
    fresh_ms_arg = DeclareLaunchArgument(
        'fresh_ms',
        default_value='850.0',
        description='Sensor freshness deadline upper bound in ms'
    )
    ttc_dynamic_arg = DeclareLaunchArgument(
        'ttc_dynamic',
        default_value='true',
        description='Tighten deadlines to min(fresh_ms, TTC) when true'
    )

    fusion = get_package_share_directory('hesai_zed_fusion')
    fusion_launch = os.path.join(fusion,'launch','fusion.launch.py')
    included_fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fusion_launch),
        # launch_arguments=().items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        fresh_ms_arg,
        ttc_dynamic_arg,
        included_fusion,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': use_sim_time
            }],
        ),
        Node(
            package='cdr_monitor',
            executable='cdr_monitor_node',
            parameters=[sensors_yaml, {
                'use_sim_time': use_sim_time,
                'fresh_ms': ParameterValue(
                    LaunchConfiguration('fresh_ms'), value_type=float),
                'ttc_dynamic': ParameterValue(
                    LaunchConfiguration('ttc_dynamic'), value_type=bool),
            }],
            output='screen',
        ),
    ])
