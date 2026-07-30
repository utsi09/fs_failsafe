from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hesai_zed_fusion',
            executable='fusion_node.py',
            name='hesai_zed_fusion',
            parameters=[{
                'model_path': '/home/taewook/fs_ws/src/yolo26x-seg.engine',
                'image_topic': '/zed/zed_node/rgb/color/rect/image/compressed',
                'lidar_topic': '/lidar_points',
                'camera_info_topic': '/zed/zed_node/rgb/color/rect/image/camera_info',
                'conf': 0.3,
                'mask_scale': 0.8,
                'lidar_frame': 'hesai_lidar',
                'base_frame': 'base_link',
                'camera_frame': '',
            }],
            output='screen',
        )
    ])
