from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depth_to_scan',
            remappings=[
                ('depth', '/camera/depth/image'),
                # ('camera_info', '/camera/depth/camera_info'),
                ('scan', '/scan'),
            ],
            parameters=[{
                # 'image_transport': 'raw',
                # 'approx_sync': True,
                # 'queue_size': 10,
                # 'range_min': 0.3,
                # 'range_max': 5.0,
                'range_max': 15.0,
                'scan_height': 2,
                'output_frame': 'base_link',
            }]
        )
    ])
