from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to your launch files
    rtabmap_launch_dir = os.path.join(
        get_package_share_directory('rtabmap_launch'))
    realsense_launch_dir = os.path.join(
        get_package_share_directory('realsense2_camera'))
    mobilebot_controller_dir = os.path.join(
        get_package_share_directory('mobilebot_controller'))

    return LaunchDescription([

        # 1. Realsense camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(realsense_launch_dir, 'launch/rs_launch.py')),
            launch_arguments={
                'enable_rgb': 'true',
                'enable_depth': 'true'
            }.items()
        ),
        
        # 2. Joy node
        Node(
            package='joy',
            executable='joy_node',
            # output='screen'
        ),

        # 3. Your custom firmware joystick controller
        Node(
            package='mobilebot_firmware',
            executable='joy_controller.py',
            # output='screen'
        ),

        # 4. RTAB-Map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(rtabmap_launch_dir, 'launch/rtabmap.launch.py')),
            launch_arguments={
                'rgb_topic': '/camera/camera/color/image_raw',
                'depth_topic': '/camera/camera/depth/image_rect_raw',
                'camera_info_topic': '/camera/camera/color/camera_info',
                'frame_id': 'map',
                'approx_sync': 'true',
                'subscribe_rgb': 'true',
                'subscribe_depth': 'true',
                'use_sim_time': 'false',
                'rtabmap_args': '--delete_db_on_start',
                'rtab_rviz': 'false',
                'approx_sync_max_interval': '0.02'
            }.items()
        ),

        Node(
            package='mapCtrl',
            executable='mapCtrl',
            # output='screen'
        ),

        # 5. Teleop launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mobilebot_controller_dir, 'launch/joystick_teleop.launch.py'))
        ),
    ])
