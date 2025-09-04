# mobile_robot

## Robot bringup

```
ros2 launch my_robot_bringup bringup.launch.py
```

## Realsense camera

```
ros2 launch realsense2_camera rs_launch.py enable_rgb:=true enable_depth:=true
```

## RTAB

```
ros2 launch rtabmap_launch rtabmap.launch.py     rgb_topic:=/camera/camera/color/image_raw     depth_topic:=/camera/camera/depth/image_rect_raw     camera_info_topic:=/camera/camera/color/camera_info     frame_id:=camera_link     approx_sync:=true     approx_sync_max_interval:=0.2     topic_queue_size:=30     sync_queue_size:=30 rtabmap_args:="--delete_db_on_start"
```

```
ros2 launch rtabmap_launch rtabmap.launch.py     rgb_topic:=/camera_rgb    depth_topic:=/camera_depth     camera_info_topic:=/camera_camera_info     frame_id:=Camera_OmniVision_OV9782_Color     approx_sync:=true     approx_sync_max_interval:=0.2     topic_queue_size:=30     sync_queue_size:=30 rtabmap_args:="--delete_db_on_start --Odom/Strategy=0"
```

## Localization Service call

```
ros2 service call /rtabmap/rtabmap/set_mode_localization std_srvs/srv/Empty "{}"
```

## Load map

```
ros2 service call /rtabmap/rtabmap/load_database rtabmap_msgs/srv/LoadDatabase "{ database_path: /home/rpi-1/.ros/rtabmap.db , clear: false}"
ros2 service call /rtabmap/rtabmap/load_database rtabmap_msgs/srv/LoadDatabase "{ database_path: /media/robotics-pc-20/New Volume/Viswa/RTAB MAP/rtabmap.db , clear: false}"

````

## Joy stick Teleop

```
ros2 launch mobilebot_controller joystick_teleop.launch.py 
```

## Motor Controller

```
ros2 run mobilebot_firmware joy_controller.py 
```

