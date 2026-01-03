# Commands

## Realsense cam

```
ros2 launch realsense2_camera rs_launch.py enable_rgb:=true enable_depth:=true
```

## Rtab launch

```
ros2 launch rtabmap_launch rtabmap.launch.py     rgb_topic:=/camera/camera/color/image_raw     depth_topic:=/camera/camera/depth/image_rect_raw     camera_info_topic:=/camera/camera/color/camera_info     frame_id:=camera_link     approx_sync:=true     subscribe_rgb:=true     subscribe_depth:=true     use_sim_time:=false rtabmap_args:="--delete_db_on_start" rtab_rviz:=true approx_sync_max_interval:=0.02
```


## Open isaac sim in streaming

```
isaacsim isaacsim.exp.full.streaming --no-window --streaming
```

## training

navigate to folder with isaaclab.sh file

```
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --headless --task=Isaac-Ant-v0 --video --max_iterations=20
```

## run python file

navigate to folder with isaaclab.sh file

```
./isaaclab.sh -p source/learning/learning/simulation/bipad.py --livestream=2
```

## Rtab

```
ros2 launch rtabmap_launch rtabmap.launch.py     rgb_topic:=/camera_rgb    depth_topic:=/camera_depth     camera_info_topic:=/camera_camera_info     frame_id:=Camera_OmniVision_OV9782_Color     approx_sync:=true     approx_sync_max_interval:=0.2     topic_queue_size:=30     sync_queue_size:=30 rtabmap_args:="--delete_db_on_start --Odom/Strategy=0"
```


## Ros Drone

```ros2 run joy joy_node```

```ros2 run mapReader joy_drone_control```


## Custom Depth RTAB

```
ros2 launch rtabmap_launch rtabmap.launch.py     rgb_topic:=/camera_rgb    depth_topic:=/custom_depth     camera_info_topic:=/camera_camera_info     frame_id:=Camera_OmniVision_OV9782_Color     approx_sync:=true     approx_sync_max_interval:=0.5     topic_queue_size:=30     sync_queue_size:=30 rtabmap_args:="--delete_db_on_start --Odom/Strategy=0"
```


ros2 launch depth_to_scan.launch.py

python3 src/depth_package/depth_package/rgb_to_depth.py