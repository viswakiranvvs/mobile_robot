# Commands

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

ransforms:
- header:
    stamp:
      sec: 1757326080
      nanosec: 315977963
    frame_id: map
  child_frame_id: drone_base_link
  transform:
    translation:
      x: -5.062188625335693
      y: -4.148741722106934
      z: 0.8592492938041687
    rotation:
      x: -4.829525458673308e-05
      y: -0.0004970230115062147
      z: 0.8515284975714373
      w: 0.5243080854415899
---
transforms:
- header:
    stamp:
      sec: 280
      nanosec: 544013325
    frame_id: world
  child_frame_id: Camera_OmniVision_OV9782_Color
  transform:
    translation:
      x: -5.074501037597656
      y: -4.077759265899658
      z: 0.9080643057823181
    rotation:
      x: 0.6877332925796509
      y: 0.16374509036540985
      z: -0.16325373947620392
      w: -0.688156008720398
---
transforms:
- header:
    stamp:
      sec: 280
      nanosec: 544013325
    frame_id: Camera_OmniVision_OV9782_Color
  child_frame_id: Camera_OmniVision_OV9782_Color_world
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.5
      y: -0.5
      z: 0.5
      w: 0.5
