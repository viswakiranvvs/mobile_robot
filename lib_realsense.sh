#!/bin/bash
set -e

# ========= SETTINGS =========
LIBREAL_VER_BRANCH="master"   # or use r/256-related version
WORKSPACE=~/ros2_ws
CORES=$(nproc)
# ============================

echo "=== Updating system ==="
sudo apt update

echo "=== Install build tools ==="
sudo apt install -y git cmake build-essential libssl-dev

echo "=== Remove old librealsense (if any) ==="
sudo rm -rf /usr/local/lib/cmake/realsense2
sudo rm -rf /usr/local/include/librealsense2
sudo rm -f /usr/local/lib/librealsense2*

cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
# Optional: checkout specific tag, e.g., v2.56.4
git checkout v2.56.4
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
make -j$(nproc)
sudo make install


echo "=== Build realsense2 ROS2 wrapper ==="
cd ${WORKSPACE}
# Fetch latest
cd src/realsense-ros || { echo "realsense-ros source not found!"; exit 1; }
git checkout r/256   # or r/255 depending on SDK
cd ${WORKSPACE}
colcon build --packages-select realsense2_camera realsense2_description realsense2_camera_msgs \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -Drealsense2_DIR=/usr/local/lib/cmake/realsense2
colcon build --packages-select realsense2_camera realsense2_description realsense2_camera_msgs --symlink-install

echo "=== Done! Source workspace now: ==="
echo "source ${WORKSPACE}/install/setup.bash"

# source ~/ros2_ws/install/setup.bash
# ros2 launch realsense2_camera rs_launch.py
# rs-enumerate-devices
ssh rpi-1@172.18.141.5

ros2 launch realsense2_camera rs_launch.py enable_rgb:=true enable_depth:=true

ros2 run rtabmap_viz rtabmap_viz

ros2 launch rtabmap_launch rtabmap.launch.py     rgb_topic:=/camera/camera/color/image_raw     depth_topic:=/camera/camera/depth/image_rect_raw     camera_info_topic:=/camera/camera/color/camera_info     frame_id:=camera_link     approx_sync:=true     subscribe_rgb:=true     subscribe_depth:=true     use_sim_time:=false rtabmap_args:="--delete_db_on_start" rtab_rviz:=true approx_sync_max_interval:=0.02 namespace:=rtabmap_pi

ros2 launch my_robot_bringup bringup.launch.py