#!/bin/bash
# Depth AND colour. The vendor's launch has enable_color:=false, so without
# this there is no /camera/color/image_raw at all and the cloud is XYZ only.
#
# MEASURED CONSTRAINT (2026-09-21): with colour on, depth and colour must run
# at the SAME fps and only 30 resolves. 6/6, 15/15 and every mixed pair
# (depth 6 + colour 30, colour 6 + depth 30) fail with "Failed to resolve the
# request". That is why this cannot be the gentle 6 fps of
# start_realsense_slow.sh.
#
# RISK: 30 fps depth is what killed the Jetson's xHCI controller before
# ("HC died", whole USB bus gone until a reboot). It was healthy when this was
# written (all six /dev/video* present). If the camera starts misbehaving, go
# back to start_realsense_slow.sh, which is depth-only at 6 fps.
#
# Colour is 424x240 because there is no compressed transport here: raw
# 640x480 at 30 is ~27 MB/s, hopeless over wifi. 424x240 is ~4.6 MB/s.
#
# Stop the depth-only service first: sudo systemctl stop realsense_ros2
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(cd "$(dirname "$0")" && pwd)/cyclone_dds.xml
source /opt/ros/foxy/setup.bash
source /home/ysc/lite_cog_ros2/driver/realsense_ws/install/setup.bash
exec ros2 launch realsense2_camera dr_camera_launch.py \
  depth_fps:=30.0 \
  enable_color:=true color_width:=424 color_height:=240 color_fps:=30.0
