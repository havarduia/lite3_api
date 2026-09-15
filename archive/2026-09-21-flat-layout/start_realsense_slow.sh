#!/bin/bash
# Depth-only at 6 fps. The Jetson's xHCI controller dies under the default
# 30 fps stream ("HC died"), taking the whole USB bus with it until a reboot.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/ysc/cyclone_dds.xml
source /opt/ros/foxy/setup.bash
source /home/ysc/lite_cog_ros2/driver/realsense_ws/install/setup.bash
exec ros2 launch realsense2_camera dr_camera_launch.py depth_fps:=6.0
