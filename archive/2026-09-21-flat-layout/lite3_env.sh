#!/bin/bash
# Source this before running anything that uses lite3.py.
export CYCLONEDDS_URI=file:///home/ysc/cyclone_dds.xml
#   source ~/lite3_env.sh && python3 my_behaviour.py
#
# All five workspaces are needed. In particular realsense_ws is required even
# when launching with launch_realsense:=false, because the launch description
# calls get_package_share_directory('realsense2_camera') at construction time,
# before the IfCondition is evaluated.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/foxy/setup.bash
source ~/lite_cog_ros2/driver/realsense_ws/install/setup.bash
source ~/lite_cog_ros2/navigation2-foxy/install/setup.bash
source ~/lite_cog_ros2/nav/install/setup.bash
source ~/lite_cog_ros2/transfer/install/setup.bash
