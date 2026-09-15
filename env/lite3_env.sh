#!/bin/bash
# Source this before running anything that uses lite3.py.
_env_dir=$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)
export CYCLONEDDS_URI=file://$_env_dir/cyclone_dds.xml
# Make `from robot.lite3 import Lite3` work from anywhere.
export PYTHONPATH="$(dirname "$_env_dir")${PYTHONPATH:+:$PYTHONPATH}"
#   source ~/robot/env/lite3_env.sh && python3 my_behaviour.py
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
