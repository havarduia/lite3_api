#!/bin/bash
# The camera on the rebuilt stack: librealsense 2.58.4 (native V4L backend),
# the patched uvcvideo kernel module, and realsense-ros 4.58.3 from
# ~/realsense_ros4_ws. Depth, colour and the camera IMU - see camera.launch.py.
#
# Rollback to the vendor stack: point
# ~/lite_cog_ros2/system/scripts/depth_camera/realsense/start_realsense.sh back
# at its .orig copy. It needs librealsense 2.50 (RSUSB) reinstalled and the
# stock uvcvideo restored from ~/archive/backups/kmods-*.tgz to stream.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/foxy/setup.bash
source /home/ysc/realsense_ros4_ws/install/setup.bash
exec ros2 launch "$(cd "$(dirname "$0")" && pwd)/camera.launch.py"
