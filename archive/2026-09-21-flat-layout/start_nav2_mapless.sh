#!/bin/bash
# Launch the mapless Nav2 stack. Normally called by lite3.Lite3.nav_start(),
# which enforces stand-first ordering and cleans up orphaned nodes.
source ~/lite3_env.sh
exec ros2 launch dr_nav2_mapless dr_nav2_mapless.launch.py launch_realsense:=false use_rviz:=false
