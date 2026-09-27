#!/bin/bash
# Launch the mapless Nav2 stack. Normally called by lite3.Lite3.nav_start(),
# which enforces stand-first ordering and cleans up orphaned nodes.
source "$(dirname "${BASH_SOURCE[0]:-$0}")/lite3_env.sh"
# The sonars as Range for the costmap range_layer. Same process group as the
# launch below, so nav_stop() kills it with the stack.
python3 -m robot.sonar_range &
exec ros2 launch dr_nav2_mapless dr_nav2_mapless.launch.py launch_realsense:=false use_rviz:=false
