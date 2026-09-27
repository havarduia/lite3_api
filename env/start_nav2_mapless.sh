#!/bin/bash
# Launch the mapless Nav2 stack. Normally called by lite3.Lite3.nav_start(),
# which enforces stand-first ordering and cleans up orphaned nodes.
source "$(dirname "${BASH_SOURCE[0]:-$0}")/lite3_env.sh"
# The sonars as Range for the costmap range_layer. Same process group as the
# launch below, so nav_stop() kills it with the stack.
rm -f /tmp/sonar_range.ready
python3 -m robot.sonar_range &
# Wait for its first message (python takes a couple of seconds to start here),
# or the range layer warns "No range readings received". Give up after 10 s
# and launch anyway, so a dead sonar still shows up as that warning.
for _ in $(seq 100); do [ -e /tmp/sonar_range.ready ] && break; sleep 0.1; done
exec ros2 launch dr_nav2_mapless dr_nav2_mapless.launch.py launch_realsense:=false use_rviz:=false
