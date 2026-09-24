"""nav - the mapless Nav2 stack: start/stop it, read its costmap, goto().
Mixed into Lite3, so these are bot.nav_start(), bot.goto(1.5) etc.
"""
import math
import os
import signal
import subprocess
import time

from .protocol import Lite3Error

# --- nav2 ------------------------------------------------------------------
# Launched by env/start_nav2_mapless.sh.
# Nav2's children do not carry the launch file's name, so killing by name
# either misses them or, if you widen the pattern, reaches outside the stack.
# static_transform_publisher in particular is used by transfer and realsense
# too - pkill'ing it takes THOSE services down with it, because ros2 launch
# tears down a whole unit when one of its children dies. So the stack is
# launched via setsid into its own process group and killed by that group.
NAV_PGID_FILE = '/tmp/lite3_nav2.pgid'
NAV_MARKERS = ('dr_nav2_mapless', 'bt_navigator', 'planner_server',
               'controller_server')
# Never kill a group containing one of these - they belong to the services.
# Match executables, not words: the nav2 launch line itself contains
# "launch_realsense:=false", and a bare 'realsense' here made the guard
# exclude the very group it was meant to kill.
NAV_NEVER = ('jetson2motion', 'transfer_ros2', 'start_transfer',
             'realsense2_camera', 'realsense_ros2', 'voa_composition',
             'voa_ros2')

LETHAL = 99


def _repo(*parts):
    """A path inside this repo, wherever it happens to be checked out.

    Beats hardcoding ~/... : the scripts in env/ move with the code.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(os.path.dirname(here), *parts)


class Nav:
    """Needs self._node.grid, pose, turn() and halt(), all from Lite3."""

    @property
    def nav_running(self):
        return subprocess.run(['pgrep', '-f', 'bt_navigator'],
                              capture_output=True).returncode == 0

    @staticmethod
    def _nav_groups():
        """Process groups that belong to a nav2 stack and nothing else.

        Returns {pgid: [command lines]}. A group holding any NAV_NEVER process
        is excluded - that is the guard that stops a cleanup from taking
        transfer_ros2 or realsense_ros2 down with it.
        """
        out = subprocess.run(['ps', '-eo', 'pgid=,args='],
                             capture_output=True, text=True).stdout
        groups = {}
        for line in out.splitlines():
            line = line.strip()
            if not line:
                continue
            pgid, _, args = line.partition(' ')
            if not pgid.isdigit():
                continue
            # Drop launch arguments (foo:=bar) before matching, so a value
            # like launch_realsense:=false cannot look like a realsense node.
            args = ' '.join(t for t in args.split() if ':=' not in t)
            groups.setdefault(int(pgid), []).append(args)
        mine = os.getpgid(0)
        return {g: cmds for g, cmds in groups.items()
                if g != mine
                and any(m in c for c in cmds for m in NAV_MARKERS)
                and not any(n in c for c in cmds for n in NAV_NEVER)}

    def nav_stop(self, timeout=8.0):
        """Stop the nav2 stack by process group. Returns the number killed."""
        killed = 0
        for pgid in self._nav_groups():
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                try:
                    os.killpg(pgid, sig)
                except ProcessLookupError:
                    break
                end = time.time() + timeout / 3.0
                while time.time() < end:
                    try:
                        os.killpg(pgid, 0)
                    except ProcessLookupError:
                        break
                    time.sleep(0.2)
                else:
                    continue
                break
            killed += 1
        if os.path.exists(NAV_PGID_FILE):
            os.remove(NAV_PGID_FILE)
        self._node.grid = None
        time.sleep(1.0)
        return killed

    def nav_start(self, timeout=40.0):
        """Launch mapless Nav2. Refuses unless the robot is standing and its
        pose has settled - a costmap built across the stand-up pose jump shows
        a clear path straight into a real obstacle."""
        self._require_standing()
        if not self.pose_settled():
            raise Lite3Error(
                'pose is still moving; refusing to build a costmap around it. '
                'Wait for the robot to settle after standing.')
        self.nav_stop()
        proc = subprocess.Popen(
            ['setsid', _repo('env', 'start_nav2_mapless.sh')],
            stdout=open('/tmp/nav2.log', 'w'), stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL)
        # setsid makes the child a session and process-group leader, so its
        # pid is the pgid of the whole stack.
        with open(NAV_PGID_FILE, 'w') as f:
            f.write(str(proc.pid))
        if not self._wait(lambda: self.nav_running, timeout):
            raise Lite3Error('nav2 did not come up; see /tmp/nav2.log')
        if not self._wait(lambda: self._node.grid is not None, timeout):
            raise Lite3Error('nav2 is up but published no costmap; see /tmp/nav2.log')
        groups = self._nav_groups()
        if len(groups) > 1:
            raise Lite3Error(
                '%d nav2 process groups are running - an earlier stack '
                'survived. Call nav_stop() and retry.' % len(groups))
        return True

    def cost_at(self, x, y):
        """Global costmap cost at an odom-frame point. None if off the map."""
        g = self._node.grid
        if g is None:
            raise Lite3Error('no global costmap - is nav2 running?')
        i = g.info
        cx = int((x - i.origin.position.x) / i.resolution)
        cy = int((y - i.origin.position.y) / i.resolution)
        if not (0 <= cx < i.width and 0 <= cy < i.height):
            return None
        return g.data[cy * i.width + cx]

    def cost_ahead(self, out_to=3.5, step=0.25, settle=0):
        """[(distance, cost), ...] along the robot's heading.

        settle: re-read until two profiles a second apart agree, giving up
        after this many tries (0 = read once, the old behaviour).

        USE IT AFTER nav_start(). The costmap starts empty, and because
        `track_unknown_space` is False every unobserved cell reads FREE (0) -
        so an immediate profile says "clear" all the way out, you pick the
        furthest cell, and goto() then refuses that same cell as LETHAL once
        real observations land. The giveaway is cost 0 at distance 0, the
        robot's own cell, which is never really free on a populated map.
        """
        def profile():
            x, y, yaw = self.wait_pose()
            return [(d, self.cost_at(x + d * math.cos(yaw),
                                     y + d * math.sin(yaw)))
                    for d in [i * step for i in range(int(out_to / step) + 1)]]

        prev = None
        for _ in range(settle):
            prof = profile()
            if prof == prev:
                return prof
            prev = prof
            time.sleep(1.0)
        return prev if prev is not None else profile()

    def goto(self, forward, heading_deg=None, timeout=60.0, check=True):
        """Navigate `forward` metres ahead, routing around obstacles.

        Nav2 only gets him to the SPOT (yaw_goal_tolerance is 3.14 in
        lite_nav2_mapless.yaml): making Nav2 rotate him at the goal had the
        legged base stepping and drifting round it instead of stopping. So by
        default he stops facing however he arrived.

        heading_deg, if given, is the final heading relative to the heading
        at the call - positive is left, as turn_deg(). It is done after
        arrival with turn(), which is odometry-accurate.

        Returns the action status: 4 = SUCCEEDED, 6 = ABORTED.
        Checks the goal cell first - NavFn cannot plan into a LETHAL cell and
        the goal tolerance will not escape one.

        For an in-place turn use turn_deg() - a goal at the robot's own
        position gives the planner nothing to plan and aborts.
        """
        from nav2_msgs.action import NavigateToPose
        from geometry_msgs.msg import PoseStamped
        from rclpy.action import ActionClient

        self._require_standing()
        self._require_battery()
        if not self.nav_running:
            raise Lite3Error('nav2 is not running - call nav_start() first')
        if abs(forward) < 0.2:
            raise Lite3Error(
                'goal is %.2f m away - too close for the planner to produce a '
                'path, it will abort. Use turn_deg() for rotation in place.'
                % abs(forward))
        x, y, yaw = self.wait_pose()
        gx, gy = x + forward * math.cos(yaw), y + forward * math.sin(yaw)
        gyaw = yaw + math.radians(heading_deg or 0.0)

        if check:
            c = self.cost_at(gx, gy)
            if c is None:
                raise Lite3Error('goal (%.2f, %.2f) is outside the costmap' % (gx, gy))
            if c >= LETHAL:
                raise Lite3Error(
                    'goal cell cost %d is LETHAL - NavFn cannot plan into it '
                    'and would abort with status 6. Pick a goal from '
                    'cost_ahead(), not from scan(): the costmap knows about '
                    'obstacles the camera cannot currently see.' % c)

        client = ActionClient(self._node, NavigateToPose, 'navigate_to_pose')
        self._clients.append(client)
        if not client.wait_for_server(timeout_sec=10.0):
            raise Lite3Error('navigate_to_pose action server not available')

        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = gx, gy
        goal.pose.orientation.z = math.sin(gyaw / 2.0)
        goal.pose.orientation.w = math.cos(gyaw / 2.0)
        msg = NavigateToPose.Goal()
        msg.pose = goal

        fut = client.send_goal_async(msg)
        if not self._wait(fut.done, 10.0):
            raise Lite3Error('no response to the goal')
        handle = fut.result()
        if handle is None or not handle.accepted:
            raise Lite3Error('goal rejected')
        # Published so estop() can cancel the goal even when the SIGINT
        # handler fires somewhere else entirely.
        self._goal_handle = handle
        result_fut = handle.get_result_async()
        try:
            if not self._wait(result_fut.done, timeout):
                handle.cancel_goal_async()
                self.halt()
                raise Lite3Error('goal timed out after %.0fs; cancelled' % timeout)
        except KeyboardInterrupt:
            # estop() has already run from the signal handler; just propagate.
            raise
        finally:
            self._goal_handle = None
        status = result_fut.result().status
        if status == 4 and heading_deg is not None:
            err = gyaw - self.wait_pose()[2]
            err = math.atan2(math.sin(err), math.cos(err))
            if abs(err) > math.radians(5):
                self.turn(err)
        return status
