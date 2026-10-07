"""nav - the mapless Nav2 stack: start/stop it, read its costmap, goto().
Mixed into Lite3, so these are bot.nav_start(), bot.goto(1.5) etc.
"""
import math
import os
import signal
import subprocess
import time

from .protocol import Lite3Error

# --- nav2 ---
# The stack runs in its own process group (setsid) and is killed by that
# group: killing by name misses its children, or reaches the
# static_transform_publisher the services use. README.md section 7.2.
NAV_MARKERS = ('dr_nav2_mapless', 'bt_navigator', 'planner_server',
               'controller_server', 'robot.sonar_range')
# Never kill a group holding one of these. Executables, not words: the
# launch line itself says launch_realsense:=false.
NAV_NEVER = ('jetson2motion', 'transfer_ros2', 'start_transfer',
             'realsense2_camera', 'realsense_ros2', 'voa_composition',
             'voa_ros2')

LETHAL = 99
SUCCEEDED, CANCELED, ABORTED = 4, 5, 6      # NavigateToPose result status, as goto() returns it
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # the scripts in env/ move with the code


def status_text(status):
    """A goto() status in words."""
    return {SUCCEEDED: 'arrived', CANCELED: 'cancelled',
            ABORTED: 'aborted - no path'}.get(status, 'status %s' % status)


def clamp(v, limit):
    """v held within +-limit. Anything not finite becomes 0: max/min pass a
    NaN through as +limit, which is full speed, not a stop."""
    return max(-limit, min(limit, v)) if math.isfinite(v) else 0.0


def dist(a, b):
    """Metres between two poses."""
    return math.hypot(b[0] - a[0], b[1] - a[1])


def wrap(angle):
    """An angle in radians brought into -pi..pi."""
    return math.atan2(math.sin(angle), math.cos(angle))


def free_ahead(profile):
    """Metres a cost_ahead() profile stays plannable from the robot outwards:
    the last distance before the first LETHAL or off-map cell.
    """
    far = 0.0
    for d, c in profile:
        if c is None or c >= LETHAL:
            break
        far = d
    return far


def goal_ahead(profile, margin=0.25, minimum=0.5):
    """A goto() distance `margin` short of the first cell goto() would refuse,
    or None when that is under `minimum`: xy_goal_tolerance is 0.25, so a
    nearer goal "arrives" at once."""
    goal = free_ahead(profile) - margin
    return goal if goal >= minimum else None


def _alive(pgid):
    """Is any process of that group still running?"""
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    # A dead leader stays on as a zombie until reaped; zombies do not count.
    ps = subprocess.run(['ps', '-eo', 'pgid=,stat='], capture_output=True, text=True)
    states = [f[1] for f in map(str.split, ps.stdout.splitlines()) if f[:1] == [str(pgid)]]
    return ps.returncode != 0 or any(not s.startswith('Z') for s in states)


def _kill_group(pgid, timeout, wait):
    """SIGINT, then SIGTERM, then SIGKILL, each given timeout/3 s to empty the group."""
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return
        if wait(lambda: not _alive(pgid), timeout / 3.0, 0.2):
            return


class Nav:
    """Needs self._node.grid, pose, turn() and halt(), all from Lite3."""

    @property
    def nav_running(self):
        return subprocess.run(['pgrep', '-f', 'bt_navigator'],
                              capture_output=True).returncode == 0

    @staticmethod
    def _nav_groups():
        """{pgid: [command lines]} of the groups that are a nav2 stack and hold no
        NAV_NEVER process.
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
            _kill_group(pgid, timeout, self._wait)
            killed += 1
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
        # setsid: a session and process group of its own, for nav_stop().
        with open('/tmp/nav2.log', 'w') as log:
            subprocess.Popen(['setsid', os.path.join(REPO, 'env', 'start_nav2_mapless.sh')],
                             stdout=log, stderr=subprocess.STDOUT,
                             stdin=subprocess.DEVNULL)
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
        """[(distance, cost), ...] along the robot's heading. settle: re-read until
        two profiles a second apart agree, at most this many times. Use it after
        nav_start(): the costmap starts empty and unseen cells read as free.
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
        """Navigate `forward` metres ahead, routing around obstacles. Returns the
        action status (SUCCEEDED, CANCELED or ABORTED).

        heading_deg: final heading relative to the one at the call (+ve left),
        done with turn() after arrival; Nav2 itself only gets him to the spot.
        Refuses a LETHAL goal cell, and a goal under 0.2 m (use turn_deg()).
        """
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

        handle = self.goal_send(gx, gy, gyaw)
        result_fut = self._goal_result
        try:
            # estop() from another thread drops the handle: Nav2 is being killed and
            # the result may never come.
            if not self._wait(lambda: result_fut.done() or self._goal_handle is None, timeout):
                handle.cancel_goal_async()
                self.halt()
                raise Lite3Error('goal timed out after %.0fs; cancelled' % timeout)
        finally:
            self._goal_handle = None
        status = result_fut.result().status if result_fut.done() else CANCELED
        if status != SUCCEEDED:
            self.halt()                     # whoever ended it, he ends stopped
        if status == SUCCEEDED and heading_deg is not None:
            err = wrap(gyaw - self.wait_pose()[2])
            if abs(err) > math.radians(5):
                self.turn(err)
        return status

    def goal_send(self, gx, gy, gyaw=0.0):
        """Hand Nav2 a goal at an odom-frame point and return once it is
        accepted, not when he arrives. A goal sent while another is under way
        replaces it. goto_cancel() and estop() end it. No checks: goto() is
        the one that refuses a bad goal."""
        from nav2_msgs.action import NavigateToPose
        from geometry_msgs.msg import PoseStamped
        from rclpy.action import ActionClient

        if self._nav_client is None:        # one for the life of the node, not one per goal
            self._nav_client = ActionClient(self._node, NavigateToPose, 'navigate_to_pose')
        client = self._nav_client
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
        self._goal_result = handle.get_result_async()
        self.goal = (gx, gy, gyaw)
        return handle

    goal = None     # (x, y, yaw) in odom of the goal last accepted; goal_active() says if it still stands

    def goal_active(self):
        """Is Nav2 still working on the goal last sent?"""
        return self._goal_handle is not None and not self._goal_result.done()

    def goto_cancel(self):
        """Ask Nav2 to drop the goal goto() is waiting on, from another
        thread; goto() then returns CANCELED. True if there was one. Nav2
        itself stays up - estop() is the one that kills it."""
        handle = self._goal_handle
        if handle is None:
            return False
        handle.cancel_goal_async()
        return True
