"""nav - the Nav2 stack: start/stop it, read its costmap, goto().
Mixed into Lite3, so these are bot.nav_start(), bot.goto(1.5) etc.

Two modes. Mapless: Nav2 plans in odom, goto() goes some metres ahead.
Map mode (load_floor): Nav2 plans on a floor map, and go_to() goes to a
named place on it once set_pose() has told him where he is.
"""
import math
import os
import signal
import subprocess
import time

from . import locate
from . import places as place_file
from .lio_relay import inv, join
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
LOC_STALE_S = 1.5       # locate.py speaks twice a second; quiet this long, he does not know where he is
POSE_STD = (0.3, 0.26)  # m, rad: how sure a pose given by hand (a tap on the map) is taken to be
ROUTE_TIMEOUT = 300.0   # s for one go_to(): 30 m takes about two minutes
THERE_M = 0.2           # nearer than this the planner makes no path, and he is there anyway
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
    or None when that is under `minimum`: set when xy_goal_tolerance was 0.25
    (0.15 now); a goal inside the tolerance "arrives" at once."""
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

    def nav_start(self, timeout=40.0, floor=None):
        """Launch Nav2: mapless, or on a floor's map (load_floor()). Refuses
        unless the robot is standing and its pose has settled - a costmap built
        across the stand-up pose jump shows a clear path straight into a real
        obstacle."""
        self._require_standing()
        if not self.pose_settled():
            raise Lite3Error(
                'pose is still moving; refusing to build a costmap around it. '
                'Wait for the robot to settle after standing.')
        script = ['start_nav2_mapless.sh']
        if floor is not None:
            place_file.floor_dir(floor)         # a floor that is there, before anything is stopped
            script = ['start_nav2_map.sh', floor]
        self.nav_stop()
        # setsid: a session and process group of its own, for nav_stop().
        with open('/tmp/nav2.log', 'w') as log:
            subprocess.Popen(['setsid', os.path.join(REPO, 'env', script[0])] + script[1:],
                             stdout=log, stderr=subprocess.STDOUT,
                             stdin=subprocess.DEVNULL)
        if not self._wait(lambda: self.nav_running, timeout):
            raise Lite3Error('nav2 did not come up; see /tmp/nav2.log')
        if floor is not None:
            # No costmap yet: Nav2 only finishes starting once AMCL has a pose.
            if not self._wait(lambda: self._loc() is not None, timeout):
                raise Lite3Error('nav2 is up but robot/locate.py says nothing; see /tmp/nav2.log')
        elif not self._wait(lambda: self._node.grid is not None, timeout):
            raise Lite3Error('nav2 is up but published no costmap; see /tmp/nav2.log')
        groups = self._nav_groups()
        if len(groups) > 1:
            raise Lite3Error(
                '%d nav2 process groups are running - an earlier stack '
                'survived. Call nav_stop() and retry.' % len(groups))
        return True

    def cost_at(self, x, y, frame='odom'):
        """Global costmap cost at a point given in odom, or on the floor map
        (frame='map'). None if off the map."""
        g = self._node.grid
        if g is None:
            raise Lite3Error('no global costmap - is nav2 running?')
        x, y, _ = self._in_nav_frame(x, y, 0.0, frame)
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

        status = self._goal_wait(self.goal_send(gx, gy, gyaw), timeout)
        if status == SUCCEEDED and heading_deg is not None:
            err = wrap(gyaw - self.wait_pose()[2])
            if abs(err) > math.radians(5):
                self.turn(err)
        return status

    def _goal_wait(self, handle, timeout, lost=None):
        """Wait out the goal just sent and return its action status. lost(): a
        reason to give up on the way, or None."""
        result_fut = self._goal_result
        why = [None]

        def over():
            why[0] = lost() if lost else None
            # estop() from another thread drops the handle: Nav2 is being killed and
            # the result may never come.
            return result_fut.done() or self._goal_handle is None or why[0]
        try:
            if not self._wait(over, timeout):
                handle.cancel_goal_async()
                self.halt()
                raise Lite3Error('goal timed out after %.0fs; cancelled' % timeout)
            if why[0] and not result_fut.done():
                handle.cancel_goal_async()
                self.halt()
                raise Lite3Error('lost on the way, stopped: ' + why[0])
        finally:
            self._goal_handle = None
        status = result_fut.result().status if result_fut.done() else CANCELED
        if status != SUCCEEDED:
            self.halt()                     # whoever ended it, he ends stopped
        return status

    def goal_send(self, gx, gy, gyaw=0.0, frame='odom'):
        """Hand Nav2 a goal at a point given in odom (or frame='map') and return
        once it is accepted, not when he arrives. A goal sent while another is
        under way replaces it. goto_cancel() and estop() end it. No checks:
        goto() and go_to_point() are the ones that refuse a bad goal."""
        from nav2_msgs.action import NavigateToPose
        from geometry_msgs.msg import PoseStamped
        from rclpy.action import ActionClient

        if self._nav_client is None:        # one for the life of the node, not one per goal
            self._nav_client = ActionClient(self._node, NavigateToPose, 'navigate_to_pose')
        client = self._nav_client
        if not client.wait_for_server(timeout_sec=10.0):
            raise Lite3Error('navigate_to_pose action server not available')

        # Nav2 takes the numbers as being in the frame it plans in, whatever the
        # header says, so in map mode an odom point is moved onto the map here.
        nx, ny, nyaw = self._in_nav_frame(gx, gy, gyaw, frame)
        if frame != 'odom':
            gx, gy, gyaw = join(inv(self._fix()), (gx, gy, gyaw))
        goal = PoseStamped()
        goal.header.frame_id = self._nav_frame()
        goal.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = float(nx), float(ny)
        goal.pose.orientation.z = math.sin(nyaw / 2.0)
        goal.pose.orientation.w = math.cos(nyaw / 2.0)
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

    # --- map mode ----------------------------------------------------------
    # A floor map under Nav2 (env/start_nav2_map.sh): AMCL places him on it from
    # the lidar, robot/locate.py says whether to believe it. README section 7.

    _pose_pub = None

    def floors(self):
        """The floors that have a map (~/lite3_maps/<floor>/, from env/get_map.sh)."""
        return place_file.floors()

    @property
    def floor(self):
        """The floor Nav2 is running on, or None: mapless, or Nav2 down. Read off
        the running stack, so every Lite3 gives the same answer."""
        out = subprocess.run(['pgrep', '-af', r'robot\.locate '], capture_output=True, text=True).stdout
        return out.split('\n')[0].split()[-1] if out.strip() else None

    def _floor(self):
        floor = self.floor
        if floor is None:
            raise Lite3Error('no floor is loaded - load_floor() first')
        return floor

    def load_floor(self, name, timeout=40.0):
        """Start Nav2 on that floor's map (replacing whatever Nav2 was up). He
        does not know where he is on it yet: set_pose() next, and Nav2 only
        finishes starting then."""
        return self.nav_start(timeout=timeout, floor=name)

    def _loc(self):
        """robot/locate.py's newest reading, or None once it has gone quiet."""
        n = self._node
        if n.loc is None or time.time() - n.loc_time > LOC_STALE_S:
            return None
        return n.loc

    def lost_why(self):
        """None while he knows where he is on the map, else why not, in words."""
        loc = self._loc()
        if loc is None:
            return 'robot/locate.py says nothing (is a floor loaded?)'
        return None if int(loc.state) == locate.LOCALIZED else locate.WHY[int(loc.state)]

    @property
    def localized(self):
        return self.lost_why() is None

    def _fix(self):
        """odom -> map as (x, y, yaw): locate.py's last pose against the odometry
        of the moment it came. None while there is no pose."""
        loc, n = self._loc(), self._node
        if loc is None or int(loc.state) == locate.NO_POSE or n.loc_odom is None:
            return None
        return join((loc.x, loc.y, loc.yaw), inv(n.loc_odom))

    @property
    def map_pose(self):
        """(x, y, yaw) on the floor map, or None. Follows odometry between
        locate.py's two readings a second."""
        fix, odom = self._fix(), self._node.odom
        if fix is None or odom is None:
            return None
        x, y, yaw = join(fix, odom)
        return x, y, wrap(yaw)

    def _nav_frame(self):
        """The frame Nav2 plans in: its costmap says. odom until there is one."""
        g = self._node.grid
        return g.header.frame_id if g is not None else 'odom'

    def _in_nav_frame(self, x, y, yaw, frame):
        """A pose given in 'odom' or 'map', in the frame Nav2 plans in."""
        target = self._nav_frame()
        if frame == target:
            return x, y, yaw
        fix = self._fix()
        if fix is None:
            raise Lite3Error('no way from %s to %s: %s' % (frame, target, self.lost_why() or 'no map pose'))
        return join(fix, (x, y, yaw)) if target == 'map' else join(inv(fix), (x, y, yaw))

    def set_pose(self, x, y, yaw_deg, timeout=30.0):
        """Tell him where he is on the loaded floor's map, roughly: within about
        half a metre and 20 degrees. Returns once robot/locate.py agrees and
        Nav2, which was waiting for this, has finished starting. Raises if what
        the lidar sees does not fit the map there."""
        from geometry_msgs.msg import PoseWithCovarianceStamped

        self._floor()
        self._require_standing()            # lying down the relay passes no scans on: AMCL would see nothing
        if self._pose_pub is None:
            self._pose_pub = self._node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        m.pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2.0)
        m.pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2.0)
        m.pose.covariance[0] = m.pose.covariance[7] = POSE_STD[0] ** 2
        m.pose.covariance[35] = POSE_STD[1] ** 2
        end, said = time.time() + timeout, None
        while time.time() < end:
            loc = self._loc()
            taken = loc is not None and int(loc.state) != locate.NO_POSE and dist((loc.x, loc.y), (x, y)) < 1.0
            # Again until AMCL has it: it may not be listening yet, or still hold an older pose.
            if not taken and (said is None or time.time() - said > 3.0):
                self._pose_pub.publish(m)
                said = time.time()
            if taken and self.localized and self._node.grid is not None:
                return True
            time.sleep(0.2)
        raise Lite3Error('gave the pose (%.2f, %.2f, %.0f deg), but: %s' % (
            x, y, yaw_deg, self.lost_why() or 'Nav2 did not finish starting; see /tmp/nav2.log'))

    def go_to_point(self, x, y, yaw_deg=None, timeout=ROUTE_TIMEOUT):
        """Navigate to a point on the floor map. Returns the action status, as
        goto() does; goto_cancel() and estop() end it the same way.

        yaw_deg: the heading to end on, in the map's frame; None leaves him
        facing the way he arrived. Refuses unless he knows where he is, and a
        point off the map, in a wall or off the mapped floor. Stops, and
        raises, if he stops knowing where he is on the way.
        """
        for v in (x, y) if yaw_deg is None else (x, y, yaw_deg):
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise Lite3Error('a map point is numbers, not %r' % (v,))
        self._require_standing()
        self._require_battery()
        self._floor()
        why = self.lost_why()
        if why:
            raise Lite3Error('not going: ' + why)
        c = self.cost_at(x, y, frame='map')
        if c is None:
            raise Lite3Error('point (%.2f, %.2f) is outside the map' % (x, y))
        if c >= LETHAL:
            raise Lite3Error('point (%.2f, %.2f) is in a wall, an obstacle or off the mapped floor' % (x, y))
        here = self.map_pose
        status = SUCCEEDED
        if dist(here, (x, y)) >= THERE_M:
            gyaw = math.atan2(y - here[1], x - here[0]) if yaw_deg is None else math.radians(yaw_deg)
            status = self._goal_wait(self.goal_send(x, y, gyaw, frame='map'), timeout, lost=self.lost_why)
        if status == SUCCEEDED and yaw_deg is not None:
            err = wrap(math.radians(yaw_deg) - self.map_pose[2])
            if abs(err) > math.radians(5):
                self.turn(err)
        return status

    def go_to(self, name, timeout=ROUTE_TIMEOUT):
        """Navigate to a saved place on the loaded floor, by name."""
        p = place_file.find(self._floor(), name)
        return self.go_to_point(p['x'], p['y'], p['yaw_deg'], timeout=timeout)

    def places(self):
        """The places saved on the loaded floor."""
        return place_file.load(self._floor())

    def save_place(self, name, x=None, y=None, yaw_deg=None):
        """Save a place on the loaded floor: where he stands and faces now, or
        a point on the map (heading 0 unless given)."""
        floor = self._floor()
        if x is None or y is None:
            why = self.lost_why()
            if why:
                raise Lite3Error('cannot save where he stands: ' + why)
            x, y, yaw = self.map_pose
            yaw_deg = math.degrees(yaw) if yaw_deg is None else yaw_deg
        else:
            c = self.cost_at(x, y, frame='map')
            if c is None or c >= LETHAL:
                raise Lite3Error('point (%.2f, %.2f) is off the map, in a wall or off the mapped floor' % (x, y))
        return place_file.save(floor, name, x, y, yaw_deg or 0.0)

    def rename_place(self, old, new):
        return place_file.rename(self._floor(), old, new)

    def delete_place(self, name):
        place_file.delete(self._floor(), name)
