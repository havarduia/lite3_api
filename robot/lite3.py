#!/usr/bin/env python3
"""lite3 - one-import API for the DeepRobotics Lite3 Pro on ROS2 Foxy.

    source ~/robot/env/lite3_env.sh
    python3 -c "
    from robot.lite3 import Lite3
    with Lite3() as bot:
        bot.stand()
        bot.walk(0.5)
        print(bot.scan_text())
    "

A Lite3 instance owns one rclpy node spinning in a background thread, so state
is always live: bot.battery, bot.pose and bot.standing are plain attributes,
not callbacks you have to pump. Motion calls block until they finish and are
guaranteed to leave a zero Twist behind, including on exception or Ctrl-C.

Everything the robot gets wrong quietly is enforced here rather than left to
the caller:

  * RMW is pinned to CycloneDDS before rclpy loads. A FastRTPS shell discovers
    nothing and hangs with no error.
  * Auto mode is set before any velocity. In manual mode /cmd_vel is silently
    ignored - packets arrive and nothing happens.
  * Motion refuses below 20% battery, where the robot declines to stand and
    reports no reason.
  * Motion refuses unless basic_state == 6 (standing). Velocity is ignored
    when the robot is prone, again silently.
  * nav_start() refuses unless the robot is standing AND its pose has settled.
    The stand-up transition jumps leg odometry by over a metre in one step; a
    costmap built before that jump shows a clear path straight into a real
    obstacle.
  * The nav2 stack is launched into its own process group and stopped by that
    group. Its children do not carry the launch file's name, so pkill -f
    dr_nav2_mapless leaves them running - but widening the pattern to catch
    them reaches static_transform_publisher, which transfer_ros2 and
    realsense_ros2 also use, and takes those services down too.
  * goto() checks the goal cell for LETHAL before sending. NavFn cannot plan
    into one and the 0.15 m tolerance will not escape it.
"""
import math
import os
import signal
import threading
import time
from contextlib import contextmanager
os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

import rclpy                                           # noqa: E402
from geometry_msgs.msg import Twist                    # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry       # noqa: E402
from rclpy.executors import SingleThreadedExecutor     # noqa: E402
from rclpy.node import Node                            # noqa: E402
from rclpy.qos import (DurabilityPolicy, QoSProfile,   # noqa: E402
                       ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2                # noqa: E402
from std_msgs.msg import Int32MultiArray               # noqa: E402
from tf2_ros import Buffer, TransformListener          # noqa: E402

from . import protocol as P                            # noqa: E402
from .depth import Depth                               # noqa: E402
from .nav import LETHAL, Nav                           # noqa: E402
from .protocol import ACTIONS, Lite3Error              # noqa: E402,F401

MAX_TILT_DEG = 14.0      # STICK_PITCH full scale, see protocol.py
HEARTBEAT_HZ = 2.0

# --- /robot_state_debug ----------------------------------------------------
STATE_FIELDS = ('basic', 'gait', 'policy', 'motion', 'task', 'need_move',
                'zero_flag', 'battery')
STANDING = 6
# Lying/ready postures, reached different ways: 1 after a commanded lie-down,
# 8 after power-on, 98 seen on a cold boot. Transitional states (4, 5, 7) and
# zeroing (17) are not safe to toggle from.
LYING = (1, 8, 98)
# Settled, commandable states. 9 is deliberately NOT here: it is the
# transitional value on the way 8 -> 9 -> 1 when a controller appears (and it
# also shows up in the low-battery 1 -> 8 -> 9 -> 1 oscillation). Treating 9
# as ready makes the next command get refused for toggling mid-transition.
READY_STATES = (1, 6, 98)
# Settled but NOT armed: powered on (98) or the interlock dropped (8). jy_exe
# accepts nothing here - the FIRST command it receives is consumed by its own
# arming sequence (98 -> 9 -> 1, about 2 s) and never acted on. Measured
# 2026-09-21: heartbeat_start() alone does NOT arm, and neither do mode
# commands; only a real command triggers it, so the stand toggle is what gets
# eaten. stand() re-sends it rather than making callers sleep.
UNARMED = (8, 98)
# A toggle that is acted on moves basic_state out of the lying set within
# ~25 ms (measured both 98 -> 9 and 1 -> 17). Wait this long before reading
# "still lying" as "the toggle was swallowed". Generous on purpose: getting
# this wrong the other way sends a second toggle at a robot that IS standing
# up, which lies him back down. Costs nothing on the swallowed path, which
# cannot be detected before the transition settles at ~2 s regardless.
ARM_GRACE = 2.0

BATTERY_REFUSE = 20
BATTERY_WARN = 30

# --- limits ----------------------------------------------------------------
MAX_SPEED = 0.6          # m/s
MAX_YAW_RATE = 0.8       # rad/s
HARD_TIMEOUT = 30.0      # s, ceiling on any single motion call

# Turning in place, the body corner (0.30 m ahead, 0.20 m aside) sweeps a
# circle of ~0.36 m radius; anything nearer than this on the side being
# turned toward blocks the turn. The depth camera only sees +-45 deg, so
# something directly beside or behind him is still invisible.
TURN_SWEEP = 0.43

class _Node(Node):
    def __init__(self):
        super().__init__('lite3_api')
        best = QoSProfile(depth=1)
        best.reliability = ReliabilityPolicy.BEST_EFFORT
        cloud_qos = QoSProfile(depth=2)
        cloud_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub = self.create_publisher(Twist, 'cmd_vel', best)
        self.state = None
        self.odom = None
        self.cloud = None
        self.grid = None
        self.create_subscription(Int32MultiArray, '/robot_state_debug',
                                 self._state_cb, 10)
        self.create_subscription(Odometry, 'leg_odom2', self._odom_cb, 10)
        self.create_subscription(PointCloud2, '/camera/depth/color/points',
                                 self._cloud_cb, cloud_qos)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._grid_cb, latched)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

    def _state_cb(self, m):
        self.state = dict(zip(STATE_FIELDS, m.data))

    def _odom_cb(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = (p.x, p.y, yaw)

    def _cloud_cb(self, m):
        self.cloud = m

    def _grid_cb(self, m):
        self.grid = m


class Lite3(Nav, Depth):
    """Live handle on the robot. Use as a context manager."""

    def __init__(self, auto_mode=True, timeout=10.0, estop_on_sigint=True):
        rclpy.init(args=None)
        self._node = _Node()
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self._node)
        self._stop_spin = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        self._closed = False
        self._clients = []
        self._hb_stop = threading.Event()
        self._hb_thread = None
        self._hb_count = 0
        self._goal_handle = None
        self._estopped = False
        self._prev_sigint = None
        if estop_on_sigint:
            self._install_sigint()
        if not self._wait(lambda: self._node.state is not None, timeout):
            self.close()
            raise Lite3Error(
                'no /robot_state_debug within %.0fs. Is transfer_ros2.service '
                'running, and is Lite_motion stopped? (They fight over UDP '
                '43897.)' % timeout)
        if auto_mode:
            self.auto()

    # --- emergency stop ----------------------------------------------------
    # Ctrl-C alone is NOT a stop once Nav2 is driving: controller_server is a
    # separate process publishing its own cmd_vel, so zero Twists from here
    # just race it. A real stop has to remove the other publisher too.
    def _install_sigint(self):
        try:
            self._prev_sigint = signal.getsignal(signal.SIGINT)
            signal.signal(signal.SIGINT, self._on_sigint)
        except ValueError:
            # Not the main thread; signals cannot be installed there.
            self._prev_sigint = None

    def _on_sigint(self, signum, frame):
        print('\n*** Ctrl-C - EMERGENCY STOP ***')
        self.estop()
        raise KeyboardInterrupt

    def estop(self, disarm=False):
        """Stop the robot as hard as software can, in the order that matters.

        1. SIGINT is ignored for the duration, so a second Ctrl-C cannot
           abort the stop halfway and leave the robot walking.
        2. Any Nav2 goal is cancelled.
        3. Zero Twist burst, so whatever is still listening gets a stop.
        4. The Nav2 stack is killed - this is the one that matters, because
           controller_server outlives this process and keeps publishing.
        5. Another zero Twist burst, now that nothing else is publishing.

        disarm=True additionally stops the heartbeat, after which jy_exe
        ignores every network command (basic_state falls to 8). That is the
        strongest stop available, but UNTESTED while the robot is standing -
        it may or may not settle gracefully - so it is opt-in.
        """
        prev = None
        try:
            prev = signal.getsignal(signal.SIGINT)
            signal.signal(signal.SIGINT, signal.SIG_IGN)
        except ValueError:
            pass
        try:
            if self._goal_handle is not None:
                try:
                    self._goal_handle.cancel_goal_async()
                except Exception:
                    pass
                self._goal_handle = None
            self.halt()
            try:
                self.nav_stop()
            except Exception:
                pass
            self.halt()
            if disarm:
                self.heartbeat_stop()
            self._estopped = True
        finally:
            if prev is not None:
                try:
                    signal.signal(signal.SIGINT, prev)
                except ValueError:
                    pass
        return True

    # --- plumbing ----------------------------------------------------------
    def _spin(self):
        while not self._stop_spin.is_set():
            self._exec.spin_once(timeout_sec=0.05)

    @staticmethod
    def _wait(pred, timeout, period=0.05):
        end = time.time() + timeout
        while time.time() < end:
            if pred():
                return True
            time.sleep(period)
        return pred()

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self._prev_sigint is not None:
            try:
                signal.signal(signal.SIGINT, self._prev_sigint)
            except ValueError:
                pass
        # Stop the robot BEFORE dropping the keepalive, not after. Stopping the
        # keepalive disengages the interlock (basic_state -> 8), and whether a
        # STANDING robot settles gracefully from there has never been observed
        # - we have only ever seen it disengage while he was lying down. So
        # halt while commands are still being accepted, then disarm.
        try:
            self.halt()
        except Exception:
            pass
        self.heartbeat_stop()
        # Teardown order is load-bearing and each step is a crash we hit:
        #   1. stop the spin thread FIRST - destroying anything underneath a
        #      live executor segfaults,
        #   2. then action clients, while the node is still alive, or
        #      ActionClient.__del__ runs post-teardown and raises InvalidHandle,
        #   3. then the executor and the node.
        self._stop_spin.set()
        self._thread.join(timeout=5.0)
        for c in self._clients:
            try:
                c.destroy()
            except Exception:
                pass
        self._clients = []
        try:
            self._exec.shutdown(timeout_sec=2.0)
            self._exec.remove_node(self._node)
            self._node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # --- state -------------------------------------------------------------
    @property
    def state(self):
        """dict: basic, gait, policy, motion, task, need_move, zero_flag, battery."""
        return dict(self._node.state or {})

    @property
    def battery(self):
        """Percent, or None on an older transfer build that does not publish it."""
        return self.state.get('battery')

    @property
    def standing(self):
        return self.state.get('basic') == STANDING

    @property
    def pose(self):
        """(x, y, yaw) in the odom frame, or None."""
        return self._node.odom

    def wait_pose(self, timeout=5.0):
        if not self._wait(lambda: self._node.odom is not None, timeout):
            raise Lite3Error('no /leg_odom2 - refusing to move blind')
        return self._node.odom

    def pose_settled(self, window=1.5, tol=0.02):
        """True if the robot's odometry has not moved for `window` seconds.

        The stand-up transition jumps the pose by over a metre in one step.
        Anything that builds a world model must wait for this to be True.
        """
        self.wait_pose()
        a = self.pose
        time.sleep(window)
        b = self.pose
        return math.hypot(b[0] - a[0], b[1] - a[1]) <= tol

    # --- posture and mode --------------------------------------------------
    
    def heartbeat_start(self):
        """Emit the controller keepalive at 2 Hz until stopped.
        SAFETY: this replaces the handheld's role in the interlock - the robot
        will accept commands with no controller powered on."""
        if self.heartbeat_running:
            return
        self._hb_stop.clear()
        self._hb_count = 0
        self._hb_thread = threading.Thread(target=self._hb_loop, daemon=True)
        self._hb_thread.start()

    def _hb_loop(self):
        while not self._hb_stop.is_set():
            try:
                P.send(P.HEARTBEAT)
                self._hb_count += 1
            except OSError:
                pass
            self._hb_stop.wait(1.0 / HEARTBEAT_HZ)

    def heartbeat_stop(self):
        self._hb_stop.set()
        if self._hb_thread is not None:
            self._hb_thread.join(timeout=2.0)
            self._hb_thread = None

    @property
    def heartbeat_running(self):
        return self._hb_thread is not None and self._hb_thread.is_alive()

    @contextmanager
    def tilt(self, pitch_deg, settle=1.5):
        """Hold the body pitched NOSE UP by pitch_deg (negative = nose down),
        standing in place, for the duration of the with-block.

            with bot.tilt(12):
                ...look at the person's face...

        Uses the app's posture mode: enter, resend the pitch stick at 10 Hz
        (as the app does), then level, exit, and settle - on every exit path.
        Don't walk inside it; the robot is not in walking mode then.
        settle: seconds to wait for the body to reach the tilt before the
        block runs (about 1.5-2 s to get there fully).
        """
        self._require_standing()
        frac = max(-1.0, min(1.0, pitch_deg / MAX_TILT_DEG))
        value = int(-32767 * frac)              # negative stick = nose up
        stop = threading.Event()

        def hold():
            while not stop.is_set():
                P.send(P.STICK_PITCH, value)
                stop.wait(0.1)

        P.send(P.POSTURE_ENTER)
        time.sleep(0.5)
        th = threading.Thread(target=hold, daemon=True)
        th.start()
        try:
            time.sleep(settle)                  # let the body reach the tilt
            yield
        finally:
            stop.set()
            th.join()
            for _ in range(10):                 # back to level before leaving
                P.send(P.STICK_PITCH, 0)
                time.sleep(0.1)
            P.send(P.POSTURE_EXIT)
            time.sleep(1.0)

    @contextmanager
    def heartbeat(self):
        self.heartbeat_start()
        try:
            yield self
        finally:
            self.heartbeat_stop()
    
    def wait_ready(self, timeout=10.0):
        """Block until basic_state is SETTLED and commandable. True if it is.

        The transition after a controller appears runs 8 -> 9 -> 1 over about
        two seconds, and **9 is transitional, not ready**. Waiting merely for
        "not 8" returns during that window, and the stand() that follows is
        refused for toggling from a transitional state. So wait for one of the
        settled values instead.

        SETTLED IS NOT THE SAME AS ARMED. 98 and 8 (see UNARMED) are stable -
        nothing is in flight, so this correctly returns True - but jy_exe will
        swallow the next command to arm itself. That is not something waiting
        can fix: measured 2026-09-21, the robot sits at 98 indefinitely under a
        running heartbeat until something commands it. stand() handles the
        swallowed toggle, so callers do NOT need to sleep after
        heartbeat_start().
        """
        return self._wait(lambda: self.state.get('basic') in READY_STATES,
                          timeout)

    def auto(self):
        """Required before any velocity. No topic reports the mode, so we just
        set it; touching a handheld joystick silently reverts it to manual.

        Also leaves posture ("twist body") mode first: left in it, the robot
        reports standing (basic_state 6) but silently ignores every velocity
        command - seen 2026-09-18, probably after the app's posture control.
        """
        P.send(P.POSTURE_EXIT)
        time.sleep(0.3)
        P.send(P.MODE_AUTO)
        time.sleep(0.5)

    def manual(self):
        P.send(P.MODE_MANUAL)
        time.sleep(0.5)

    def zero(self):
        P.send(P.ZERO)

    def stand(self, timeout=20.0):
        """Stand up. Idempotent - the underlying command is a toggle, this is not.

        From an UNARMED state the first toggle is eaten by jy_exe's arming
        sequence and the robot ends up armed but still lying (98 -> 9 -> 1), so
        it is sent again. Re-toggling is safe ONLY because we wait for a
        SETTLED lying value first - a toggle sent while 9/17/4/5/7 is in flight
        is what lies the robot back down mid-move.
        """
        if self.standing:
            return True
        basic = self.state.get('basic')
        if basic not in LYING:
            raise Lite3Error(
                'basic_state=%s is transitional or unknown; expected one of %s '
                '(lying). Not toggling - it could lie the robot down mid-move.'
                % (basic, LYING))
        self._require_battery()
        for retry in (False, True):
            if self._toggle_stand(timeout):
                time.sleep(1.0)      # let the pose settle before anyone reads it
                return True
            basic = self.state.get('basic')
            if retry or basic not in LYING:
                break
            # Settled back in a lying state: the toggle armed him instead of
            # standing him. One retry, now that jy_exe is listening.
            print('NOTE: first stand toggle was consumed arming the robot '
                  '(basic_state=%s) - sending it again' % basic)
        raise Lite3Error(
            'timed out standing (basic_state=%s). Is the handheld powered ON, '
            'or heartbeat_start() running? With neither, jy_exe ignores every '
            'network command and basic_state sits at 8.'
            % self.state.get('basic'))

    def _toggle_stand(self, timeout):
        """One stand toggle. True if standing; False if he settled back down.

        Returns early on a swallowed toggle instead of burning the whole
        timeout: the arming transition is over within ~2 s, so a lying state
        still showing after ARM_GRACE means nothing is coming.
        """
        P.send(P.STAND_TOGGLE)
        sent = time.time()
        end = sent + timeout
        while time.time() < end:
            if self.standing:
                return True
            if (time.time() - sent > ARM_GRACE
                    and self.state.get('basic') in LYING):
                return False
            time.sleep(0.05)
        return self.standing

    def sit(self, timeout=20.0):
        """Lie down. Idempotent."""
        if not self.standing:
            return True
        self.halt()
        P.send(P.STAND_TOGGLE)
        return self._wait(lambda: self.state.get('basic') in LYING, timeout)

    def action(self, name, force=False):
        """Play a built-in trick from protocol.ACTIONS. Does NOT stand or sit for you.

        Sent 3 times at 1 Hz, as Lite3_LLM and lite3-sdk both do. Returns when
        the sends are done, not when the trick is: no topic reports that.
        """
        code, posture = ACTIONS[name]
        self._require_battery(force)
        basic = self.state.get('basic')
        if posture == 'stand':
            self._require_standing()
            self.halt()
        elif basic != 1:
            # 8/98 are unarmed: jy_exe would eat the first send arming itself
            # and the next one would land mid-transition.
            raise Lite3Error(
                '%s starts lying and armed (basic_state 1), not %s. '
                'stand() then sit() gets there.' % (name, basic))
        for _ in range(3):
            P.send(code)
            time.sleep(1.0)

    # --- motion ------------------------------------------------------------
    def _require_battery(self, force=False):
        b = self.battery
        if b is None or force:
            return
        if b < BATTERY_REFUSE:
            raise Lite3Error(
                'battery %d%% is below %d%%. The robot will refuse to stand and '
                'will not say why. Pass force=True to override.'
                % (b, BATTERY_REFUSE))
        if b < BATTERY_WARN:
            print('WARNING: battery %d%% - charge soon' % b)

    def _require_standing(self):
        if not self.standing:
            raise Lite3Error(
                'basic_state=%s, not standing (%d). Velocity is silently '
                'ignored unless the robot is standing - call stand() first.'
                % (self.state.get('basic'), STANDING))

    def _drive(self, vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self._node.pub.publish(t)

    def halt(self):
        """Publish a zero Twist, repeatedly. Best-effort QoS can drop one."""
        for _ in range(20):
            self._drive()
            time.sleep(0.02)

    def _run(self, vx, vy, wz, done, limit, force, abort=None):
        """Drive until done(), the time limit, or abort() returns a reason."""
        self._require_battery(force)
        self._require_standing()
        self.auto()
        self.wait_pose()
        start = self.pose
        deadline = time.time() + min(limit, HARD_TIMEOUT)
        reason = 'time limit'
        try:
            while time.time() < deadline:
                self._drive(vx, vy, wz)
                time.sleep(0.05)
                if abort is not None:
                    stop = abort()
                    if stop:
                        reason = stop
                        break
                if done(start, self.pose):
                    reason = 'target reached'
                    break
        finally:
            self.halt()
            time.sleep(0.5)
        return {'reason': reason, 'start': start, 'end': self.pose}

    def steer(self, control, limit=HARD_TIMEOUT, force=False):
        """Drive with velocities recomputed every 50 ms cycle.

        control() returns (vx, wz) to keep going or a string reason to stop.
        Same guards as walk()/turn(): battery, standing, auto mode, velocities
        clamped to MAX_SPEED / MAX_YAW_RATE, the HARD_TIMEOUT ceiling, and a
        halt on every exit. For closed-loop behaviours such as following a
        person, where no fixed (vx, wz) will do.
        """
        self._require_battery(force)
        self._require_standing()
        self.auto()
        self.wait_pose()
        start = self.pose
        deadline = time.time() + min(limit, HARD_TIMEOUT)
        reason = 'time limit'
        try:
            while time.time() < deadline:
                out = control()
                if isinstance(out, str):
                    reason = out
                    break
                vx, wz = out
                self._drive(max(-MAX_SPEED, min(MAX_SPEED, vx)), 0.0,
                            max(-MAX_YAW_RATE, min(MAX_YAW_RATE, wz)))
                time.sleep(0.05)
        finally:
            self.halt()
            time.sleep(0.5)
        end = self.pose
        return {'reason': reason, 'start': start, 'end': end,
                'moved': math.hypot(end[0] - start[0], end[1] - start[1])}

    def walk(self, distance, speed=0.15, stop_distance=0.6, force=False):
        """Walk forward `distance` metres. Blocking. Negative walks backward.

        Stops early if the depth camera sees something within `stop_distance`
        metres of the path ahead; the result's 'reason' says which obstacle
        and how far. Pass stop_distance=None to walk blind.

        The camera faces FORWARD ONLY, so the guard cannot apply to a negative
        distance - backing up is always blind, and it says so once.
        """
        speed = abs(speed)
        if not 0 < speed <= MAX_SPEED:
            raise Lite3Error('speed must be in (0, %s] m/s' % MAX_SPEED)
        vx = math.copysign(speed, distance)
        target = abs(distance)

        def done(s, p):
            return p is not None and math.hypot(p[0] - s[0], p[1] - s[1]) >= target

        abort = None
        if stop_distance is not None:
            # Posture first: a prone robot's camera stares at the floor a few
            # centimetres away, so the clearance pre-flight would report an
            # "obstacle" and hide the real problem.
            self._require_battery(force)
            self._require_standing()
            if distance < 0:
                print('NOTE: the depth camera only looks forward - backing up '
                      'is unguarded')
            else:
                # Fail before moving rather than stopping one cycle in.
                here = self.clearance()
                if here <= stop_distance:
                    raise Lite3Error(
                        'obstacle already at %.2f m, inside the %.2f m stop '
                        'distance. Refusing to start.' % (here, stop_distance))
                # clearance() walks the whole cloud, so sample it at 4 Hz
                # rather than every 50 ms control cycle.
                last = {'t': time.time()}

                def abort():
                    if time.time() - last['t'] < 0.25:
                        return None
                    last['t'] = time.time()
                    try:
                        c = self.clearance()
                    except Lite3Error:
                        return 'lost the depth stream - stopped rather than ' \
                               'walking blind'
                    if c <= stop_distance:
                        return 'obstacle at %.2f m' % c
                    return None

        r = self._run(vx, 0.0, 0.0, done, target / speed + 10.0, force, abort)
        r['moved'] = math.hypot(r['end'][0] - r['start'][0],
                                r['end'][1] - r['start'][1])
        return r

    def strafe(self, distance, speed=0.15, force=False):
        """Crab sideways. Positive is left."""
        vy = math.copysign(abs(speed), distance)
        target = abs(distance)

        def done(s, p):
            return p is not None and math.hypot(p[0] - s[0], p[1] - s[1]) >= target

        return self._run(0.0, vy, 0.0, done, target / abs(speed) + 10.0, force)

    def turn(self, radians, rate=0.4, force=False, guard=True):
        """Turn in place by `radians` RELATIVE to the heading at the moment
        this is called. Positive is left. Any magnitude works, including 180
        and beyond a full revolution.

        Measured by accumulating per-sample increments, NOT by wrapping the
        total difference from the start. Wrapping the total saturates at pi:
        a 180 degree request could never satisfy `>= pi`, so the turn fell
        through to its time limit and span ~409 degrees before stopping.
        """
        rate = min(abs(rate), MAX_YAW_RATE)
        wz = math.copysign(rate, radians)
        target = abs(radians)
        acc = {'prev': None, 'total': 0.0}

        def done(s, p):
            if p is None:
                return False
            if acc['prev'] is None:
                acc['prev'] = s[2]
            # Wrap each INCREMENT, never the running total. Sampling is 20 Hz
            # and the rate is capped at 0.8 rad/s, so a genuine step is under
            # 0.05 rad; anything near pi is the -pi/+pi seam, not motion.
            step = math.atan2(math.sin(p[2] - acc['prev']),
                              math.cos(p[2] - acc['prev']))
            acc['total'] += step
            acc['prev'] = p[2]
            return abs(acc['total']) >= target

        abort = None
        if guard:
            if self._node.cloud is None:
                print('NOTE: no depth stream - turning without the side check')
            else:
                side = 0 if radians > 0 else 1          # left, right
                last = {'t': 0.0}

                def abort():
                    if time.time() - last['t'] < 0.25:  # scan() walks the cloud
                        return None
                    last['t'] = time.time()
                    try:
                        near = self.side_clear()[side]
                    except Lite3Error:
                        return 'lost the depth stream - stopped turning'
                    if near < TURN_SWEEP:
                        return 'obstacle %.2f m to the %s - stopped turning' % (
                            near, ('left', 'right')[side])
                    return None

        r = self._run(0.0, 0.0, wz, done, target / rate + 10.0, force, abort)
        r['turned'] = acc['total']          # unwrapped, so 180+ reads correctly
        return r

    def turn_deg(self, degrees, **kw):
        r = self.turn(math.radians(degrees), **kw)
        r['turned_deg'] = math.degrees(r['turned'])
        return r

    # --- voice -------------------------------------------------------------
    # The speaker is on the motion computer, not here; voice.py does the ssh +
    # aplay. Imported lazily so lite3 still loads on a box without voice.py,
    # and cached so the Voice handle (which only holds config) is made once.
    @property
    def voice(self):
        v = getattr(self, '_voice', None)
        if v is None:
            from .voice import Voice
            v = self._voice = Voice()
        return v

    def say(self, text, wait=True, voice=None, rocky=False):
        """Speak `text` out of the robot. Needs a TTS engine - see voice.say."""
        return self.voice.say(text, wait=wait, voice=voice, rocky=rocky)

    def play(self, name, wait=True):
        """Play one of the robot's built-in clips, e.g. play('okstop')."""
        return self.voice.play(name, wait=wait)

    # --- summary -----------------------------------------------------------
    def status(self):
        s = self.state
        p = self.pose
        return ('basic={basic} gait={gait} battery={battery}% '.format(**s)
                + ('standing' if self.standing else 'not standing')
                + (' pose=(%.2f, %.2f, %.0f deg)' % (p[0], p[1], math.degrees(p[2]))
                   if p else ' pose=unknown')
                + ' nav2=' + ('up' if self.nav_running else 'down'))


def _cli():
    import sys
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        print('commands: status stand sit auto walk <m> turn <deg> scan '
              'nav-start nav-stop goto <m> cost action <%s>'
              % '|'.join(ACTIONS))
        return
    cmd = args[0]
    with Lite3(auto_mode=(cmd not in ('status', 'scan', 'cost'))) as bot:
        if cmd == 'status':
            print(bot.status())
        elif cmd == 'stand':
            bot.stand(); print(bot.status())
        elif cmd == 'sit':
            bot.sit(); print(bot.status())
        elif cmd == 'auto':
            bot.auto(); print('auto mode set')
        elif cmd == 'walk':
            print(bot.walk(float(args[1])))
        elif cmd == 'turn':
            print(bot.turn_deg(float(args[1])))
        elif cmd == 'scan':
            print(bot.scan_text())
        elif cmd == 'nav-start':
            bot.nav_start(); print('nav2 up')
        elif cmd == 'nav-stop':
            bot.nav_stop(); print('nav2 down')
        elif cmd == 'goto':
            print('status', bot.goto(float(args[1])))
        elif cmd == 'estop':
            print('estop:', bot.estop(disarm='--disarm' in args))
            print(bot.status())
        elif cmd == 'action':
            bot.action(args[1]); print(bot.status())
        elif cmd == 'cost':
            for d, c in bot.cost_ahead():
                mark = ' LETHAL' if c is not None and c >= LETHAL else (
                    ' inflated' if c is not None and c > 50 else '')
                print('  +%4.2f m  cost %s%s' % (d, c, mark))
        else:
            print('unknown command: ' + cmd)


if __name__ == '__main__':
    _cli()
