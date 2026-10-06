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

State is live attributes (bot.battery, bot.pose, bot.standing). Motion calls
block and always leave a zero Twist behind. What the API enforces for you,
and why, is in README.md section 5.
"""
import math
import os
import signal
import threading
import time
from contextlib import contextmanager
from functools import cached_property
os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

import rclpy                                           # noqa: E402
from geometry_msgs.msg import Twist                    # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry       # noqa: E402
from rclpy.executors import (ShutdownException,        # noqa: E402
                             SingleThreadedExecutor, TimeoutException)
from rclpy.node import Node                            # noqa: E402
from rclpy.qos import (DurabilityPolicy, QoSProfile,   # noqa: E402
                       ReliabilityPolicy)
from sensor_msgs.msg import Imu, PointCloud2           # noqa: E402
from std_msgs.msg import Float64, Int32MultiArray      # noqa: E402

from . import protocol as P                            # noqa: E402
from .depth import Depth, sampled                      # noqa: E402
from .nav import LETHAL, Nav, clamp, dist, wrap        # noqa: E402
from .protocol import ACTIONS, Lite3Error              # noqa: E402,F401

MAX_TILT_DEG = 14.0      # STICK_PITCH full scale, see protocol.py
HEARTBEAT_HZ = 2.0
# The robot's topics come 160 times a second each. Taking every message cost
# most of a core, so each queue keeps only the newest and is read this often.
SPIN_HZ = 25.0           # pose, tilt and state are at most 40 ms old; the motion loop runs at 20 Hz
SPIN_BURST = 12          # at most this many callbacks per read: one per subscription and a few to spare
# The point cloud is megabytes a frame and unpacking it is what costs: 30 a
# second took half a core. The depth checks look 4 times a second.
CLOUD_HZ = 10.0

# --- /robot_state_debug ---
# error and charging come from our Jetson2Motion patch; an older build omits them.
STATE_FIELDS = ('basic', 'gait', 'policy', 'motion', 'task', 'need_move',
                'zero_flag', 'battery', 'error', 'charging')
STANDING = 6
# Settled lying states: 1 after a commanded lie-down, 8 no interlock, 98 cold boot.
LYING = (1, 8, 98)
# Settled and commandable. 9 is transitional (8 -> 9 -> 1), so it is not here.
READY_STATES = (1, 6, 98)
# Settled but not armed: jy_exe eats the first command arming itself, so
# stand() sends its toggle again.
UNARMED = (8, 98)
# Seconds before "still lying" means the toggle was swallowed. Too short
# sends a second toggle at a robot that IS standing up, which lies him down.
ARM_GRACE = 2.0

BATTERY_REFUSE = 20
BATTERY_WARN = 30

# --- limits ----------------------------------------------------------------
MAX_SPEED = 0.6          # m/s
MAX_YAW_RATE = 1.6       # rad/s COMMANDED: he turns at about 0.7 of what is asked
HARD_TIMEOUT = 30.0      # s, ceiling on any single motion call

# Nearest thing on the side a turn in place allows: the body corner sweeps ~0.36 m.
TURN_SWEEP = 0.43

# He turns on for ~0.17 s after the zero command, so turn() stops that much
# early and reports the heading once he settles.
TURN_COAST_S = 0.17
TURN_SETTLE_S = 0.5

# Roll or pitch past this aborts any motion call. Raise it before trying a slope.
TILT_LIMIT_DEG = 30.0
# A handheld stick past this (axes are -1..1) means a person has taken over.
HANDHELD_DEADBAND = 0.1


def take_ready(executor, most=SPIN_BURST):
    """Run the callbacks that are ready now, at most `most`, and return. Not
    spin_once(timeout_sec=0) in a loop: each of those that finds nothing
    still pays for a whole wait, and the waits are what cost."""
    for _ in range(most):
        try:
            handler, _, _ = executor.wait_for_ready_callbacks(timeout_sec=0)
        except (TimeoutException, ShutdownException):
            return
        handler()


def _set_sigint(handler):
    """Install a SIGINT handler and return the one it replaced - or None off
    the main thread, where signals cannot be set."""
    try:
        return signal.signal(signal.SIGINT, handler)
    except ValueError:
        return None


class _Node(Node):
    def __init__(self):
        super().__init__('lite3_api')
        best = QoSProfile(depth=1)
        best.reliability = ReliabilityPolicy.BEST_EFFORT
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub = self.create_publisher(Twist, 'cmd_vel', best)
        self.state = None
        self.odom = None
        self.cloud = None
        self.grid = None
        self.us_front = self.us_rear = None
        self.tilt = None            # (roll, pitch) in degrees
        self.stick_time = 0.0       # last time a handheld stick was pushed
        self.create_subscription(Int32MultiArray, '/robot_state_debug',
                                 self._state_cb, 1)
        self.create_subscription(Odometry, 'leg_odom2', self._odom_cb, 1)
        # On a node of its own so it can be read at its own, slower, rate.
        self.cloud_node = Node('lite3_api_cloud')
        self.cloud_node.create_subscription(PointCloud2, '/camera/depth/color/points',
                                            lambda m: setattr(self, 'cloud', m), best)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda m: setattr(self, 'grid', m), latched)
        # The rear one bottoms out at 0.28 m, which lying down also means "no echo":
        # a reading, not a guard.
        self.create_subscription(Float64, '/us_publisher/ultrasound_distance',
                                 lambda m: setattr(self, 'us_rear', m.data), 1)
        self.create_subscription(Float64, '/us_publisher/ultrasound_front',
                                 lambda m: setattr(self, 'us_front', m.data), 1)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 1)
        self.create_subscription(Twist, '/handle_state', self._handle_cb, best)

    def _state_cb(self, m):
        self.state = dict(zip(STATE_FIELDS, m.data))

    def _odom_cb(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = (p.x, p.y, yaw)

    def _imu_cb(self, m):
        q = m.orientation
        roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                          1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        self.tilt = (math.degrees(roll), math.degrees(pitch))

    def _handle_cb(self, m):
        if max(abs(m.linear.x), abs(m.linear.y),
               abs(m.angular.z)) > HANDHELD_DEADBAND:
            self.stick_time = time.time()


class Lite3(Nav, Depth):
    """Live handle on the robot. Use as a context manager."""

    def __init__(self, auto_mode=True, timeout=10.0, estop_on_sigint=True):
        rclpy.init(args=None)
        self._node = _Node()
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self._node)
        self._cloud_exec = SingleThreadedExecutor()
        self._cloud_exec.add_node(self._node.cloud_node)
        self._stop_spin = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        self._closed = False
        self._nav_client = None
        self._hb_stop = threading.Event()
        self._hb_thread = None
        self._goal_handle = None
        self._prev_sigint = _set_sigint(self._on_sigint) if estop_on_sigint else None
        if not self._wait(lambda: self._node.state is not None, timeout):
            self.close()
            raise Lite3Error(
                'no /robot_state_debug within %.0fs. Is transfer_ros2.service '
                'running, and is Lite_motion stopped? (They fight over UDP '
                '43897.)' % timeout)
        if auto_mode:
            self.auto()

    # --- emergency stop ---
    # Ctrl-C alone is not a stop once Nav2 drives: controller_server is another
    # process publishing its own cmd_vel.
    def _on_sigint(self, signum, frame):
        print('\n*** Ctrl-C - EMERGENCY STOP ***')
        self.estop()
        raise KeyboardInterrupt

    def estop(self, disarm=False):
        """Stop as hard as software can: ignore SIGINT meanwhile, cancel the Nav2
        goal, halt, kill Nav2 (it would keep publishing), halt again.

        disarm=True also drops the heartbeat. UNTESTED while standing, so opt-in.
        """
        prev = _set_sigint(signal.SIG_IGN)
        try:
            try:
                self.goto_cancel()
            except Exception:
                pass
            self._goal_handle = None        # goto() stops waiting: Nav2 is about to die
            self.halt()
            try:
                self.nav_stop()
            except Exception:
                pass
            self.halt()
            if disarm:
                self.heartbeat_stop()
        finally:
            if prev is not None:
                _set_sigint(prev)
        return True

    # --- plumbing ----------------------------------------------------------
    def _spin(self):
        cloud_due = 0.0
        while not self._stop_spin.is_set():
            tick = time.time()
            take_ready(self._exec)
            if tick >= cloud_due:
                cloud_due = tick + 1.0 / CLOUD_HZ
                take_ready(self._cloud_exec, 1)
            self._stop_spin.wait(max(0.0, 1.0 / SPIN_HZ - (time.time() - tick)))

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
            _set_sigint(self._prev_sigint)
        # Halt BEFORE dropping the keepalive: what a standing robot does when it
        # disarms has never been observed.
        try:
            self.halt()
        except Exception:
            pass
        self.heartbeat_stop()
        # Order is load-bearing: the spin thread first (else segfault), then the
        # action client while the node lives (else InvalidHandle), then the rest.
        self._stop_spin.set()
        self._thread.join(timeout=5.0)
        if self._nav_client is not None:
            try:
                self._nav_client.destroy()
            except Exception:
                pass
            self._nav_client = None
        try:
            self._exec.shutdown(timeout_sec=2.0)
            self._exec.remove_node(self._node)
            self._cloud_exec.shutdown(timeout_sec=2.0)
            self._node.cloud_node.destroy_node()
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
        """dict of STATE_FIELDS: basic, gait, policy, motion, task, need_move,
        zero_flag, battery, and error and charging where the transfer build
        sends them."""
        return dict(self._node.state or {})

    @property
    def basic(self):
        """basic_state, the one that gates everything, or None."""
        return self.state.get('basic')

    @property
    def battery(self):
        """Percent, or None on an older transfer build that does not publish it."""
        return self.state.get('battery')

    @property
    def standing(self):
        return self.basic == STANDING

    @property
    def ultrasound(self):
        """(front, rear) ultrasonic range in metres, None where not heard."""
        return (self._node.us_front, self._node.us_rear)

    @property
    def attitude(self):
        """(roll, pitch) of the body in degrees from the robot's IMU, or None."""
        return self._node.tilt

    @property
    def pose(self):
        """(x, y, yaw) in the odom frame, or None."""
        return self._node.odom

    def wait_pose(self, timeout=5.0):
        if not self._wait(lambda: self._node.odom is not None, timeout):
            raise Lite3Error('no /leg_odom2 - refusing to move blind')
        return self._node.odom

    def pose_settled(self, window=1.5, tol=0.02):
        """True if odometry has not moved for `window` seconds. Standing up jumps
        the pose by over a metre, so anything building a world model waits for this.
        """
        self.wait_pose()
        a = self.pose
        time.sleep(window)
        b = self.pose
        return dist(a, b) <= tol

    # --- posture and mode --------------------------------------------------
    def heartbeat_start(self):
        """Emit the controller keepalive at 2 Hz until stopped.
        SAFETY: this replaces the handheld's role in the interlock - the robot
        will accept commands with no controller powered on."""
        if self.heartbeat_running:
            return
        self._hb_stop.clear()
        self._hb_thread = threading.Thread(target=self._hb_loop, daemon=True)
        self._hb_thread.start()

    def _hb_loop(self):
        while not self._hb_stop.is_set():
            try:
                P.send(P.HEARTBEAT)
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
        """Context manager: hold the body pitched nose up by pitch_deg (negative =
        nose down) while standing in place, and level it again on every way out.
        Don't walk inside it. settle: seconds to wait for the tilt before the block.
        """
        self._require_standing()
        frac = clamp(pitch_deg / MAX_TILT_DEG, 1.0)
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
    def upright(self, heartbeat=True):
        """Stand for the with-block and sit on the way out, whatever happens.
        heartbeat=True holds the interlock from here (no handheld needed) until
        close(), so he sits FIRST and is disarmed SECOND.
        """
        if heartbeat:
            # Before waiting: with no handheld he sits at 8, not ready, until a keepalive arrives.
            self.heartbeat_start()
            if not self.wait_ready():
                raise Lite3Error('interlock never came up - is transfer_ros2 '
                                 'running?')
        try:
            self.stand()
            yield self
        finally:
            self.sit()

    def wait_ready(self, timeout=10.0):
        """Block until basic_state is settled and commandable (READY_STATES). True
        if it is. Settled is not armed: stand() handles the toggle that arming
        eats, so no sleep is needed after this.
        """
        return self._wait(lambda: self.basic in READY_STATES, timeout)

    def auto(self):
        """Required before any velocity; nothing reports the mode, so just set it.
        Leaves posture mode first: in it he reports standing but ignores velocity.
        """
        P.send(P.POSTURE_EXIT)
        time.sleep(0.3)
        P.send(P.MODE_AUTO)
        time.sleep(0.5)

    def stand(self, timeout=20.0):
        """Stand up. Idempotent, though the underlying command is a toggle. From an
        unarmed state the first toggle only arms him, so it is sent once more.
        """
        if self.standing:
            return True
        basic = self.basic
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
            basic = self.basic
            if retry or basic not in LYING:
                break
            # Settled back in a lying state: the toggle armed him instead of
            # standing him. One retry, now that jy_exe is listening.
            print('NOTE: first stand toggle was consumed arming the robot '
                  '(basic_state=%s) - sending it again' % basic)
        raise Lite3Error(
            'timed out standing (basic_state=%s). Is the handheld powered ON, '
            'or heartbeat_start() running? With neither, jy_exe ignores every '
            'network command and basic_state sits at 8.' % self.basic)

    def _toggle_stand(self, timeout):
        """One stand toggle. True if standing; False as soon as he has settled back
        lying (the toggle was swallowed), without waiting out the timeout.
        """
        P.send(P.STAND_TOGGLE)
        sent = time.time()
        end = sent + timeout
        while time.time() < end:
            if self.standing:
                return True
            if time.time() - sent > ARM_GRACE and self.basic in LYING:
                return False
            time.sleep(0.05)
        return self.standing

    def sit(self, timeout=20.0):
        """Lie down. Idempotent."""
        if not self.standing:
            return True
        self.halt()
        P.send(P.STAND_TOGGLE)
        return self._wait(lambda: self.basic in LYING, timeout)

    def action(self, name, force=False):
        """Play a built-in trick from protocol.ACTIONS. Does NOT stand or sit for
        you. Sent 3 times at 1 Hz; returns when sent, not when the trick is over.
        """
        code, posture = ACTIONS[name]
        self._require_battery(force)
        if posture == 'stand':
            self._require_standing()
            self.halt()
        elif self.basic != 1:
            # 8/98 are unarmed: jy_exe would eat the first send arming itself
            # and the next one would land mid-transition.
            raise Lite3Error(
                '%s starts lying and armed (basic_state 1), not %s. '
                'stand() then sit() gets there.' % (name, self.basic))
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
                % (self.basic, STANDING))

    def _drive(self, vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self._node.pub.publish(t)

    def halt(self):
        """Publish a zero Twist, repeatedly. Best-effort QoS can drop one."""
        for _ in range(20):
            self._drive()
            time.sleep(0.02)

    def _safety(self, since):
        """A reason to stop every motion call, whatever it is doing."""
        if self._node.stick_time > since:
            return 'handheld took over - stopped'
        t = self._node.tilt
        if t and max(abs(t[0]), abs(t[1])) > TILT_LIMIT_DEG:
            return 'body tilted %.0f deg roll / %.0f deg pitch - stopped' % t
        return None

    def _loop(self, cycle, limit, force):
        """The one motion loop behind walk, strafe, turn and steer. cycle(start)
        drives for one 50 ms cycle and returns a reason to stop, or None. Guards on
        the way in, _safety() every cycle, a halt on every way out.
        """
        self._require_battery(force)
        self._require_standing()
        self.auto()
        self.wait_pose()
        start = self.pose
        began = time.time()
        deadline = began + min(limit, HARD_TIMEOUT)
        reason = 'time limit'
        try:
            while time.time() < deadline:
                stop = self._safety(began)
                if stop is None:
                    stop = cycle(start)
                if stop is not None:
                    reason = stop
                    break
        finally:
            self.halt()
            time.sleep(0.5)
        return {'reason': reason, 'start': start, 'end': self.pose}

    def _run(self, vx, vy, wz, done, limit, force, abort=None):
        """Drive at a fixed velocity until done(start, pose), the time limit,
        or abort() returns a reason."""
        def cycle(start):
            self._drive(vx, vy, wz)
            time.sleep(0.05)
            return (abort() if abort else None) or (
                'target reached' if done(start, self.pose) else None)

        return self._loop(cycle, limit, force)

    def steer(self, control, limit=HARD_TIMEOUT, force=False):
        """Drive with velocities recomputed every 50 ms: control() returns (vx, wz)
        to keep going or a string reason to stop. Same guards, clamps, time ceiling
        and final halt as walk() and turn().
        """
        def cycle(start):
            out = control()
            if isinstance(out, str):
                return out
            vx, wz = out
            self._drive(clamp(vx, MAX_SPEED), 0.0, clamp(wz, MAX_YAW_RATE))
            time.sleep(0.05)

        r = self._loop(cycle, limit, force)
        r['moved'] = dist(r['start'], r['end'])
        return r

    @staticmethod
    def _speed(speed):
        """A walking speed, checked: 0 would never arrive (and divides by
        zero in the time limit), and over MAX_SPEED is not a request to clamp
        quietly."""
        speed = abs(speed)
        if not 0 < speed <= MAX_SPEED:
            raise Lite3Error('speed must be in (0, %s] m/s' % MAX_SPEED)
        return speed

    def walk(self, distance, speed=0.15, stop_distance=0.6, force=False):
        """Walk `distance` metres (negative = backward). Blocking. Stops early for
        anything the depth camera sees within stop_distance (None = walk blind);
        the result's 'reason' says why it stopped. Backing up is always blind.
        """
        speed = self._speed(speed)
        vx = math.copysign(speed, distance)
        target = abs(distance)
        abort = None
        if stop_distance is not None:
            # Posture first: lying down, the camera sees floor and reports an obstacle.
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
                clear = sampled(self.clearance, here)

                def blocked():
                    try:
                        c = clear()
                    except Lite3Error:
                        return 'lost the depth stream - stopped rather than ' \
                               'walking blind'
                    return 'obstacle at %.2f m' % c if c <= stop_distance else None

                abort = blocked

        r = self._run(vx, 0.0, 0.0, lambda s, p: dist(s, p) >= target,
                      target / speed + 10.0, force, abort)
        r['moved'] = dist(r['start'], r['end'])
        return r

    def strafe(self, distance, speed=0.15, force=False):
        """Crab sideways. Positive is left."""
        speed = self._speed(speed)
        vy = math.copysign(speed, distance)
        target = abs(distance)
        return self._run(0.0, vy, 0.0, lambda s, p: dist(s, p) >= target,
                         target / speed + 10.0, force)

    def turn(self, radians, rate=0.4, force=False, guard=True):
        """Turn in place by `radians` relative to the current heading. Positive is
        left; any magnitude, beyond a full revolution too.
        """
        rate = min(abs(rate), MAX_YAW_RATE)
        if not rate > 0:
            raise Lite3Error('rate must be above 0 rad/s')
        wz = math.copysign(rate, radians)
        target = abs(radians)
        stop_at = target - min(rate * TURN_COAST_S, target / 2)
        acc = {'prev': None, 'total': 0.0}

        def done(s, p):
            if p is None:
                return False
            if acc['prev'] is None:
                acc['prev'] = s[2]
            # Wrap each INCREMENT, never the running total: the total saturates at pi
            # and a 180 degree turn never finishes.
            acc['total'] += wrap(p[2] - acc['prev'])
            acc['prev'] = p[2]
            return abs(acc['total']) >= stop_at

        abort = None
        if guard:
            if self._node.cloud is None:
                print('NOTE: no depth stream - turning without the side check')
            else:
                side = 0 if radians > 0 else 1          # left, right
                sides = sampled(self.side_clear)

                def blocked():
                    try:
                        near = sides()[side]
                    except Lite3Error:
                        return 'lost the depth stream - stopped turning'
                    if near < TURN_SWEEP:
                        return 'obstacle %.2f m to the %s - stopped turning' % (
                            near, ('left', 'right')[side])
                    return None

                abort = blocked

        r = self._run(0.0, 0.0, wz, done, target / rate + 10.0, force, abort)
        # Let the coast finish, then count it, so turned is where he settled.
        time.sleep(TURN_SETTLE_S)
        p = self._node.odom
        if p is not None and acc['prev'] is not None:
            done(None, p)
        r['turned'] = acc['total']          # unwrapped, so 180+ reads correctly
        return r

    def turn_deg(self, degrees, **kw):
        r = self.turn(math.radians(degrees), **kw)
        r['turned_deg'] = math.degrees(r['turned'])
        return r

    # --- voice ---
    # The speaker is on the motion computer; talk.py does the ssh. Imported
    # lazily so lite3 loads without it.
    @cached_property
    def voice(self):
        from .talk import Voice
        return Voice()

    def say(self, text, wait=True, voice=None, alien=False):
        """Speak `text` out of the robot, with Piper - see talk.Voice.say."""
        return self.voice.say(text, wait=wait, voice=voice, alien=alien)

    def play(self, name, wait=True):
        """Play one of the robot's built-in clips, e.g. play('okstop')."""
        return self.voice.play(name, wait=wait)

    # --- summary -----------------------------------------------------------
    def status(self):
        s = self.state
        p = self.pose
        f, r = self.ultrasound
        t = self.attitude
        return ('basic={basic} gait={gait} battery={battery}% '.format(**s)
                + ('error=%s ' % s['error'] if s.get('error') else '')
                + ('charging ' if s.get('charging') else '')
                + ('standing' if self.standing else 'not standing')
                + (' pose=(%.2f, %.2f, %.0f deg)' % (p[0], p[1], math.degrees(p[2]))
                   if p else ' pose=unknown')
                + ' nav2=' + ('up' if self.nav_running else 'down')
                + ' sonar front=%s rear=%s' % tuple(
                    '-' if v is None else '%.2f' % v for v in (f, r))
                + (' tilt=(%.0f, %.0f) deg' % t if t else ''))


def _cli():
    import sys
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        print('commands: status stand sit auto walk <m> turn <deg> scan '
              'nav-start nav-stop goto <m> cost estop [--disarm] action <%s>'
              % '|'.join(ACTIONS))
        return
    cmd = args[0]
    with Lite3(auto_mode=(cmd not in ('status', 'scan', 'cost'))) as bot:
        if cmd == 'status':
            time.sleep(0.5)     # let the sonar and IMU topics arrive
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
