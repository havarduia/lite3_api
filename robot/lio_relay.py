"""lio_relay - FAST-LIO2's pose from the Orin, onto the robot's ROS graph.

    python3 -m robot.lio_relay          the relay (source env/lite3_env.sh first)
    python3 -m robot.lio_relay check    self-check of the maths, no ROS
    python3 -m robot.lio_relay start    start FAST-LIO2 on the Orin (also: stop, state)

Publishes /lio_odom (FAST-LIO2's pose as it comes) and /odom_fused
(/leg_odom2 with FAST-LIO2's correction; plain /leg_odom2 whenever the Orin
is silent or its pose jumps).

The Orin is Humble with Fast DDS on domain 42; the robot's nodes are
CycloneDDS on domain 0 and crash on meeting it. So a child process listens
there and pipes each pose to this one. README.md section 10.7.
"""
import collections
import json
import math
import os
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

LIO_TOPIC = '/Odometry'     # FAST-LIO2: pose of its `body` (the lidar's IMU) in `camera_init`
OUT_TOPIC = '/lio_odom'     # the same motion as odom -> base_link, shaped like /leg_odom2
LIO_DOMAIN = '42'
LIO_RMW = 'rmw_fastrtps_cpp'
# base_link -> lidar, as in the Orin's ~/bin/lidar_nav.launch.py. The pitch is
# measured (20.4 deg nose-down); x, y, z are that file's guesses.
MOUNT_XYZ = (0.20, 0.0, 0.10)
MOUNT_PITCH = 0.356
RETRY_S = 2.0
ORIN_PANEL = 'http://192.168.1.5:8000'      # the Orin's web panel; FAST-LIO2 is its Live run
LEG_TOPIC = 'leg_odom2'
FUSED_TOPIC = '/odom_fused'
LIO_DELAY_S = 0.05      # how old a FAST-LIO2 pose is on arrival; rough, from one walked loop
GAP_S = 0.5             # longer than this between poses: a new run, or it stalled
STEP_TOL = (0.15, 0.2)  # m, rad: a step differing more than this from the legs' is not believed


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def rot(q, v):
    return qmul(qmul(q, (v[0], v[1], v[2], 0.0)), qconj(q))[:3]


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


Q_MOUNT = (0.0, math.sin(MOUNT_PITCH / 2), 0.0, math.cos(MOUNT_PITCH / 2))


def to_base(p, q):
    """The lidar's pose since FAST-LIO started -> base_link's pose since then.
    ponytail: takes the body as level at that start (the Live tab asks for
    standing still); level it with /lio/gravity if runs start otherwise."""
    q_base = qmul(qmul(Q_MOUNT, q), qconj(Q_MOUNT))
    moved = rot(Q_MOUNT, p)
    lever = rot(q_base, MOUNT_XYZ)
    return tuple(MOUNT_XYZ[i] + moved[i] - lever[i] for i in range(3)), q_base


def twist(prev, now):
    """(vx, vy, wz) in base_link from two (t, x, y, yaw) samples."""
    dt = now[0] - prev[0]
    if dt <= 0:
        return 0.0, 0.0, 0.0
    dx, dy = now[1] - prev[1], now[2] - prev[2]
    c, s = math.cos(now[3]), math.sin(now[3])
    turn = math.atan2(math.sin(now[3] - prev[3]), math.cos(now[3] - prev[3]))
    return (c * dx + s * dy) / dt, (-s * dx + c * dy) / dt, turn / dt


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def join(a, b):
    """2D poses (x, y, yaw): b, given in frame a, in a's parent."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]


def inv(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return -c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2]


class Fuse:
    """Leg odometry corrected by FAST-LIO2. `fix` maps a leg pose to the fused
    one and only changes while FAST-LIO2's steps agree with the legs', so the
    fused pose never jumps: not on a divergence, a stall or a new run.
    ponytail: a slow drift in FAST-LIO2 passes; compare over seconds if it shows."""

    def __init__(self):
        self.fix = (0.0, 0.0, 0.0)
        self.anchor = None      # maps a FAST-LIO2 pose to the fused one
        self.prev = None        # (t, lio, leg) last taken
        self.using = False

    def lio(self, t, lio, leg):
        """A FAST-LIO2 pose and the leg pose of the same moment."""
        ok = False
        if self.prev and t - self.prev[0] < GAP_S:
            a, b = join(inv(self.prev[1]), lio), join(inv(self.prev[2]), leg)
            ok = (math.hypot(a[0] - b[0], a[1] - b[1]) < STEP_TOL[0]
                  and abs(wrap(a[2] - b[2])) < STEP_TOL[1])
        if not ok:              # the legs carried that stretch: go on from where they put us
            self.anchor = join(join(self.fix, leg), inv(lio))
        self.fix = join(join(self.anchor, lio), inv(leg))
        self.prev, self.using = (t, lio, leg), ok

    def pose(self, leg):
        x, y, yaw = join(self.fix, leg)
        return x, y, wrap(yaw)


def live(what, **options):
    """The Orin's FAST-LIO2 run: live('start'), live('stop', name='hall'),
    live('state'). Returns the panel's answer; start it with the robot
    standing still. A stop saves the run's map on the Orin."""
    data = None if what == 'state' else json.dumps(options).encode()
    req = urllib.request.Request('%s/api/live/%s' % (ORIN_PANEL, what), data,
                                 {'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:     # 409: it is already in that state
        return json.load(e)


def listen():
    """Child, on the Orin's DDS: one line per pose on stdout."""
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data

    def line(m):
        p, q, s = m.pose.pose.position, m.pose.pose.orientation, m.header.stamp
        print(s.sec + s.nanosec * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w, flush=True)

    rclpy.init()
    node = rclpy.create_node('lio_relay_listen')
    node.create_subscription(Odometry, LIO_TOPIC, line, qos_profile_sensor_data)
    parent = os.getppid()
    while os.getppid() == parent:           # no pose, no write: notice a dead parent here
        rclpy.spin_once(node, timeout_sec=1.0)


def main():
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.clock import Clock, ClockType

    rclpy.init()
    node = rclpy.create_node('lio_relay')
    lio_pub = node.create_publisher(Odometry, OUT_TOPIC, 10)
    fused_pub = node.create_publisher(Odometry, FUSED_TOPIC, 10)
    steady = Clock(clock_type=ClockType.STEADY_TIME)    # what /leg_odom2 is stamped with
    env = dict(os.environ, RMW_IMPLEMENTATION=LIO_RMW, ROS_DOMAIN_ID=LIO_DOMAIN)
    env.pop('CYCLONEDDS_URI', None)
    box = {}                # 'lio': the newest (arrival, stamp, position, quaternion)

    def read():
        while True:
            box['child'] = subprocess.Popen(
                [sys.executable, '-m', 'robot.lio_relay', 'listen'],
                env=env, stdout=subprocess.PIPE, universal_newlines=True)
            for text in box['child'].stdout:
                t, *v = map(float, text.split())
                box['lio'] = (time.monotonic(), t) + to_base(v[:3], v[3:])
            print('lio_relay: the listening end stopped; retrying', file=sys.stderr)
            time.sleep(RETRY_S)

    fuse = Fuse()
    legs = collections.deque(maxlen=50)     # (arrival, leg pose): about a second of them
    state = {'seen': None, 'prev': None, 'using': False}

    def take_lio(s):
        arrived, t, p, q = s
        now = (t, p[0], p[1], yaw_of(q))
        prev, state['prev'] = state['prev'], now
        # a gap is a new FAST-LIO run, with a new origin: no speed across it
        vx, vy, wz = twist(prev, now) if prev and t - prev[0] < 1.0 else (0.0, 0.0, 0.0)
        m = Odometry()
        m.header.stamp = steady.now().to_msg()
        m.header.frame_id, m.child_frame_id = 'odom', 'base_link'
        pos, ori = m.pose.pose.position, m.pose.pose.orientation
        pos.x, pos.y, pos.z = p
        ori.x, ori.y, ori.z, ori.w = q
        m.twist.twist.linear.x, m.twist.twist.linear.y = vx, vy
        m.twist.twist.angular.z = wz
        lio_pub.publish(m)
        then = arrived - LIO_DELAY_S
        fuse.lio(arrived, now[1:], min(legs, key=lambda g: abs(g[0] - then))[1])

    def leg(m):
        pos, ori = m.pose.pose.position, m.pose.pose.orientation
        q = (ori.x, ori.y, ori.z, ori.w)
        pose = (pos.x, pos.y, yaw_of(q))
        arrived = time.monotonic()
        legs.append((arrived, pose))
        s = box.get('lio')
        if s is not None and s is not state['seen']:
            state['seen'] = s
            take_lio(s)
        using = fuse.using and arrived - fuse.prev[0] < GAP_S
        if using != state['using']:
            state['using'] = using
            print('lio_relay: /odom_fused is', 'corrected by FAST-LIO2' if using
                  else 'leg odometry alone', file=sys.stderr)
        # the leg message itself, moved: its stamp, height, tilt and speeds stay
        m.header.frame_id, m.child_frame_id = 'odom', 'base_link'
        pos.x, pos.y, _ = fuse.pose(pose)
        half = fuse.fix[2] / 2
        ori.x, ori.y, ori.z, ori.w = qmul((0.0, 0.0, math.sin(half), math.cos(half)), q)
        fused_pub.publish(m)

    node.create_subscription(Odometry, LEG_TOPIC, leg, 1)
    threading.Thread(target=read, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if box.get('child'):
            box['child'].terminate()


def demo():
    """Self-check of the pose maths: no ROS, no robot."""
    def near(a, b):
        return all(abs(x - y) < 1e-9 for x, y in zip(a, b))

    ident = (0.0, 0.0, 0.0, 1.0)
    assert near(to_base((0, 0, 0), ident)[0], (0, 0, 0))
    # the lidar going 1 m along its own nose-down x is the body going forward and down
    p, q = to_base((1, 0, 0), ident)
    assert near(p, (math.cos(MOUNT_PITCH), 0, -math.sin(MOUNT_PITCH))) and near(q, ident)
    # a body pose turned into what FAST-LIO would report for it, and back
    q_body = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))      # yaw 90 deg
    p_body = (1.0, 2.0, 0.0)
    lever = rot(q_body, MOUNT_XYZ)
    p_lidar = rot(qconj(Q_MOUNT), [p_body[i] + lever[i] - MOUNT_XYZ[i] for i in range(3)])
    q_lidar = qmul(qmul(qconj(Q_MOUNT), q_body), Q_MOUNT)
    p, q = to_base(p_lidar, q_lidar)
    assert near(p, p_body) and near(q, q_body) and abs(yaw_of(q) - math.pi / 2) < 1e-9
    # facing +y, 0.05 m further along +y and 0.1 rad of turn in 0.1 s
    assert near(twist((0.0, 0, 0, math.pi / 2), (0.1, 0, 0.05, math.pi / 2 + 0.1)),
                (0.5 * math.cos(0.1), -0.5 * math.sin(0.1), 1.0))
    # fusing. The two odometries count from different places and directions.
    def run(fuse, steps, lio, leg, lio_step, leg_step, t=0.0):
        for _ in range(steps):
            lio, leg, t = join(lio, lio_step), join(leg, leg_step), t + 0.1
            fuse.lio(t, lio, leg)
        return lio, leg, t

    f = Fuse()
    lio, leg = (5.0, 5.0, math.pi / 2), (0.0, 0.0, 0.0)
    f.lio(0.0, lio, leg)
    # they agree: the fused pose is the leg pose
    lio, leg, t = run(f, 10, lio, leg, (0.1, 0, 0.02), (0.1, 0, 0.02))
    assert near(f.pose(leg), leg) and f.using
    # the legs under-count by a tenth: the fused pose goes FAST-LIO2's distance
    before = f.pose(leg)
    lio, leg, t = run(f, 10, lio, leg, (0.1, 0, 0), (0.09, 0, 0), t)
    after = f.pose(leg)
    assert abs(math.hypot(after[0] - before[0], after[1] - before[1]) - 1.0) < 1e-9
    # FAST-LIO2 shoots away: the fused pose does not, and follows the legs
    held = f.fix
    lio, leg, t = run(f, 3, lio, leg, (5.0, 0, 0), (0.1, 0, 0), t)
    assert near(f.fix, held) and not f.using
    # it comes back, and so does a new run from a new origin after a silence
    lio, leg, t = run(f, 2, lio, leg, (0.1, 0, 0), (0.1, 0, 0), t)
    assert f.using and near(f.fix, held)
    f.lio(t + 10, (0.0, 0.0, 0.0), leg)
    assert near(f.fix, held) and not f.using
    print('lio_relay ok')


if __name__ == '__main__':
    word = ''.join(sys.argv[1:])
    if word in ('start', 'stop', 'state'):
        print(json.dumps(live(word), indent=1))
    else:
        {'listen': listen, 'check': demo}.get(word, main)()
