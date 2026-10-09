"""locate - does the robot know where it is on the floor map?

    python3 -m robot.locate <floor>     the node (env/start_nav2_map.sh starts it with Nav2)
    python3 -m robot.locate check       self-check, no ROS

Publishes /localized twice a second: [state, x, y, yaw, spread_xy,
spread_yaw, fit], the pose being base_link on the map. One node works this
out so that every Lite3 (panel, command line, a script) reads the same
answer from one small topic.

Two things have to hold. AMCL's estimate must be narrow (its covariance),
and the lidar scan, drawn from that pose, must land on the map's walls.
The second is what catches the wrong floor and a robot that was carried:
AMCL settles on something there too. What it cannot catch is sliding along
a corridor, where every wall still fits.
"""
import collections
import math
import os
import sys
import time

import numpy as np
import yaml

from . import places
from .lio_relay import inv, join, wrap
from .protocol import Lite3Error

TOPIC = '/localized'
Reading = collections.namedtuple('Reading', 'state x y yaw spread_xy spread_yaw fit')
NO_POSE, LOCALIZED, SPREAD, NO_FIT, NO_SCAN, SETTLING = range(6)
WHY = {NO_POSE: 'it has not been told where it is',
       LOCALIZED: 'localized',
       SPREAD: 'its position estimate is too wide',
       NO_FIT: 'what the lidar sees does not fit the map here',
       NO_SCAN: 'no usable lidar scan',
       SETTLING: 'checking its position'}

PUBLISH_HZ = 2.0
READ_HZ = 10.0          # odometry is sampled at this, not at its 50 Hz: each message costs Python time
HISTORY_S = 2.0         # odometry remembered, to find where it was when a scan was taken
EDGE_S = 0.15           # a stamp this far outside what is remembered still gets the nearest pose
# Not tuned on a floor yet (plan task 10):
NEAR_M = 0.15           # a return this close to a mapped wall counts as on it
FIT_RANGE_M = 10.0      # further returns are left out: a small heading error moves them far
FIT_EVERY = 4           # every 4th beam, 180 of the 720
FIT_MIN = 0.7           # at least this share of the returns on walls. In an empty room on paper a pose
                        # 0.2 m or 3 deg off still passes, and another room sharing a corner scores 0.62
MIN_RETURNS = 30
SPREAD_XY_M = 0.5
SPREAD_YAW = 0.35       # rad
SCAN_OLD_S = 1.0
FOUND_S = 1.0           # it must look right this long before it is called localized
LOST_S = 2.0            # and wrong this long before it is called lost: a person walking past is not that


def read_pgm(path):
    """A binary PGM as rows of bytes, top row first."""
    with open(path, 'rb') as f:
        data = f.read()
    fields, i = [], 0
    try:
        while len(fields) < 4:
            while data[i:i + 1].isspace():
                i += 1
            if data[i:i + 1] == b'#':
                i = data.index(b'\n', i)
                continue
            j = i
            while not data[j:j + 1].isspace():
                j += 1
            fields.append(data[i:j])
            i = j
        w, h, top = int(fields[1]), int(fields[2]), int(fields[3])
        if fields[0] != b'P5' or top > 255:
            raise ValueError('not an 8-bit binary PGM')
        return np.frombuffer(data, np.uint8, w * h, i + 1).reshape(h, w)
    except (ValueError, IndexError) as e:
        raise Lite3Error('cannot read the map picture %s: %s' % (path, e))


def grow(occupied, cells):
    """Each occupied cell spread `cells` cells up, down, left and right."""
    out = occupied
    for _ in range(cells):
        g = out.copy()
        g[1:] |= out[:-1]
        g[:-1] |= out[1:]
        g[:, 1:] |= out[:, :-1]
        g[:, :-1] |= out[:, 1:]
        out = g
    return out


class Map:
    """A floor map in Nav2's format, as the cells within NEAR_M of a wall."""

    def __init__(self, yaml_path):
        with open(yaml_path) as f:
            info = yaml.safe_load(f)
        self.res = float(info['resolution'])
        self.x0, self.y0, turned = (float(v) for v in info['origin'])
        if turned:
            raise Lite3Error('the map %s is turned (origin yaw %s); only 0 is read' % (yaml_path, turned))
        picture = read_pgm(os.path.join(os.path.dirname(yaml_path), info['image']))
        dark = picture / 255.0 if info.get('negate') else (255 - picture.astype(np.float32)) / 255.0
        # the picture's top row is the map's far edge: row 0 here is y0
        self.near = grow((dark > float(info.get('occupied_thresh', 0.65)))[::-1].copy(),
                         int(round(NEAR_M / self.res)))

    def fit(self, pose, angle_min, angle_increment, ranges):
        """The share of a scan's returns that land on the map's walls, the scan
        taken at `pose` (x, y, yaw of base_link on the map). None if there
        are too few returns to say."""
        r = np.asarray(ranges, dtype=np.float32)[::FIT_EVERY]
        a = pose[2] + angle_min + angle_increment * FIT_EVERY * np.arange(len(r))
        ok = np.isfinite(r) & (r > 0) & (r < FIT_RANGE_M)
        if ok.sum() < MIN_RETURNS:
            return None
        r, a = r[ok], a[ok]
        cx = np.floor((pose[0] + r * np.cos(a) - self.x0) / self.res).astype(int)
        cy = np.floor((pose[1] + r * np.sin(a) - self.y0) / self.res).astype(int)
        h, w = self.near.shape
        on_map = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)     # off the map is a miss
        return float(self.near[cy[on_map], cx[on_map]].sum()) / len(r)


def spread(covariance):
    """(metres, radians) of spread from a pose's 36-number covariance."""
    return (math.sqrt(max(covariance[0], covariance[7], 0.0)), math.sqrt(max(covariance[35], 0.0)))


def pose_at(history, stamp):
    """Where odometry was at `stamp`, between the two samples either side of
    it. history: (stamp, (x, y, yaw)), oldest first. None if `stamp` is
    more than EDGE_S outside it."""
    if not history or stamp < history[0][0] - EDGE_S or stamp > history[-1][0] + EDGE_S:
        return None
    before = history[0]
    for after in history:
        if after[0] >= stamp:
            break
        before = after
    (t0, a), (t1, b) = before, after
    f = min(1.0, max(0.0, (stamp - t0) / (t1 - t0))) if t1 > t0 else 0.0
    return a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), wrap(a[2] + f * wrap(b[2] - a[2]))


def judge(have_pose, scan_age, spread_xy, spread_yaw, fit):
    """The state as it looks right now."""
    if not have_pose:
        return NO_POSE
    if scan_age is None or scan_age > SCAN_OLD_S or fit is None:
        return NO_SCAN
    if spread_xy > SPREAD_XY_M or spread_yaw > SPREAD_YAW:
        return SPREAD
    return LOCALIZED if fit >= FIT_MIN else NO_FIT


class Steady:
    """Localized or not, changing only once the new answer has held:
    FOUND_S to become yes, LOST_S to become no."""

    def __init__(self):
        self.yes = False
        self.since = None

    def update(self, now, looks):
        if looks == self.yes:
            self.since = None
        else:
            if self.since is None:
                self.since = now
            if now - self.since >= (FOUND_S if looks else LOST_S):
                self.yes, self.since = looks, None
        return self.yes


def main(floor):
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Float64MultiArray

    from .lite3 import ODOM_TOPIC, take_ready
    from .lio_relay import yaw_of

    floor_map = Map(os.path.join(places.floor_dir(floor), 'map.yaml'))
    rclpy.init()
    node = rclpy.create_node('locate')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    pub = node.create_publisher(Float64MultiArray, TOPIC, 1)
    new = {}        # the newest message of each kind, until it is looked at
    latched = QoSProfile(depth=1)
    latched.durability = DurabilityPolicy.TRANSIENT_LOCAL       # AMCL's last estimate, also if we start after it
    node.create_subscription(Odometry, ODOM_TOPIC, lambda m: new.__setitem__('odom', m), 1)
    node.create_subscription(LaserScan, '/scan', lambda m: new.__setitem__('scan', (time.monotonic(), m)), 1)
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: new.__setitem__('amcl', m), latched)

    def stamp(m):
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def flat(pose):
        q = pose.orientation
        return pose.position.x, pose.position.y, yaw_of((q.x, q.y, q.z, q.w))

    history = collections.deque(maxlen=int(HISTORY_S * READ_HZ))
    steady = Steady()
    fix = wide = scan = None    # odom -> map; AMCL's spread; the newest (arrival, scan)
    tick = 0
    try:
        while rclpy.ok():
            take_ready(executor)
            odom, amcl = new.pop('odom', None), new.pop('amcl', None)
            scan = new.pop('scan', scan)
            if odom is not None:
                history.append((stamp(odom), flat(odom.pose.pose)))
            if amcl is not None and history:
                # AMCL says where it was at its scan's time: pair it with the odometry
                # of that time. One from before we started gets the newest, and the
                # scan fit below is what finds out if it has moved since.
                then = pose_at(history, stamp(amcl)) or history[-1][1]
                fix = join(flat(amcl.pose.pose), inv(then))
                wide = spread(amcl.pose.covariance)
            tick += 1
            if tick % int(READ_HZ / PUBLISH_HZ) == 0:
                now = time.monotonic()
                pose, fit, age = (math.nan,) * 3, None, None
                if fix is not None and history:
                    x, y, yaw = join(fix, history[-1][1])
                    pose = (x, y, wrap(yaw))
                    if scan is not None:
                        age, s = now - scan[0], scan[1]
                        was = pose_at(history, stamp(s)) or history[-1][1]
                        fit = floor_map.fit(join(fix, was), s.angle_min, s.angle_increment, s.ranges)
                looks = judge(fix is not None, age, *(wide or (0.0, 0.0)), fit)
                state = LOCALIZED if steady.update(now, looks == LOCALIZED) else \
                    (SETTLING if looks == LOCALIZED else looks)
                pub.publish(Float64MultiArray(data=[float(state)] + list(pose) + list(wide or (math.nan, math.nan))
                                              + [math.nan if fit is None else fit]))
            time.sleep(1.0 / READ_HZ)
    except KeyboardInterrupt:
        pass


def demo():
    import tempfile

    def room(w, h, res=0.05, margin=1.0):
        """A map of an empty room, its corner at (0, 0), in a temporary folder."""
        cols, rows = int((w + 2 * margin) / res), int((h + 2 * margin) / res)
        px = np.full((rows, cols), 254, np.uint8)
        c0, c1, r0, r1 = int(margin / res), int((margin + w) / res), int(margin / res), int((margin + h) / res)
        px[r0, c0:c1 + 1] = px[r1, c0:c1 + 1] = 0
        px[r0:r1 + 1, c0] = px[r0:r1 + 1, c1] = 0
        folder = tempfile.mkdtemp()
        with open(os.path.join(folder, 'map.pgm'), 'wb') as f:
            f.write(b'P5\n# made for a check\n%d %d\n255\n' % (cols, rows) + px[::-1].tobytes())
        with open(os.path.join(folder, 'map.yaml'), 'w') as f:
            f.write('image: map.pgm\nresolution: %s\norigin: [%s, %s, 0.0]\nnegate: 0\n'
                    'occupied_thresh: 0.65\nfree_thresh: 0.15\n' % (res, -margin, -margin))
        return os.path.join(folder, 'map.yaml')

    def scan_in(w, h, pose, n=720):
        """What a lidar at `pose` in that room would measure."""
        out = []
        for i in range(n):
            a = pose[2] - math.pi + i * 2 * math.pi / n
            c, s = math.cos(a), math.sin(a)
            tx = ((w if c > 0 else 0.0) - pose[0]) / c if abs(c) > 1e-9 else math.inf
            ty = ((h if s > 0 else 0.0) - pose[1]) / s if abs(s) > 1e-9 else math.inf
            out.append(min(tx, ty))
        return -math.pi, 2 * math.pi / n, out

    m = Map(room(12.0, 9.0))
    assert m.res == 0.05 and (m.x0, m.y0) == (-1.0, -1.0) and m.near.shape == (220, 280)
    cell = lambda x, y: m.near[int((y - m.y0) / m.res), int((x - m.x0) / m.res)]     # noqa: E731
    assert cell(0.0, 4.0) and cell(0.1, 4.0) and not cell(0.3, 4.0) and not cell(6.0, 4.5)
    assert cell(6.0, 9.0) and not cell(6.0, 8.5)        # the far wall is at the far y: rows not upside down

    here = (4.0, 3.0, 0.5)
    scan = scan_in(12.0, 9.0, here)
    assert m.fit(here, *scan) > 0.95
    assert m.fit((here[0], here[1], here[2] + math.radians(15)), *scan) < 0.3     # turned
    assert m.fit((here[0] + 0.5, here[1] + 0.5, here[2]), *scan) < 0.2            # moved off both ways
    # another floor's room: nothing fits, unless two of its walls happen to lie on two of ours
    assert m.fit((6.0, 4.5, 0.5), *scan_in(8.0, 5.0, (4.0, 2.5, 0.5))) == 0.0
    assert 0.5 < m.fit((2.0, 2.0, 0.5), *scan_in(8.0, 5.0, (2.0, 2.0, 0.5))) < FIT_MIN
    # the blind spot: moved along one pair of walls, the other pair still fits
    assert 0.3 < m.fit((here[0] + 0.5, here[1], here[2]), *scan) < 0.7
    assert m.fit((500.0, 500.0, 0.0), *scan) == 0.0                               # off the map: all misses
    nothing = [math.inf] * 720
    assert m.fit(here, scan[0], scan[1], nothing) is None                         # nothing in range: cannot say
    assert m.fit(here, scan[0], scan[1], nothing[:400] + scan[2][400:]) > 0.95    # "nothing there" beams do not count

    assert spread([0.04] + [0] * 6 + [0.09] + [0] * 27 + [0.01]) == (0.3, 0.1)

    walk = collections.deque([(10.0 + i * 0.1, (i * 0.05, 0.0, 3.1 + i * 0.01)) for i in range(20)])
    x, y, yaw = pose_at(walk, 10.55)
    assert abs(x - 0.275) < 1e-9 and y == 0.0 and abs(wrap(yaw - 3.155)) < 1e-9
    assert abs(pose_at(walk, 11.5)[2] - wrap(3.25)) < 1e-9                        # across +-pi
    last = walk[-1][1]
    assert pose_at(walk, 12.0) == (last[0], last[1], wrap(last[2])) and pose_at(walk, 9.9) == walk[0][1]    # just outside: the nearest
    assert pose_at(walk, 12.5) is None and pose_at(walk, 9.0) is None and pose_at([], 1.0) is None

    # AMCL placed him at (5, 5, 90 deg) when odometry read (1, 0, 0); odometry then goes 2 m on
    fix = join((5.0, 5.0, math.pi / 2), inv((1.0, 0.0, 0.0)))
    x, y, yaw = join(fix, (3.0, 0.0, 0.0))
    assert abs(x - 5.0) < 1e-9 and abs(y - 7.0) < 1e-9 and abs(yaw - math.pi / 2) < 1e-9

    assert judge(False, 0.1, 0.1, 0.1, 0.9) == NO_POSE
    assert judge(True, None, 0.1, 0.1, None) == NO_SCAN and judge(True, 2.0, 0.1, 0.1, 0.9) == NO_SCAN
    assert judge(True, 0.1, 0.1, 0.1, None) == NO_SCAN
    assert judge(True, 0.1, 0.8, 0.1, 0.9) == SPREAD and judge(True, 0.1, 0.1, 0.5, 0.9) == SPREAD
    assert judge(True, 0.1, 0.1, 0.1, 0.65) == NO_FIT and judge(True, 0.1, 0.1, 0.1, 0.9) == LOCALIZED
    assert set(WHY) == set(range(6)) and len(Reading._fields) == 7

    s = Steady()
    assert not s.update(0.0, True) and not s.update(0.9, True) and s.update(1.0, True)    # held FOUND_S
    assert s.update(5.0, False) and s.update(6.9, False)                # someone walks past: still localized
    assert s.update(7.0, True) and s.update(8.0, False) and s.update(9.9, False)   # and the count starts again
    assert not s.update(10.0, False)                                    # wrong for LOST_S: lost
    assert not s.update(10.5, True) and not s.update(10.6, False) and not s.update(11.4, True)
    print('locate ok')


if __name__ == '__main__':
    if sys.argv[1:] == ['check']:
        demo()
    elif len(sys.argv) == 2:
        main(sys.argv[1])
    else:
        sys.exit(__doc__)
