"""rviz - show the live robot in rviz2 on the laptop.

    python3 -m robot.rviz [sensors] [topic ...]      on the laptop (ROS2 Jazzy)
    python3 -m robot.rviz --record FILE [sensors] [topic ...]     the same, kept in FILE
    python3 -m robot.rviz --play FILE [speed]        FILE again, with no robot

Copies /joint_states, /tf, /tf_static, /odom_fused and any topics named from the robot to
the laptop over ssh, and starts robot_state_publisher (urdf/lite3.urdf) and
rviz2 there. Why ssh and not DDS, and what to expect: README section 10.6.

The robot end is this same file, started by the laptop end:
    python3 -m robot.rviz --send topic ...
"""
import json
import math
import os
import signal
import struct
import subprocess
import sys
import tempfile
import time
import urllib.request
import xml.etree.ElementTree as ET

ROBOT = 'ysc@lite3-perception'
REMOTE = 'source ~/robot/env/lite3_env.sh && cd ~/robot && exec python3 -m robot.rviz --send '
TOPICS = ['/joint_states', '/tf', '/tf_static', '/odom_fused']
ODOM = '/odom_fused'    # becomes odom -> base_link here, with the height from height()
# Leg geometry from urdf/lite3.urdf, for height(): hip to thigh sideways, thigh and shank
# lengths, and how far the meshes reach past the knee, the foot and under the torso.
HIP_Y, THIGH, SHANK = 0.09735, 0.2, 0.21012
KNEE_R, FOOT_R, BELLY = 0.03, 0.023, 0.054
MAX_HZ = 20.0           # per topic; latched topics are never dropped
BIG, BIG_HZ = 20000, 5.0    # messages over BIG bytes (clouds) go at BIG_HZ
SHARE = 4.0             # a topic waits this many times its last write before the next
GOAL, PERSON = '/intent/goal', '/intent/person'    # from the panel (hmi.py), drawn as markers here
# What `sensors` on the command line stands for: what he senses, and what he intends.
# The costmaps and paths only exist with Nav2 up, goal and person with the panel running.
SENSORS = ['/camera/depth/color/points', '/us_publisher/ultrasound_front',
           '/us_publisher/ultrasound_distance', '/global_costmap/costmap',
           '/local_costmap/costmap', '/plan', '/local_plan', GOAL, PERSON]
MARK_S = 1.5            # a goal or person marker fades this long after the panel last sent it (2 Hz)
RESCAN_S = 2.0          # look for topics that were not there yet (Nav2 started later)
RETRY_S = 2.0           # between attempts to reach the robot again
ANNOUNCE = 255          # frame index that carries a topic announcement, not a message
HEAD = struct.Struct('<BI')
REC = struct.Struct('<dBI')     # a recording: seconds since it began, then the frame as it came
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HD_DIR = os.path.join(ROOT, 'urdf', 'meshes_hd')
HD_URL = ('https://raw.githubusercontent.com/DeepRoboticsLab/Lite3_rl_training/HEAD/'
          'legged_gym/resources/robots/lite3/meshes/')
HD_MESHES = ['torso'] + [leg + part for leg in ('fl', 'fr', 'hl', 'hr')
                         for part in ('_hip', '_thigh', '_shank')]
CAM_URL = ('https://raw.githubusercontent.com/IntelRealSense/realsense-ros/ros2-master/'
           'realsense2_description/meshes/d435.dae')
# realsense2_description draws the body 17.5 mm right of camera_link (the left imager).
# The robot's camera_link is on the centre line, so here the body is drawn centred on it.
CAM_ORIGIN = {'xyz': '0.0043 0 0', 'rpy': '1.5708 0 1.5708'}


def send(topics):
    """Robot end: raw (undecoded) messages to stdout as index, length, bytes."""
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message

    out = sys.stdout.buffer
    rclpy.init()
    node = rclpy.create_node('rviz_relay')
    subs, last = {}, {}

    def frame(i, data):
        try:
            out.write(HEAD.pack(i, len(data)) + data)
            out.flush()
        except OSError:                 # the laptop end went away
            os._exit(0)

    def relay(i, latched):
        def cb(data):
            now = time.monotonic()
            if latched or now >= last.get(i, 0.0):
                frame(i, data)
                # On a slow link the write is what takes the time. Waiting SHARE times
                # that long keeps one big topic from starving the model's small ones.
                hz = BIG_HZ if len(data) > BIG else MAX_HZ
                last[i] = now + max(1.0 / hz, SHARE * (time.monotonic() - now))
        return cb

    def scan():
        for i, topic in enumerate(topics):
            if topic in subs:
                continue
            pubs = node.get_publishers_info_by_topic(topic)
            if not pubs:
                continue
            latched = any(p.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
                          for p in pubs)
            qos = QoSProfile(depth=10 if latched else 1)
            if latched:
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            else:
                qos.reliability = ReliabilityPolicy.BEST_EFFORT
            kind = pubs[0].topic_type
            frame(ANNOUNCE, json.dumps([i, topic, kind, latched]).encode())
            subs[topic] = node.create_subscription(
                get_message(kind), topic, relay(i, latched), qos, raw=True)

    scan()
    node.create_timer(RESCAN_S, scan)
    rclpy.spin(node)


def read(pipe, n):
    data = pipe.read(n)
    if len(data) < n:
        raise EOFError
    return data


def recorded(path, speed=1.0, sleep=time.sleep):
    """The frames of a recording, each at the moment it first came (or `speed` times as fast)."""
    with open(path, 'rb') as f:
        start = time.monotonic()
        while True:
            head = f.read(REC.size)
            if len(head) < REC.size:
                return                      # the end, or a last frame cut short by a kill
            t, i, n = REC.unpack(head)
            data = f.read(n)
            if len(data) < n:
                return
            sleep(max(0.0, t / speed - (time.monotonic() - start)))
            yield i, data


def fetch_hd():
    """The vendor's dense meshes and Intel's D435 into urdf/meshes_hd (73 MB, not in git).
    True if all there."""
    os.makedirs(HD_DIR, exist_ok=True)
    for url in [HD_URL + name + '.STL' for name in HD_MESHES] + [CAM_URL]:
        path = os.path.join(HD_DIR, os.path.basename(url))
        if os.path.exists(path):
            continue
        print('downloading %s (once)' % os.path.basename(url))
        try:
            urllib.request.urlretrieve(url, path + '.part')
        except OSError as e:
            print('no dense meshes (%s); using the small ones in urdf/meshes' % e)
            return False
        os.rename(path + '.part', path)
    return True


def model():
    """urdf/lite3.urdf with whole mesh paths (rviz wants them), dense meshes if we have them."""
    from .sonar_range import SONARS    # here, not at the top: the robot end does not need it
    hd = fetch_hd()
    root = ET.parse(os.path.join(ROOT, 'urdf', 'lite3.urdf')).getroot()
    for link in root.iter('link'):
        mesh = link.find('visual/geometry/mesh')
        if link.get('name') == 'camera_link':
            if hd:          # the real camera in place of the grey box
                vis = link.find('visual')
                vis.remove(vis.find('material'))
                vis.find('origin').attrib.update(CAM_ORIGIN)
                geom = vis.find('geometry')
                geom.remove(geom.find('box'))
                ET.SubElement(geom, 'mesh', filename='file://' + os.path.join(HD_DIR, 'd435.dae'))
            continue
        if mesh is None:
            continue
        if hd:      # one file per leg, already mirrored, same frames as the small ones
            mesh.attrib.pop('scale', None)
            mesh.set('filename', 'file://' + os.path.join(HD_DIR, link.get('name').lower() + '.STL'))
        else:
            mesh.set('filename', mesh.get('filename').replace(
                './meshes/', 'file://' + os.path.join(ROOT, 'urdf', 'meshes') + '/'))
    for name, (_, x, yaw, _) in SONARS.items():     # the frames robot/sonar_range.py sends
        ET.SubElement(root, 'link', name='sonar_' + name)
        joint = ET.SubElement(root, 'joint', name='base_to_sonar_' + name, type='fixed')
        ET.SubElement(joint, 'parent', link='base_link')
        ET.SubElement(joint, 'child', link='sonar_' + name)
        ET.SubElement(joint, 'origin', xyz='%s 0 0' % x, rpy='0 0 %s' % yaw)
    return ET.tostring(root, encoding='unicode')


def height(joints):
    """Body centre above the floor for these /joint_states angles: whatever part of the
    model reaches lowest (a foot, a knee or the belly) rests on the floor. Level body assumed."""
    low = -BELLY
    for leg, side in (('LF', 1), ('RF', -1), ('LB', 1), ('RB', -1)):
        a, b, c = (joints[leg + '_Joint' + n] for n in ('', '_1', '_2'))
        knee = -THIGH * math.cos(b)
        foot = knee - SHANK * math.cos(b + c)
        for z, r in ((knee, KNEE_R), (foot, FOOT_R)):
            low = min(low, side * HIP_Y * math.sin(a) + z * math.cos(a) - r)
    return -low


def show(extra, record=None, play=None, speed=1.0):
    """Laptop end. `record`: also keep every frame in that file. `play`: take the frames
    from that file, not from the robot."""
    # The laptop's own DDS setup belongs to another robot network; stay on this machine.
    os.environ.pop('CYCLONEDDS_URI', None)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState, Range
    from std_msgs.msg import Float64
    from .sonar_range import FOV, MIN_RANGE, SONARS
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import Marker

    urdf = tempfile.NamedTemporaryFile('w', suffix='.urdf')
    urdf.write(model())
    urdf.flush()
    extra = [t for arg in extra for t in (SENSORS if arg == 'sensors' else [arg])]
    topics = TOPICS + [t for t in extra if t not in TOPICS]
    procs = [
        # The program itself: `ros2 run` leaves it running when it is told to stop.
        subprocess.Popen([os.path.join(get_package_prefix('robot_state_publisher'), 'lib',
                                       'robot_state_publisher', 'robot_state_publisher'),
                          urdf.name],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
        subprocess.Popen(['rviz2', '-d', os.path.join(ROOT, 'env', 'lite3.rviz')],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
    ]
    rviz = procs[1]

    def connect():
        # Keepalive: a laptop that changes network leaves a dead connection that
        # ssh would otherwise sit on for many minutes.
        procs.append(subprocess.Popen(
            ['ssh', '-o', 'BatchMode=yes', '-o', 'ServerAliveInterval=3',
             '-o', 'ServerAliveCountMax=2', ROBOT, REMOTE + ' '.join(topics)],
            stdout=subprocess.PIPE))
        return procs[-1]

    def live():
        ssh = connect()
        while True:
            try:
                i, n = HEAD.unpack(read(ssh.stdout, HEAD.size))
                yield i, read(ssh.stdout, n)
            except EOFError:
                print('lost the robot end (is ~/robot pulled, the robot reachable?); retrying')
                ssh.terminate()
                procs.remove(ssh)
                time.sleep(RETRY_S)
                ssh = connect()

    tape = open(record, 'wb') if record else None
    began = time.monotonic()
    # Stop the same clean way however we are told to (a plain kill, or Ctrl-C when
    # started in the background, where the shell has it ignored).
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, signal.default_int_handler)
    rclpy.init()
    node = rclpy.create_node('rviz_relay')
    pubs, names = {}, {}
    tf_pub = node.create_publisher(TFMessage, '/tf', 10)
    sonar = {topic: (name, top, node.create_publisher(Range, '/sonar/' + name, 1))
             for name, (topic, _, _, top) in SONARS.items()}
    mark_pub = node.create_publisher(Marker, '/intent/markers', 5)
    high = stamp = None

    def odom_tf(data):
        # The robot's odom_to_tf.py does this too, when Nav2 runs, but with the
        # odometry's own height, which stays at standing height when the robot lies
        # down. So this one is always used, and the robot's is dropped (robot_tf).
        odom = deserialize_message(data, Odometry)
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id, t.child_frame_id = 'odom', 'base_link'
        p = odom.pose.pose.position
        t.transform.translation.x, t.transform.translation.y = p.x, p.y
        t.transform.translation.z = p.z if high is None else high
        t.transform.rotation = odom.pose.pose.orientation
        tf_pub.publish(TFMessage(transforms=[t]))
        return odom.header.stamp

    def sonar_range(topic, data):
        # What robot/sonar_range.py does on the robot, which only runs with Nav2.
        name, top, pub = sonar[topic]
        r = Range()
        r.header.stamp, r.header.frame_id = stamp, 'sonar_' + name
        r.radiation_type, r.field_of_view = Range.ULTRASOUND, FOV
        r.min_range, r.max_range = MIN_RANGE, top
        r.range = min(deserialize_message(data, Float64).data, top)
        pub.publish(r)

    def mark(topic, data):
        # As markers because they fade: a goal he has reached, or a person he no
        # longer sees, is gone from the view MARK_S later.
        m = Marker(ns=topic, action=Marker.ADD, lifetime=Duration(sec=int(MARK_S), nanosec=int(MARK_S % 1 * 1e9)))
        m.header.frame_id = 'odom'
        if topic == GOAL:                   # an arrow lying on the floor, pointing the way he will face
            m.type, m.pose = Marker.ARROW, deserialize_message(data, PoseStamped).pose
            m.pose.position.z = 0.03
            m.scale.x, m.scale.y, m.scale.z = 0.5, 0.08, 0.08
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.72, 0.1, 1.0
        else:                               # someone standing: a column of a person's size
            m.type, m.pose.position = Marker.CYLINDER, deserialize_message(data, PointStamped).point
            m.pose.position.z, m.pose.orientation.w = 0.9, 1.0
            m.scale.x, m.scale.y, m.scale.z = 0.45, 0.45, 1.8
            m.color.r, m.color.g, m.color.b, m.color.a = 0.3, 0.8, 1.0, 0.5
        mark_pub.publish(m)

    def robot_tf(data):
        keep = [t for t in deserialize_message(data, TFMessage).transforms
                if t.child_frame_id != 'base_link']
        if keep:
            tf_pub.publish(TFMessage(transforms=keep))

    try:
        for i, data in recorded(play, speed) if play else live():
            if rviz.poll() is not None:
                break
            if tape:
                tape.write(REC.pack(time.monotonic() - began, i, len(data)) + data)
            if i != ANNOUNCE:
                if i in pubs:
                    if names[i] == '/tf':
                        robot_tf(data)
                        continue
                    pubs[i].publish(data)
                    if names[i] in (ODOM, '/leg_odom2'):     # recordings from before 2026-10-08
                        stamp = odom_tf(data)
                    elif names[i] in sonar and stamp is not None:
                        sonar_range(names[i], data)
                    elif names[i] in (GOAL, PERSON):
                        mark(names[i], data)
                    elif names[i] == '/joint_states':
                        joints = deserialize_message(data, JointState)
                        high = height(dict(zip(joints.name, joints.position)))
                continue
            i, topic, kind, latched = json.loads(data)
            if i in pubs:               # announced again after a reconnect
                continue
            try:
                msg = get_message(kind)
            except (AttributeError, ModuleNotFoundError, ValueError):
                print('skipping %s: no %s on this laptop' % (topic, kind))
                continue
            qos = QoSProfile(depth=10)
            if latched:
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            pubs[i] = node.create_publisher(msg, topic, qos)
            names[i] = topic
            print('relaying %s [%s]' % (topic, kind))
        else:
            # Once only: the stamps are the robot's clock, and rviz ignores frames
            # older than ones it has, so a second pass would show a frozen robot.
            print('end of %s; close rviz when done, run again to see it again' % play)
            rviz.wait()
    except KeyboardInterrupt:
        pass
    finally:
        if tape:
            tape.close()
        for p in procs:
            p.terminate()


def check():
    """python3 -m robot.rviz --check: height() against the standing height the robot
    reports, and a recording read back."""
    def pose(a, b, c):
        return {leg + '_Joint' + n: v for leg in ('LF', 'RF', 'LB', 'RB')
                for n, v in (('', a), ('_1', b), ('_2', c))}
    # Standing, /leg_odom2 said 0.324 to the foot centre; the foot reaches FOOT_R below that.
    assert abs(height(pose(0.0, 0.67, -1.35)) - (0.324 + FOOT_R)) < 0.01
    assert BELLY <= height(pose(0.0, 1.2, -2.7)) < 0.15       # folded: near the floor
    # A recording comes back frame for frame, each waited for, and a cut-off tail is dropped.
    frames, waits = [(ANNOUNCE, b'[0]'), (0, b'abc'), (0, b'')], []
    with tempfile.NamedTemporaryFile() as f:
        for t, (i, data) in enumerate(frames):
            f.write(REC.pack(t * 10.0, i, len(data)) + data)
        f.write(REC.pack(30.0, 0, 99) + b'short')
        f.flush()
        assert list(recorded(f.name, speed=2.0, sleep=waits.append)) == frames
    assert [round(w) for w in waits] == [0, 5, 10]
    print('ok')


if __name__ == '__main__':
    if sys.argv[1:2] == ['--check']:
        check()
    elif sys.argv[1:2] == ['--send']:
        send(sys.argv[2:])
    elif sys.argv[1:2] == ['--record']:
        show(sys.argv[3:], record=sys.argv[2])
    elif sys.argv[1:2] == ['--play']:
        show([], play=sys.argv[2], speed=float(sys.argv[3]) if sys.argv[3:] else 1.0)
    else:
        show(sys.argv[1:])
