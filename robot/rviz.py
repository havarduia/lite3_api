"""rviz - show the live robot in rviz2 on the laptop.

    python3 -m robot.rviz [topic ...]        on the laptop (ROS2 Jazzy)

Copies /joint_states, /tf, /tf_static and any topics named from the robot to
the laptop over ssh, and starts robot_state_publisher (urdf/lite3.urdf) and
rviz2 there. Why ssh and not DDS, and what to expect: README section 10.6.

The robot end is this same file, started by the laptop end:
    python3 -m robot.rviz --send topic ...
"""
import json
import os
import struct
import subprocess
import sys
import tempfile
import time

ROBOT = 'ysc@lite3-perception'
REMOTE = 'source ~/robot/env/lite3_env.sh && cd ~/robot && exec python3 -m robot.rviz --send '
TOPICS = ['/joint_states', '/tf', '/tf_static']
MAX_HZ = 20.0           # per topic; latched topics are never dropped
RESCAN_S = 2.0          # look for topics that were not there yet (Nav2 started later)
ANNOUNCE = 255          # frame index that carries a topic announcement, not a message
HEAD = struct.Struct('<BI')
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


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
            if latched or now - last.get(i, 0.0) >= 1.0 / MAX_HZ:
                last[i] = now
                frame(i, data)
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


def show(extra):
    """Laptop end."""
    # The laptop's own DDS setup belongs to another robot network; stay on this machine.
    os.environ.pop('CYCLONEDDS_URI', None)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from rosidl_runtime_py.utilities import get_message

    # rviz wants whole mesh paths; the URDF as shipped has them relative.
    urdf = tempfile.NamedTemporaryFile('w', suffix='.urdf')
    urdf.write(open(os.path.join(ROOT, 'urdf', 'lite3.urdf')).read().replace(
        './meshes/', 'file://' + os.path.join(ROOT, 'urdf', 'meshes') + '/'))
    urdf.flush()
    topics = TOPICS + [t for t in extra if t not in TOPICS]
    procs = [
        subprocess.Popen(['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                          urdf.name],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
        subprocess.Popen(['rviz2', '-d', os.path.join(ROOT, 'env', 'lite3.rviz')],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL),
        subprocess.Popen(['ssh', '-o', 'BatchMode=yes', ROBOT, REMOTE + ' '.join(topics)],
                         stdout=subprocess.PIPE),
    ]
    rviz, ssh = procs[1], procs[2]
    rclpy.init()
    node = rclpy.create_node('rviz_relay')
    pubs = {}
    try:
        while rviz.poll() is None:
            i, n = HEAD.unpack(read(ssh.stdout, HEAD.size))
            data = read(ssh.stdout, n)
            if i != ANNOUNCE:
                if i in pubs:
                    pubs[i].publish(data)
                continue
            i, topic, kind, latched = json.loads(data)
            try:
                msg = get_message(kind)
            except (AttributeError, ModuleNotFoundError, ValueError):
                print('skipping %s: no %s on this laptop' % (topic, kind))
                continue
            qos = QoSProfile(depth=10)
            if latched:
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            pubs[i] = node.create_publisher(msg, topic, qos)
            print('relaying %s [%s]' % (topic, kind))
    except EOFError:
        print('the robot end stopped (is ~/robot pulled, and the robot reachable?)')
    except KeyboardInterrupt:
        pass
    finally:
        for p in procs:
            p.terminate()


if __name__ == '__main__':
    if sys.argv[1:2] == ['--send']:
        send(sys.argv[2:])
    else:
        show(sys.argv[1:])
