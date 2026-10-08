"""lio_relay - FAST-LIO2's pose from the Orin, as /lio_odom on the robot's ROS graph.

    python3 -m robot.lio_relay          the relay (source env/lite3_env.sh first)
    python3 -m robot.lio_relay check    self-check of the pose maths, no ROS

The Orin is Humble with Fast DDS on domain 42; the robot's nodes are
CycloneDDS on domain 0 and crash on meeting it. So a child process listens
there and pipes each pose to this one. README.md section 10.7.
"""
import math
import os
import subprocess
import sys
import time

LIO_TOPIC = '/Odometry'     # FAST-LIO2: pose of its `body` (the lidar's IMU) in `camera_init`
OUT_TOPIC = '/lio_odom'     # the same motion as odom -> base_link, shaped like /leg_odom2
LIO_DOMAIN = '42'
LIO_RMW = 'rmw_fastrtps_cpp'
# base_link -> lidar, as in the Orin's ~/bin/lidar_nav.launch.py. The pitch is
# measured (20.4 deg nose-down); x, y, z are that file's guesses.
MOUNT_XYZ = (0.20, 0.0, 0.10)
MOUNT_PITCH = 0.356
RETRY_S = 2.0


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
    pub = node.create_publisher(Odometry, OUT_TOPIC, 10)
    steady = Clock(clock_type=ClockType.STEADY_TIME)    # what /leg_odom2 is stamped with
    env = dict(os.environ, RMW_IMPLEMENTATION=LIO_RMW, ROS_DOMAIN_ID=LIO_DOMAIN)
    env.pop('CYCLONEDDS_URI', None)
    child = None
    try:
        while True:
            child = subprocess.Popen([sys.executable, '-m', 'robot.lio_relay', 'listen'],
                                     env=env, stdout=subprocess.PIPE, universal_newlines=True)
            prev = None
            for text in child.stdout:
                t, *v = map(float, text.split())
                p, q = to_base(v[:3], v[3:])
                now = (t, p[0], p[1], yaw_of(q))
                # a gap is a new FAST-LIO run, with a new origin: no speed across it
                vx, vy, wz = twist(prev, now) if prev and t - prev[0] < 1.0 else (0.0, 0.0, 0.0)
                prev = now
                m = Odometry()
                m.header.stamp = steady.now().to_msg()
                m.header.frame_id, m.child_frame_id = 'odom', 'base_link'
                pos, ori = m.pose.pose.position, m.pose.pose.orientation
                pos.x, pos.y, pos.z = p
                ori.x, ori.y, ori.z, ori.w = q
                m.twist.twist.linear.x, m.twist.twist.linear.y = vx, vy
                m.twist.twist.angular.z = wz
                pub.publish(m)
            print('lio_relay: the listening end stopped; retrying', file=sys.stderr)
            time.sleep(RETRY_S)
    except KeyboardInterrupt:
        pass
    finally:
        if child:
            child.terminate()


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
    print('lio_relay ok')


if __name__ == '__main__':
    {'listen': listen, 'check': demo}.get(''.join(sys.argv[1:]), main)()
