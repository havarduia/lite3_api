"""sonar_range - the two ultrasonic sensors as sensor_msgs/Range, for Nav2.

Republishes Jetson2Motion's bare Float64 readings at 20 Hz on /sonar/front
and /sonar/rear, with static TFs from base_link. Started with Nav2 by
env/start_nav2_mapless.sh, in its process group. README.md section 6.2.

    python3 -m robot.sonar_range
"""
import math
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Float64
from tf2_ros import StaticTransformBroadcaster

from .lite3 import ODOM_TOPIC, take_ready

# name: (input topic, x in base_link, yaw, max range). A reading at or past
# max goes out AS max, which the range layer reads as "nothing there". The
# front is cut to 0.8 m: its wide beam turns a chair off to the side into a
# wall across the aisle. The rear is the only sensor behind him.
SONARS = {
    'front': ('/us_publisher/ultrasound_front', 0.23, 0.0, 0.8),
    'rear': ('/us_publisher/ultrasound_distance', -0.31, math.pi, 4.0),
}
MIN_RANGE = 0.27   # just under its 0.28 floor, so 0.28 still marks
# Beam width, measured 2026-09-28: objects ~1 m away at +-34 deg are caught
# intermittently (the edge), one at 27-32 deg steadily. 4.5 / 4.7 = no echo.
FOV = 1.15         # rad, ~+-33 deg
RATE_HZ = 20.0
# Written on the first publish. env/start_nav2_mapless.sh waits for it before
# launching Nav2, so the range layer never starts its no-readings clock
# before there are readings.
READY_FILE = '/tmp/sonar_range.ready'


class SonarRange(Node):
    def __init__(self):
        super().__init__('sonar_range')
        self._stamp = None           # latest odometry stamp (steady clock)
        self._last = {}
        self._pubs = {}
        self._ready = False
        # depth 1, read RATE_HZ times a second in main(): these come at 160 Hz
        # and taking every message cost most of a core
        self.create_subscription(Odometry, ODOM_TOPIC, self._odom, 1)
        tfs = []
        for name, (topic, x, yaw, _) in SONARS.items():
            self._pubs[name] = self.create_publisher(Range, '/sonar/' + name, 10)
            self.create_subscription(
                Float64, topic, lambda m, n=name: self._last.__setitem__(n, m.data), 1)
            t = TransformStamped()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'sonar_' + name
            t.transform.translation.x = x
            t.transform.rotation.z = math.sin(yaw / 2)
            t.transform.rotation.w = math.cos(yaw / 2)
            tfs.append(t)
        self._static = StaticTransformBroadcaster(self)
        self._static.sendTransform(tfs)

    def _odom(self, msg):
        # odom -> base_link is stamped with this (steady clock, via
        # odom_to_tf.py), so a Range carrying it always has a TF to match.
        self._stamp = msg.header.stamp

    def _publish(self):
        if self._stamp is None:
            return
        for name, value in self._last.items():
            if not self._ready:
                open(READY_FILE, 'w').close()
                self._ready = True
            r = Range()
            r.header.stamp = self._stamp
            r.header.frame_id = 'sonar_' + name
            r.radiation_type = Range.ULTRASOUND
            r.field_of_view = FOV
            top = SONARS[name][3]
            r.min_range, r.max_range = MIN_RANGE, top
            r.range = min(float(value), top)
            self._pubs[name].publish(r)


def main():
    rclpy.init()
    node = SonarRange()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    try:
        while rclpy.ok():
            take_ready(ex, 6)               # the three subscriptions, and a few to spare
            node._publish()
            time.sleep(1.0 / RATE_HZ)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
