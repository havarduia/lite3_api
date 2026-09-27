"""sonar_range - the two ultrasonic sensors as sensor_msgs/Range, for Nav2.

Jetson2Motion publishes them as bare Float64 at ~160 Hz (front =
/us_publisher/ultrasound_front, rear = /us_publisher/ultrasound_distance).
The costmap's RangeSensorLayer needs Range messages in a frame it can
transform, so this republishes them at 20 Hz on /sonar/front and
/sonar/rear, with static TFs base_link -> sonar_front / sonar_rear.

Started with Nav2 by env/start_nav2_mapless.sh, in the same process group,
so nav_stop() takes it down too.

Measured 2026-09-27/28 (tape and depth camera): readings are good to ~2 cm;
0.28 is its MINIMUM ("something within ~0.3 m"); 4.5 (sometimes ~4.7) means
no echo. A tilted surface can deflect the pulse and read far.

    python3 -m robot.sonar_range
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Float64
from tf2_ros import StaticTransformBroadcaster

# name: (input topic, x in base_link, yaw, max range). x from the
# camera-sonar offset (front face ~0.23 m ahead of base_link); the rear is not
# measured, so it is placed at the back of the body (nose is 0.274 m ahead,
# body 0.61 m long).
#
# Max range: a reading at or past it goes out AS max, which the range layer
# treats as "nothing there". The FRONT is cut to 0.8 m: its wide beam hears a
# chair base 30 deg off to the side, and the layer spreads every mark over
# the whole cone, so a far front echo becomes a wall across the aisle. The
# depth camera covers that distance properly; the front sonar is kept for
# what the camera misses up close (under ~0.6 m, glass). The REAR is the only
# sensor behind him, so it keeps its full range.
SONARS = {
    'front': ('/us_publisher/ultrasound_front', 0.23, 0.0, 0.8),
    'rear': ('/us_publisher/ultrasound_distance', -0.31, math.pi, 4.0),
}
MIN_RANGE = 0.27   # just under its 0.28 floor, so 0.28 still marks
# Beam width, measured 2026-09-28: objects ~1 m away at +-34 deg are caught
# intermittently (the edge), one at 27-32 deg steadily. 4.5 / 4.7 = no echo.
FOV = 1.15         # rad, ~+-33 deg
RATE_HZ = 20.0


class SonarRange(Node):
    def __init__(self):
        super().__init__('sonar_range')
        self._stamp = None           # latest /leg_odom2 stamp (steady clock)
        self._last = {}
        self._pubs = {}
        self.create_subscription(Odometry, 'leg_odom2', self._odom, 10)
        tfs = []
        for name, (topic, x, yaw, _) in SONARS.items():
            self._pubs[name] = self.create_publisher(Range, '/sonar/' + name, 10)
            self.create_subscription(
                Float64, topic, lambda m, n=name: self._last.__setitem__(n, m.data), 10)
            t = TransformStamped()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'sonar_' + name
            t.transform.translation.x = x
            t.transform.rotation.z = math.sin(yaw / 2)
            t.transform.rotation.w = math.cos(yaw / 2)
            tfs.append(t)
        self._static = StaticTransformBroadcaster(self)
        self._static.sendTransform(tfs)
        self.create_timer(1.0 / RATE_HZ, self._publish)

    def _odom(self, msg):
        # odom -> base_link is stamped with this (steady clock, via
        # odom_to_tf.py), so a Range carrying it always has a TF to match.
        self._stamp = msg.header.stamp

    def _publish(self):
        if self._stamp is None:
            return
        for name, value in self._last.items():
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
    try:
        rclpy.spin(SonarRange())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
