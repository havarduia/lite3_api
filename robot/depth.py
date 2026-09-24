"""depth - obstacle ranges from the RealSense point cloud. Mixed into Lite3.

    bot.scan()        [(bearing_deg, range_m or None), ...], +ve = LEFT
    bot.clearance()   nearest obstacle in the path ahead, metres
    bot.side_clear()  (left, right) nearest, for turning
"""
import math
import struct

from .protocol import Lite3Error

# --- camera mounting, from voa/launch/voa_launch.py -------------------------
# base_link -> camera_link  xyz 0.25489 0 0.07249  rpy 0 0.34907 0
# The 20 degree nose-down pitch is REAL. Assume the camera is level and the
# floor reads as a wall across the whole view at ~0.54 m.
CAM_PITCH, CAM_X, CAM_Z = 0.34907, 0.25489, 0.07249
STAND_HEIGHT = 0.33
FLOOR_MARGIN, CEILING = 0.08, 0.60
MIN_VALID, MAX_RANGE = 0.15, 4.0


class Depth:
    """Needs self._node.cloud and self._wait, both from Lite3."""

    def wait_cloud(self, timeout=10.0):
        if not self._wait(lambda: self._node.cloud is not None, timeout):
            raise Lite3Error(
                'no point cloud. Start the camera with: python3 -m '
                'robot.protocol camera on   (if it is up but silent, check '
                'journalctl -u realsense_ros2, and that the Jetson has all 6 '
                'cores: cat /sys/devices/system/cpu/online should say 0-5)')
        return self._node.cloud

    def scan(self, fov_deg=45, bin_deg=5):
        """Nearest obstacle range per bearing bin, in base_link.

        Returns [(bearing_deg, range_m or None), ...], bearing +ve = LEFT.
        Applies the real 20 degree nose-down camera pitch and drops the floor
        and anything above the robot's back.

        Convert range to LATERAL offset (range * sin(bearing)) before judging
        clearance: a wall reading 0.67 m at -10 deg is only 0.12 m off the
        centreline, well inside the 0.225 m half-width.
        """
        m = self.wait_cloud()
        f = {fd.name: fd.offset for fd in m.fields}
        if not {'x', 'y', 'z'} <= set(f):
            raise Lite3Error('cloud has no xyz fields')
        ox, oy, oz = f['x'], f['y'], f['z']
        step, data, n = m.point_step, m.data, m.width * m.height
        cos_p, sin_p = math.cos(CAM_PITCH), math.sin(CAM_PITCH)

        nbins = (2 * fov_deg) // bin_deg
        bins = [None] * nbins
        for i in range(0, n, 2):                 # every other point is plenty
            base = i * step
            if base + step > len(data):
                break
            xo = struct.unpack_from('<f', data, base + ox)[0]
            yo = struct.unpack_from('<f', data, base + oy)[0]
            zo = struct.unpack_from('<f', data, base + oz)[0]
            if not (zo == zo) or zo < MIN_VALID or zo > MAX_RANGE:
                continue
            xc, yc, zc = zo, -xo, -yo            # optical -> camera body
            xb = xc * cos_p + zc * sin_p + CAM_X
            yb = yc
            zb = -xc * sin_p + zc * cos_p + CAM_Z
            h = zb + STAND_HEIGHT
            if h < FLOOR_MARGIN or h > CEILING or xb < MIN_VALID:
                continue
            bearing = math.degrees(math.atan2(yb, xb))
            if abs(bearing) > fov_deg:
                continue
            idx = min(max(int((bearing + fov_deg) // bin_deg), 0), nbins - 1)
            r = math.hypot(xb, yb)
            if bins[idx] is None or r < bins[idx]:
                bins[idx] = r
        return [(-fov_deg + i * bin_deg + bin_deg / 2.0, b)
                for i, b in enumerate(bins)]

    def clearance(self, half_width=0.225, bins=None):
        """Nearest obstacle directly in the robot's path, in metres.

        Uses lateral offset, not raw range, so a wall off to one side does not
        read as an obstacle ahead. inf means nothing in the way. Pass `bins`
        from scan() to reuse one scan for several checks.
        """
        near = float('inf')
        for bearing, r in (bins if bins is not None else self.scan()):
            if r is None:
                continue
            if abs(r * math.sin(math.radians(bearing))) <= half_width:
                near = min(near, r * math.cos(math.radians(bearing)))
        return near

    def side_clear(self, bins=None):
        """(left, right): nearest obstacle range in each half of the depth
        view, inf if none. Compare with TURN_SWEEP before turning that way."""
        bins = bins if bins is not None else self.scan()
        left = min((r for b, r in bins if r is not None and b > 0), default=float('inf'))
        right = min((r for b, r in bins if r is not None and b < 0), default=float('inf'))
        return left, right

    def scan_text(self, fov_deg=45, bin_deg=5):
        rows = self.scan(fov_deg, bin_deg)
        out = ['bearing +ve = LEFT, bars scale to %.1f m' % MAX_RANGE, '']
        for bearing, r in reversed(rows):
            side = 'L' if bearing >= 0 else 'R'
            if r is None:
                bar, txt = '=' * 40, 'clear'
            else:
                bar = '#' * max(1, int(40 * r / MAX_RANGE))
                txt = '%.2f m' % r
            out.append('  %+5.1f %s  %-40s %s' % (bearing, side, bar, txt))
        out.append('')
        out.append('  clearance straight ahead: %.2f m' % self.clearance())
        return '\n'.join(out)
