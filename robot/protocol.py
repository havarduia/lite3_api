"""protocol - the Lite3's UDP codes and addresses, in one place. No ROS.

Every raw packet the package sends goes through here, so an address or a code
changes in one file. Import it from anything, including ROS-free scripts:

    from robot import protocol as P
    P.send(P.MODE_AUTO)

Or send one raw code from the shell - no ROS, and NO checks: stand_toggle
lies him down if he is already standing, and nothing checks posture before a
trick. Use Lite3 for anything but debugging and capturing unknown codes.

    python3 -m robot.protocol stand_toggle
    python3 -m robot.protocol hello            # a name from ACTIONS
    python3 -m robot.protocol 0x21010300       # any number
    python3 -m robot.protocol stick_pitch -16000
    python3 -m robot.protocol camera [on|off]   # camera services, no sudo

Codes marked OFFICIAL are in DeepRoboticsLab source; COMMUNITY ones come from
infodriver/lite3-sdk and have not been run on this robot. The rest were
captured from the handheld or phone app here.
"""
import json
import socket
import struct
import time

MOTION_IP = '192.168.1.120'          # 192.168.2.1 on the robot's own wifi
MOTION_ADDR = (MOTION_IP, 43893)     # jy_exe: every motion command
TRACKER_ADDR = (MOTION_IP, 43901)    # track service, JSON payloads
# jetson2app on THIS (perception) computer: starts/stops the camera services
APP_ADDR = ('127.0.0.1', 43899)
CAMERA_URL = 'rtsp://%s:8554/test' % MOTION_IP


class Lite3Error(RuntimeError):
    """Refused for a reason the caller can act on."""


# --- motion port, simple frames <code, value, 0> ----------------------------
STAND_TOGGLE = 0x21010202   # stand <-> lie, a TOGGLE
MODE_AUTO = 0x21010C03      # needed before any velocity
MODE_MANUAL = 0x21010C02    # the handheld sends this when a stick moves
ZERO = 0x31010C05           # reset joints to zero / init pose
HEARTBEAT = 0x21040001      # handheld keepalive, >= 2 Hz
# Posture ("twist body") mode, decoded from the phone app 2026-09-18. In it
# the app's sticks set the body attitude in place, values -32767..32767 sent
# at 10 Hz, 0 = level. STICK_PITCH full scale is ~14 deg and NEGATIVE is NOSE
# UP (checked with the camera, not just the IMU sign). STICK_ROLL is ~+-3 deg.
# Outside posture mode the same two codes are the move-mode vx / vy sticks.
POSTURE_ENTER, POSTURE_EXIT = 0x21010D05, 0x21010D06
STICK_PITCH, STICK_ROLL = 0x21010130, 0x21010131

# Built-in tricks: name -> (code, posture it starts from). hello/dance/twist
# are OFFICIAL (Lite3_LLM); the rest COMMUNITY. dance VERIFIED 2026-09-24
# (standing, 3 sends at 1 Hz); the others are not yet run on THIS robot.
# backflip (0x21010502) is left out on purpose: the Pro manual does not
# list it.
ACTIONS = {
    'hello':      (0x21010506, 'lie'),
    'dance':      (0x2101030C, 'stand'),
    'twist':      (0x21010204, 'stand'),
    'twist_jump': (0x2101020D, 'stand'),
    'long_jump':  (0x2101050B, 'lie'),     # manual: needs 2 m clear ahead
    'turn_over':  (0x21010205, 'lie'),     # rolls onto its back
}

# Known but unused, kept so nobody has to dig them up again:
#   0x21010C0B  stop/brake, value 0 then 1                       COMMUNITY
#   0x21010300 / 0x21010307 / 0x21010406  gait OR body height - sources
#       disagree, capture from the handheld before use
#   0x21010303  fast gait; 0x21010402 body up; 0x21010201 stance   COMMUNITY
#   0x0111 / 0x0113 / 0x0114  low-level joint control - BYPASSES the stock
#       controller, loses control after 5 ms without a 0x0111 frame  OFFICIAL
#   telemetry to UDP 43897: 0x0901 robot state, 0x0902 joints, 0x0905
#       handheld sticks. transfer_ros2 owns that port and republishes it.

# --- app port (APP_ADDR), simple frames ---------------------------------------
# The camera services (realsense_ros2 + voa_ros2), started the way the handheld
# does it - no sudo needed. Only works while the robot's IMU topic is up.
AI_SERVICES = 0x21012109    # value 0x40 start, 0x00 stop
AI_QUERY = 0x2101210D       # reply: same code, value 0x11 up / 0x10 down
CAMERA_ON, CAMERA_OFF = 0x40, 0x00

# --- tracker port, 12 B header <code, json_len, 1> then JSON -----------------
TRK_VIDEO = 0x21013301      # {"enabled":0|1}  video streaming
TRK_DETECT = 0x21013302     # {"enabled":0|1}  person detection
TRK_FOLLOW = 0x21013303     # {"targetID":n,"enabled":0|1}  built-in follow
TRK_TARGETS = 0x21013304    # <- {"targets":[{"id","following","bbox"}]} ~27/s
TRK_QUERY = 0x21013305      # {} -> 0x21013306 {"modes":{...}}


def send(code, value=0):
    """One simple frame to jy_exe. value is signed (stick axes go negative)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.sendto(struct.pack('<IiI', code, int(value), 0), MOTION_ADDR)
    finally:
        s.close()


def camera(on=None):
    """Start (True) or stop (False) the camera services; returns whether they
    are up afterwards, or None if jetson2app did not answer."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(('0.0.0.0', 0))
        s.settimeout(3)
        if on is not None:
            s.sendto(struct.pack('<IiI', AI_SERVICES,
                                 CAMERA_ON if on else CAMERA_OFF, 0), APP_ADDR)
            time.sleep(20 if on else 5)     # the services take ~15 s to come up
        s.sendto(struct.pack('<IiI', AI_QUERY, 0, 0), APP_ADDR)
        try:
            return struct.unpack('<IiI', s.recv(64)[:12])[1] == 0x11
        except socket.timeout:
            return None
    finally:
        s.close()


def tracker_packet(code, **fields):
    """A tracker frame. Send it from a bound socket: replies come back to it."""
    body = json.dumps(dict(timestamp=int(time.time() * 1000), sendIP='',
                           destIP='', **fields)).encode()
    return struct.pack('<iii', code, len(body), 1) + body


def lookup(name):
    """A code constant here (any case), an ACTIONS name, or a number."""
    if name.lower() in ACTIONS:
        return ACTIONS[name.lower()][0]
    code = globals().get(name.upper())
    return code if isinstance(code, int) else int(name, 0)


if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    if sys.argv[1] == 'camera':         # camera [on|off]
        up = camera({'on': True, 'off': False}.get(sys.argv[2] if len(sys.argv) > 2 else None))
        sys.exit(print('camera services:', {True: 'up', False: 'down',
                                             None: 'no answer from jetson2app'}[up]))
    code = lookup(sys.argv[1])
    value = int(sys.argv[2], 0) if len(sys.argv) > 2 else 0
    send(code, value)
    print('sent 0x%08X value=%d -> %s:%d' % ((code, value) + MOTION_ADDR))
