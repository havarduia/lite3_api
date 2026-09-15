#!/usr/bin/env python3
"""Send a raw EthCommand to jy_exe on the motion computer.

Codes confirmed by packet capture of the handheld (192.168.2.99 -> 192.168.2.1:43893)
and from ~/sdk_lib/src/sender.cpp on the motion computer:

    0x21010202  stand/lie TOGGLE    (the app/handheld [Stand] button)
    0x21010C02  MANUAL mode         (handheld sends this when you move the joystick)
    0x21010C03  AUTO mode           (required for /cmd_vel to have any effect)
    0x31010C05  reset joints to zero / init pose
    0x21040001  handheld heartbeat, ~2 Hz

    ./robot_cmd.py stand
    ./robot_cmd.py auto
    ./robot_cmd.py 0x31010C05
"""
import socket
import struct
import sys

MOTION = ('192.168.1.120', 43893)
NAMED = {
    'stand': 0x21010202,   # toggle: stands if lying, lies down if standing
    'auto': 0x21010C03,
    'manual': 0x21010C02,
    'zero': 0x31010C05,
    'init': 0x31010C05,
}


def send(code, value=0):
    # EthCommand: uint32 code, uint32 value, uint32 (type:8 | count:24)
    pkt = struct.pack('<III', code, value, 0)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.sendto(pkt, MOTION)
    return pkt


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    arg = sys.argv[1].lower()
    code = NAMED.get(arg, None)
    if code is None:
        code = int(arg, 0)
    value = int(sys.argv[2], 0) if len(sys.argv) > 2 else 0
    pkt = send(code, value)
    print(f'sent 0x{code:08X} value={value} -> {MOTION[0]}:{MOTION[1]}  '
          f'[{pkt.hex(" ")}]')
