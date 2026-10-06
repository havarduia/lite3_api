"""udp_relay - pass WebRTC video between browsers on the tailnet and mediamtx.

    python3 -m robot.udp_relay <listen ip> <port> <upstream ip> <port>

mediamtx is on the motion computer, which a browser cannot reach, so the
page sends its WebRTC traffic here (see hmi.Server.whep). A process of its
own (hmi.py starts it): inside the HMI it was starved and dropped packets.
"""
import os
import select
import socket
import sys
import time

IDLE_S = 30.0           # forget a browser this long after its last packet
RCVBUF = 4 << 20        # asked for; the kernel caps it at net.core.rmem_max


def _sock():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RCVBUF)
    return s


def main(listen, upstream):
    parent = os.getppid()
    front = _sock()
    front.bind(listen)
    ups, last = {}, {}                      # browser addr -> socket / last seen
    while os.getppid() == parent:
        ready, _, _ = select.select([front] + list(ups.values()), [], [], 2.0)
        now = time.time()
        for s in ready:
            if s is front:
                data, addr = front.recvfrom(65536)
                if addr not in ups:
                    ups[addr] = _sock()
                    ups[addr].connect(upstream)
                last[addr] = now
                try:
                    ups[addr].send(data)
                except OSError:
                    pass                    # mediamtx restarting; ICE retries
            else:
                addr = next(a for a, u in ups.items() if u is s)
                try:
                    front.sendto(s.recv(65536), addr)
                except OSError:
                    pass
        for addr in [a for a, t in last.items() if now - t > IDLE_S]:
            ups.pop(addr).close()
            del last[addr]


def demo():
    """Self-check on localhost: a datagram goes up and its echo comes back."""
    import threading
    echo = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    echo.bind(('127.0.0.1', 0))
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(('127.0.0.1', 0))
    listen = ('127.0.0.1', probe.getsockname()[1] + 1)
    threading.Thread(target=main, args=(listen, echo.getsockname()), daemon=True).start()
    time.sleep(0.3)
    probe.settimeout(2.0)
    probe.sendto(b'ping', listen)
    data, src = echo.recvfrom(100)
    echo.sendto(data + b'-pong', src)
    assert probe.recvfrom(100) == (b'ping-pong', listen)
    print('udp_relay ok')


if __name__ == '__main__':
    if len(sys.argv) == 5:
        main((sys.argv[1], int(sys.argv[2])), (sys.argv[3], int(sys.argv[4])))
    else:
        demo()
