"""hmi - a web page to watch and drive the robot, on the tailnet only.

    python3 -m robot.hmi            # then open http://lite3-perception:8080

It binds to this computer's Tailscale address (and localhost) only, so
nothing on eduroam or the robot's own Wi-Fi can reach it. For HTTPS, run
`sudo tailscale serve --bg 8080` once and open https://lite3-perception.<tailnet>.ts.net

Both cameras are shown over WebRTC from the motion computer's mediamtx,
through udp_relay.py: the front camera is the robot's own H.264 stream
(1280x720, 30 fps), the RealSense colour is put there by rs_stream.py, which
runs only while a page shows that view. The
page falls back to MJPEG if WebRTC does not connect. It holds one Lite3 with the heartbeat
running, so stop it before running any other script that drives the robot.

Safety model:
  - Driving is HOLD-TO-MOVE. The page sends the stick position ~10x/s while
    it is held; if nothing arrives for DEADMAN_S (released, tab closed, Wi-Fi
    gone) the drive loop halts. On top of that the robot's own ~1.5 s
    velocity failsafe applies.
  - Forward motion stops when the depth camera sees less than
    FORWARD_STOP_M ahead; backward motion stops when the rear sonar reads less
    than REAR_STOP_M. Turning in place is always allowed.
  - E-STOP bypasses the command queue and runs Lite3.estop() at once (cancels
    any Nav2 goal, halts, kills Nav2).
  - A browser button is not a hardware stop. Keep the handheld at hand.
  - On shutdown it sits him down before dropping the heartbeat.
"""
import asyncio
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

from aiohttp import ClientSession, web, WSMsgType

from . import protocol as P
from .lite3 import Lite3
from .protocol import Lite3Error

PORT = 8080
STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'hmi_static')
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # for -m robot.x

DEADMAN_S = 0.3        # drive halts this long after the last stick message
FORWARD_STOP_M = 0.6   # camera clearance (from body centre), as walk()
REAR_STOP_M = 0.5      # rear sonar reading
DRIVE_MAX_VX = 0.5     # m/s, the page's speed slider tops out here
DRIVE_MAX_WZ = 0.8     # rad/s
RS_FPS = 30            # RealSense colour view; the camera itself gives 30
RS_LINGER_S = 10.0     # rs_stream.py keeps running this long after the last viewer

# basic_state values seen on this robot (see project notes)
BASIC = {1: 'lying, ready', 6: 'standing', 8: 'not armed', 9: 'arming',
         98: 'fresh boot'}


def tailscale_ip():
    out = subprocess.run(['ip', '-4', '-o', 'addr', 'show', 'tailscale0'],
                         capture_output=True, text=True).stdout
    m = re.search(r'inet (\d+\.\d+\.\d+\.\d+)', out)
    if not m:
        raise SystemExit('no tailscale0 address - is tailscaled up?')
    return m.group(1)


class Frames:
    """Latest JPEG from one source, plus how many pages are watching it."""

    def __init__(self):
        self.jpeg, self.seq, self.viewers = None, 0, 0
        self.cond = threading.Condition()

    def put(self, jpeg):
        with self.cond:
            self.jpeg, self.seq = jpeg, self.seq + 1
            self.cond.notify_all()


class FrontCamera(threading.Thread):
    """The wide-angle front camera (RTSP on the motion computer) as JPEGs,
    via ffmpeg. Runs only while someone watches."""

    def __init__(self, frames):
        super().__init__(daemon=True)
        self.frames = frames

    def run(self):
        while True:
            if self.frames.viewers == 0:
                time.sleep(0.5)
                continue
            proc = subprocess.Popen(
                ['ffmpeg', '-loglevel', 'error', '-rtsp_transport', 'tcp',
                 '-i', P.CAMERA_URL, '-an', '-vf', 'scale=640:-2,fps=8',
                 '-q:v', '7', '-f', 'mjpeg', '-'],
                stdout=subprocess.PIPE, stdin=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL)
            buf = b''
            try:
                while self.frames.viewers > 0:
                    chunk = proc.stdout.read(65536)
                    if not chunk:
                        break
                    buf += chunk
                    while True:
                        a = buf.find(b'\xff\xd8')
                        b = buf.find(b'\xff\xd9', a + 2)
                        if a < 0 or b < 0:
                            break
                        self.frames.put(buf[a:b + 2])
                        buf = buf[b + 2:]
                    if len(buf) > 4_000_000:
                        buf = b''
            finally:
                proc.kill()
                proc.wait()
            time.sleep(1.0)


class RealSenseColour:
    """/camera/color/image_raw as JPEGs, encoded only while someone watches."""

    def __init__(self, frames):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        self.frames, self.last = frames, 0.0
        self.node = rclpy.create_node('hmi_camera')
        self.node.create_subscription(Image, '/camera/color/image_raw',
                                      self._cb, qos_profile_sensor_data)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        threading.Thread(target=self.ex.spin, daemon=True).start()

    def _cb(self, m):
        if self.frames.viewers == 0 or time.time() - self.last < 0.9 / RS_FPS:
            return
        import cv2
        import numpy as np
        self.last = time.time()
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        if m.encoding == 'rgb8':
            img = img[:, :, ::-1]
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            self.frames.put(jpg.tobytes())


class Robot:
    """Everything the page can ask of the robot. Blocking; called from threads."""

    def __init__(self, emit):
        self.emit = emit                        # emit(dict) -> all pages
        self.bot = Lite3(estop_on_sigint=False)
        self.bot.heartbeat_start()
        self.work = ThreadPoolExecutor(1)       # motion: one thing at a time
        self.talk_pool = ThreadPoolExecutor(1)  # talking runs beside motion
        self.busy = None                        # name of the running command
        self.estopped = False
        self.drive_cmd = (0.0, 0.0, 0.0)        # vx, wz, time received
        self.driving = False
        self.blocked = None                     # why driving is limited
        self.services = {'camera': None, 'voa': None}
        self.scan = None
        self._warned = {}                       # msg -> time, to stop log spam
        self.talker = None
        self.persona = None

    # --- commands --------------------------------------------------------
    def warn(self, msg, every=3.0):
        """A warning, but at most once per `every` s: the drive pad sends 10
        messages a second, and each refusal used to log a line."""
        if time.time() - self._warned.get(msg, 0) >= every:
            self._warned[msg] = time.time()
            self.emit({'t': 'log', 'level': 'warn', 'msg': msg})

    def submit(self, name, fn, *args):
        if self.busy:
            self.warn('busy with %s - %s ignored' % (self.busy, name))
            return
        self.estopped = False
        self.busy = name
        self.work.submit(self._run, name, fn, *args)

    def _run(self, name, fn, *args):
        self.emit({'t': 'log', 'msg': name + ' ...'})
        try:
            r = fn(*args)
            self.emit({'t': 'log', 'msg': '%s: %s' % (name, self._fmt(r))})
        except Exception as e:                   # Lite3Error and the rest
            self.emit({'t': 'log', 'level': 'error', 'msg': '%s failed: %s' % (name, e)})
        finally:
            self.busy = None

    @staticmethod
    def _fmt(r):
        if isinstance(r, dict) and 'reason' in r:
            return r['reason']
        if r is True or r is None:
            return 'done'
        return str(r)

    def estop(self):
        self.estopped = True
        self.drive_cmd = (0.0, 0.0, 0.0)
        try:
            self.bot.estop()
            self.emit({'t': 'log', 'level': 'error', 'msg': 'E-STOP: halted, Nav2 killed'})
        except Exception as e:
            self.emit({'t': 'log', 'level': 'error', 'msg': 'E-STOP error: %s' % e})

    def goto(self, metres, degrees):
        if not self.bot.nav_running:
            self.emit({'t': 'log', 'msg': 'starting Nav2 first ...'})
            self.bot.nav_start()
            self.bot.cost_ahead(out_to=3.0, step=0.1, settle=10)
        st = self.bot.goto(metres, heading_deg=degrees or None, timeout=90)
        return {4: 'arrived', 5: 'canceled', 6: 'aborted'}.get(st, 'status %s' % st)

    def service(self, which, on):
        getattr(P, which)(on)
        self.services[which] = getattr(P, which)()
        return '%s %s' % (which, 'up' if self.services[which] else 'down')

    # --- hold-to-drive ---------------------------------------------------
    def drive(self, vx, wz):
        vx = max(-DRIVE_MAX_VX, min(DRIVE_MAX_VX, float(vx)))
        wz = max(-DRIVE_MAX_WZ, min(DRIVE_MAX_WZ, float(wz)))
        self.drive_cmd = (vx, wz, time.time())
        if self.driving or (vx == 0 and wz == 0):
            return
        if self.busy:
            self.warn('busy with %s - not driving' % self.busy)
            return
        if self.bot.nav_running:
            self.warn('Nav2 is running - stop it before driving by hand')
            return
        self.driving = True
        self.submit('drive', self._drive_loop)

    def _drive_loop(self):
        guard = {'t': 0.0, 'clear': float('inf')}

        def control():
            if self.estopped:
                return 'E-STOP'
            vx, wz, t = self.drive_cmd
            if time.time() - t > DEADMAN_S:
                return 'released'
            self.blocked = None
            if vx > 0:
                if time.time() - guard['t'] > 0.25:     # clearance walks the cloud
                    guard['t'] = time.time()
                    try:
                        guard['clear'] = self.bot.clearance()
                    except Lite3Error:
                        return 'lost the depth stream'
                if guard['clear'] <= FORWARD_STOP_M:
                    self.blocked = 'obstacle %.2f m ahead' % guard['clear']
                    vx = 0.0
            elif vx < 0:
                rear = self.bot.ultrasound[1]
                if rear is not None and rear < REAR_STOP_M:
                    self.blocked = 'rear sonar %.2f m' % rear
                    vx = 0.0
            return vx, wz

        try:
            return self.bot.steer(control)
        finally:
            self.driving = False
            self.blocked = None

    # --- talking ---------------------------------------------------------
    def talk(self, kind, text, persona):
        self.talk_pool.submit(self._talk, kind, text, persona)

    def _talk(self, kind, text, persona):
        try:
            if kind == 'say':
                self.bot.say(text)
                self.emit({'t': 'said', 'who': 'robot', 'text': text})
                return
            from .talk import Talker, PERSONA
            persona = persona or PERSONA
            if self.talker is None or self.persona != persona:
                if self.talker:
                    self.talker.close()
                self.talker = Talker(persona=persona)
                self.persona = persona
            if kind == 'ask':
                self.emit({'t': 'said', 'who': 'you', 'text': text})
            else:
                self.emit({'t': 'said', 'who': 'you', 'text': '(look) ' + (text or 'what do you see?')})
            said = lambda s: self.emit({'t': 'said', 'who': 'robot', 'text': s})
            if kind == 'ask':
                self.talker.ask(text, on_sentence=said)
            else:
                self.talker.look(text or None, on_sentence=said)
        except Exception as e:
            self.emit({'t': 'log', 'level': 'error', 'msg': '%s failed: %s' % (kind, e)})

    # --- status ----------------------------------------------------------
    def status(self):
        b = self.bot
        s = b.state
        p = b.pose
        f, r = b.ultrasound
        return {
            't': 'status',
            'basic': s.get('basic'), 'basic_text': BASIC.get(s.get('basic'), '?'),
            'battery': s.get('battery'), 'standing': b.standing,
            'pose': [round(p[0], 2), round(p[1], 2), round(math.degrees(p[2]))] if p else None,
            'sonar': [f, r], 'tilt': b.attitude,
            'nav': b.nav_running, 'heartbeat': b.heartbeat_running,
            'camera': self.services['camera'], 'voa': self.services['voa'],
            'busy': self.busy, 'driving': self.driving, 'blocked': self.blocked,
            'estopped': self.estopped, 'scan': self.scan,
            'limits': {'vx': DRIVE_MAX_VX, 'wz': DRIVE_MAX_WZ},
        }

    def poll_slow(self):
        """systemd states and the depth scan, ~1 Hz from a thread."""
        self.services = {'camera': P.camera(), 'voa': P.voa()}
        try:
            self.scan = [[b, None if rr is None else round(rr, 2)]
                         for b, rr in self.bot.scan()] \
                if self.services['camera'] and self.bot._node.cloud is not None else None
        except Exception:
            self.scan = None

    def shutdown(self):
        try:
            self.estopped = True
            if self.bot.standing:
                self.bot.estop()
                self.bot.sit()
        finally:
            if self.talker:
                self.talker.close()
            self.bot.close()


# --- web -------------------------------------------------------------------
class Server:
    def __init__(self):
        self.pages = set()
        self.loop = None
        self.front = Frames()
        self.rs = Frames()
        self.robot = None
        self.log = []
        self.rs_pages = set()       # pages showing the RealSense view
        self.rs_proc = None         # rs_stream.py, running only while watched
        self.rs_stop = None         # pending stop (timer handle)

    def emit(self, msg):
        """Thread-safe broadcast to every open page."""
        if msg.get('t') in ('log', 'said'):
            msg['time'] = time.strftime('%H:%M:%S')
            self.log = (self.log + [msg])[-80:]
        if self.loop:
            self.loop.call_soon_threadsafe(asyncio.ensure_future, self._send_all(msg))

    async def _send_all(self, msg):
        data = json.dumps(msg)
        for ws in list(self.pages):
            try:
                await ws.send_str(data)
            except Exception:
                self.pages.discard(ws)

    async def index(self, request):
        return web.FileResponse(os.path.join(STATIC, 'index.html'))

    async def ws(self, request):
        ws = web.WebSocketResponse(heartbeat=5.0)
        await ws.prepare(request)
        self.pages.add(ws)
        try:
            from . import talk
            from .talk import PERSONAS, PERSONA
            personas, persona = sorted(PERSONAS), PERSONA
        except Exception as e:                  # talking unavailable, driving is not
            personas, persona = [], None
            self.emit({'t': 'log', 'level': 'warn', 'msg': 'talk unavailable: %s' % e})
        await ws.send_str(json.dumps({'t': 'hello', 'personas': personas,
                                      'persona': persona, 'log': self.log,
                                      'volume': talk.VOLUME if personas else 1.0}))
        r = self.robot
        try:
            async for m in ws:
                if m.type != WSMsgType.TEXT:
                    continue
                c = json.loads(m.data)
                k = c.get('cmd')
                if k == 'estop':
                    await self.loop.run_in_executor(None, r.estop)
                elif k == 'drive':
                    r.drive(c.get('vx', 0), c.get('wz', 0))
                elif k == 'stand':
                    r.submit('stand', r.bot.stand)
                elif k == 'sit':
                    r.submit('sit', r.bot.sit)
                elif k == 'nav':
                    r.submit('nav ' + ('start' if c.get('on') else 'stop'),
                             r.bot.nav_start if c.get('on') else r.bot.nav_stop)
                elif k == 'goto':
                    r.submit('goto %.1f m' % float(c['m']), r.goto,
                             float(c['m']), float(c.get('deg') or 0))
                elif k in ('camera', 'voa'):
                    r.submit('%s %s' % (k, 'on' if c.get('on') else 'off'),
                             r.service, k, bool(c.get('on')))
                elif k == 'watch' and c.get('cam') == 'realsense':
                    (self.rs_pages.add if c.get('on') else self.rs_pages.discard)(ws)
                    self.rs_update()
                elif k == 'volume':
                    from . import talk
                    talk.VOLUME = min(1.0, max(0.0, float(c['v'])))
                elif k in ('say', 'ask', 'look'):
                    r.talk(k, (c.get('text') or '').strip(), c.get('persona'))
        finally:
            self.pages.discard(ws)
            self.rs_pages.discard(ws)
            self.rs_update()
            r.drive_cmd = (0.0, 0.0, 0.0)       # a closed page never drives
        return ws

    def stream(self, frames):
        async def handler(request):
            resp = web.StreamResponse(headers={
                'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
                'Cache-Control': 'no-cache'})
            await resp.prepare(request)
            frames.viewers += 1
            seq = -1
            try:
                while True:
                    jpeg, seq = await self.loop.run_in_executor(None, self._next, frames, seq)
                    if jpeg is None:
                        continue
                    await resp.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                     b'Content-Length: %d\r\n\r\n' % len(jpeg) + jpeg + b'\r\n')
            except (ConnectionResetError, RuntimeError, asyncio.CancelledError):
                pass                             # the page went away
            finally:
                frames.viewers -= 1
            return resp
        return handler

    def rs_update(self):
        """Run rs_stream.py while some page shows the RealSense view (it costs
        about half a core). Stopping waits RS_LINGER_S, so a page that drops
        its websocket and reconnects keeps its video."""
        if self.rs_pages:
            if self.rs_stop:
                self.rs_stop.cancel()
                self.rs_stop = None
            if self.rs_proc is None or self.rs_proc.poll() is not None:
                self.rs_proc = subprocess.Popen(
                    [sys.executable, '-m', 'robot.rs_stream'], cwd=HERE)
        elif self.rs_proc and not self.rs_stop:
            self.rs_stop = self.loop.call_later(RS_LINGER_S, self.rs_kill)

    def rs_kill(self):
        self.rs_stop = None
        if self.rs_proc:
            self.rs_proc.terminate()
            self.rs_proc = None

    async def whep(self, request):
        """WebRTC signalling for a camera: pass the page's offer to mediamtx,
        and point the answer's video address at udp_relay."""
        url = {'front': P.CAMERA_WHEP, 'realsense': P.RS_WHEP}.get(request.match_info['cam'])
        if not url:
            return web.Response(status=404)
        offer, cam = await request.read(), request.match_info['cam']
        try:
            async with ClientSession() as s:
                # rs_stream.py has only just been started by the page's
                # 'watch': 404 until its first frames reach mediamtx (~3 s).
                for attempt in range(20 if cam == 'realsense' else 1):
                    async with s.post(url, data=offer,
                                      headers={'Content-Type': 'application/sdp'},
                                      timeout=5) as r:
                        status, sdp = r.status, await r.text()
                    if status != 404:
                        break
                    await asyncio.sleep(0.5)
                if status != 201:
                    return web.Response(status=502, text='mediamtx said %d' % status)
        except Exception as e:
            return web.Response(status=502, text='mediamtx unreachable: %s' % e)
        lines = [l for l in sdp.splitlines() if not l.startswith('a=candidate:')]
        at = lines.index('a=end-of-candidates') if 'a=end-of-candidates' in lines else len(lines)
        lines.insert(at, 'a=candidate:1 1 udp 2130706431 %s %d typ host'
                     % (self.host, P.CAMERA_RTC_PORT))
        return web.Response(status=201, text='\r\n'.join(lines) + '\r\n',
                            content_type='application/sdp')

    @staticmethod
    def _next(frames, seq):
        with frames.cond:
            frames.cond.wait_for(lambda: frames.seq != seq, timeout=2.0)
            return frames.jpeg, frames.seq

    async def ticker(self):
        n = 0
        while True:
            try:
                if n % 2 == 0:
                    await self.loop.run_in_executor(None, self.robot.poll_slow)
                await self._send_all(self.robot.status())
            except Exception as e:               # keep the status flowing
                self.emit({'t': 'log', 'level': 'error', 'msg': 'status: %s' % e})
            n += 1
            await asyncio.sleep(0.5)

    def main(self):
        host = self.host = tailscale_ip()
        self.loop = asyncio.get_event_loop()
        self.robot = Robot(self.emit)
        FrontCamera(self.front).start()
        RealSenseColour(self.rs)
        app = web.Application()
        app.router.add_get('/', self.index)
        app.router.add_get('/ws', self.ws)
        app.router.add_get('/stream/front', self.stream(self.front))
        app.router.add_get('/stream/realsense', self.stream(self.rs))
        app.router.add_post('/whep/{cam}', self.whep)
        runner = web.AppRunner(app)
        self.loop.run_until_complete(runner.setup())
        self.loop.run_until_complete(web.TCPSite(runner, host, PORT).start())
        # localhost too, so `tailscale serve` can put HTTPS in front of it
        self.loop.run_until_complete(web.TCPSite(runner, '127.0.0.1', PORT).start())
        relay = subprocess.Popen(
            [sys.executable, '-m', 'robot.udp_relay', host, str(P.CAMERA_RTC_PORT),
             P.MOTION_IP, str(P.CAMERA_RTC_PORT)],
            cwd=HERE)
        self.loop.create_task(self.ticker())
        for sig in (signal.SIGINT, signal.SIGTERM):
            self.loop.add_signal_handler(sig, self.loop.stop)
        print('HMI on http://%s:%d  (tailnet only) - Ctrl-C sits him and stops'
              % (host, PORT), flush=True)
        try:
            self.loop.run_forever()
        finally:
            relay.kill()
            self.rs_kill()
            print('shutting down: halting and sitting ...', flush=True)
            self.robot.shutdown()


if __name__ == '__main__':
    Server().main()
