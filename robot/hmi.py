"""hmi - a web page to watch and drive the robot, on the tailnet only.

    python3 -m robot.hmi            # then open http://lite3-perception:8080

It holds one Lite3 with the heartbeat running, so stop it before running any
other script that drives the robot. A browser button is not a hardware stop:
keep the handheld at hand. The safety model (hold-to-move, the depth and
sonar stops, E-STOP, what happens when the page goes away) and the camera
plumbing are in README.md section 10.3.
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
from .depth import sampled
from .lite3 import BATTERY_REFUSE, BATTERY_WARN, LYING, Lite3
from .nav import REPO, clamp, status_text
from .protocol import Lite3Error

PORT = 8080
STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'hmi_static')

DEADMAN_S = 0.3        # drive halts this long after the last stick message
FORWARD_STOP_M = 0.6   # camera clearance (from body centre), as walk()
REAR_STOP_M = 0.5      # rear sonar reading
DRIVE_MAX_VX = 0.5     # m/s, the page's speed slider tops out here
DRIVE_MAX_WZ = 0.8     # rad/s
RS_LINGER_S = 10.0     # rs_stream.py keeps running this long after the last viewer
GONE_S = 3.0           # no page for this long: a Go is cancelled. The page reconnects in 1.5 s
LET_GO_S = 2.0         # Nav2 gets this long to act on that cancel before it is an E-STOP
PING_S = 5.0           # a page that has gone silent is noticed within about 1.5 x this

# What the page needs to know to draw and warn with the same numbers as here.
LIMITS = {'vx': DRIVE_MAX_VX, 'wz': DRIVE_MAX_WZ, 'stop': FORWARD_STOP_M,
          'rear': REAR_STOP_M, 'refuse': BATTERY_REFUSE, 'warn': BATTERY_WARN,
          'lying': LYING, 'gone': GONE_S}

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
                P.camera_mjpeg('-an', '-vf', 'scale=640:-2,fps=8', '-q:v', '7'),
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
    def log(self, msg, level=None):
        self.emit({'t': 'log', 'level': level, 'msg': msg})

    def warn(self, msg, every=3.0):
        """A warning, but at most once per `every` s: the drive pad sends 10
        messages a second, and each refusal used to log a line."""
        if time.time() - self._warned.get(msg, 0) >= every:
            self._warned[msg] = time.time()
            self.log(msg, 'warn')

    def submit(self, name, fn, *args):
        if self.busy:
            self.warn('busy with %s - %s ignored' % (self.busy, name))
            return
        self.estopped = False
        self.busy = name
        self.work.submit(self._run, name, fn, *args)

    def _run(self, name, fn, *args):
        self.log(name + ' ...')
        try:
            r = fn(*args)
            self.log('%s: %s' % (name, self._fmt(r)))
        except Exception as e:                   # Lite3Error and the rest
            self.log('%s failed: %s' % (name, e), 'error')
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
            self.log('E-STOP: halted, Nav2 killed', 'error')
        except Exception as e:
            self.log('E-STOP error: %s' % e, 'error')

    def cancel(self, why='from the panel'):
        """End a Go that is under way. Nav2 stays up, unlike E-STOP."""
        going = self.bot.goto_cancel()
        if going:
            self.log('Go cancelled ' + why, 'warn')
        return going

    def abandoned(self):
        """No page is open, so nobody can press E-STOP: a Go must not carry on
        alone. (Driving stops by itself already, DEADMAN_S.)"""
        if self.cancel('- no panel connected'):
            time.sleep(LET_GO_S)
            if self.bot.goto_cancel():          # still there: Nav2 has not let go
                self.estop()

    def goto(self, metres, degrees):
        if not self.bot.nav_running:
            self.log('starting Nav2 first ...')
            self.bot.nav_start()
            self.bot.cost_ahead(out_to=3.0, step=0.1, settle=10)
        return status_text(self.bot.goto(metres, heading_deg=degrees or None, timeout=90))

    def service(self, which, on):
        self.services[which] = getattr(P, which)(on)
        return '%s %s' % (which, 'up' if self.services[which] else 'down')

    # --- hold-to-drive ---------------------------------------------------
    def drive(self, vx, wz):
        vx, wz = clamp(float(vx), DRIVE_MAX_VX), clamp(float(wz), DRIVE_MAX_WZ)
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
        clearance = sampled(self.bot.clearance)

        def control():
            if self.estopped:
                return 'E-STOP'
            vx, wz, t = self.drive_cmd
            if time.time() - t > DEADMAN_S:
                return 'released'
            self.blocked = None
            if vx > 0:
                try:
                    clear = clearance()
                except Lite3Error:
                    return 'lost the depth stream'
                if clear <= FORWARD_STOP_M:
                    self.blocked = 'obstacle %.2f m ahead' % clear
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
            from .talk import Talker, PERSONA
            persona = persona or PERSONA
            if self.talker is None or self.persona != persona:
                if self.talker:
                    self.talker.close()
                self.talker = Talker(persona=persona)
                self.persona = persona
            said = lambda s: self.emit({'t': 'said', 'who': 'robot', 'text': s})
            if kind == 'say':               # in the persona's voice, like its replies
                self.talker.say(text, on_sentence=said)
            elif kind == 'ask':
                self.emit({'t': 'said', 'who': 'you', 'text': text})
                self.talker.ask(text, on_sentence=said)
            else:
                self.emit({'t': 'said', 'who': 'you', 'text': '(look) ' + (text or 'what do you see?')})
                self.talker.look(text or None, on_sentence=said)
        except Exception as e:
            self.log('%s failed: %s' % (kind, e), 'error')

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
            'limits': LIMITS,
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
        ws = web.WebSocketResponse(heartbeat=PING_S)
        await ws.prepare(request)
        self.pages.add(ws)
        try:
            from . import talk
            from .talk import PERSONAS, PERSONA
            personas, persona = sorted(PERSONAS), PERSONA
        except Exception as e:                  # talking unavailable, driving is not
            personas, persona = [], None
            self.robot.log('talk unavailable: %s' % e, 'warn')
        await ws.send_str(json.dumps({'t': 'hello', 'personas': personas,
                                      'persona': persona, 'log': self.log,
                                      'volume': talk.VOLUME if personas else 1.0}))
        try:
            async for m in ws:
                if m.type != WSMsgType.TEXT:
                    continue
                try:
                    await self.command(ws, json.loads(m.data))
                except (ValueError, TypeError, KeyError, AttributeError) as e:
                    # e.g. an emptied number box. Not worth the connection:
                    # losing it stops the drive and costs the page 1.5 s.
                    self.robot.warn('ignored a malformed message: %r' % e)
        finally:
            self.pages.discard(ws)
            self.rs_pages.discard(ws)
            self.rs_update()
            self.robot.drive_cmd = (0.0, 0.0, 0.0)  # a closed page never drives
            if not self.pages:
                self.loop.call_later(GONE_S, self.gone)
        return ws

    async def command(self, ws, c):
        r, k = self.robot, c.get('cmd')
        if k == 'estop':
            await self.loop.run_in_executor(None, r.estop)
        elif k == 'cancel':
            await self.loop.run_in_executor(None, r.cancel)
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

    def gone(self):
        if not self.pages:                      # nobody came back
            self.loop.run_in_executor(None, self.robot.abandoned)

    async def stream_front(self, request):
        """The front camera as MJPEG, for a page whose WebRTC did not connect."""
        frames = self.front
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
                    [sys.executable, '-m', 'robot.rs_stream'], cwd=REPO)
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
                self.robot.log('status: %s' % e, 'error')
            n += 1
            await asyncio.sleep(0.5)

    def main(self):
        host = self.host = tailscale_ip()
        self.loop = asyncio.get_event_loop()
        self.robot = Robot(self.emit)
        FrontCamera(self.front).start()
        app = web.Application()
        app.router.add_get('/', self.index)
        app.router.add_get('/ws', self.ws)
        app.router.add_get('/stream/front', self.stream_front)
        app.router.add_post('/whep/{cam}', self.whep)
        runner = web.AppRunner(app)
        self.loop.run_until_complete(runner.setup())
        self.loop.run_until_complete(web.TCPSite(runner, host, PORT).start())
        # localhost too, so `tailscale serve` can put HTTPS in front of it
        self.loop.run_until_complete(web.TCPSite(runner, '127.0.0.1', PORT).start())
        relay = subprocess.Popen(
            [sys.executable, '-m', 'robot.udp_relay', host, str(P.CAMERA_RTC_PORT),
             P.MOTION_IP, str(P.CAMERA_RTC_PORT)],
            cwd=REPO)
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
