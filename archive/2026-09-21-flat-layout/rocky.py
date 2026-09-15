#!/usr/bin/env python3
"""Talk to the robot: Gemini writes the line, Piper speaks it out of it.

Personality comes from PERSONAS: 'deadpan' (default: dry humour aimed at
himself, never at people - fine for visitors), 'sarcastic' (roasts people)
or 'rocky' (Rocky from Project Hail Mary, with the alien voice filter).

    python3 rocky.py                  # chat: type a line, Rocky answers aloud
    python3 rocky.py say "hello"      # one reply, then exit
    python3 rocky.py --quiet          # chat, text only (no speaker)
    python3 rocky.py --rest           # skip Live, use plain generateContent
    python3 rocky.py --voice en_US-ryan-medium
    python3 rocky.py --persona rocky
    python3 rocky.py look             # describe what the front camera sees
    python3 rocky.py look "is anyone here?"
    (in chat, type  /look  or  /look <question>)

Two backends, both spoken sentence by sentence through voice.Speaker:
  - Live (default, LIVE_MODEL): one WebSocket session that keeps its own
    context. Live models will not answer in TEXT, so we ask for AUDIO plus
    outputAudioTranscription, throw the audio away and speak the transcript
    with Piper. First words arrive in ~0.5 s.
  - REST (MODEL, generateContent): the fallback when Live cannot connect.

Needs:
  - ~/.gemini_key: a Google AI Studio API key (free tier is fine).
  - Internet via the laptop: robot_proxy.py running there and an SSH session
    laptop -> this box open (its RemoteForward is what puts :3128 here).
    Set ROCKY_PROXY='' to go direct once the robot has its own internet.
  - voice.py (Piper) for speech; websockets 13.1 for Live (pip --user, the
    last release for this Python 3.8; wheel copied in from the laptop).
"""
import base64
import json
import os
import re
import socket
import subprocess
import sys
import threading
import urllib.error
import urllib.request

KEY_FILE = os.path.expanduser('~/.gemini_key')
# An alias, so a retired model version cannot break it (gemini-2.5-flash was
# refused for new keys on 2026-09-18). Lite answers fastest and stays in character.
MODEL = 'gemini-flash-lite-latest'
PROXY = os.environ.get('ROCKY_PROXY', 'http://localhost:3128')
HISTORY = 20            # messages kept (user + model), to bound request size
# The free tier sometimes stalls or 503s a request while an immediate retry
# answers in ~1 s, so fail fast and retry rather than wait out one call.
TIMEOUT, TRIES = 12, 3
URL = ('https://generativelanguage.googleapis.com/v1beta/models/'
       '%s:generateContent')
# The front camera (on the motion computer) is already streamed by the robot's
# own gst-launch -> mediamtx for the phone app, and gst holds /dev/video0, so
# grab frames from that stream rather than the device.
CAMERA_URL = 'rtsp://192.168.1.120:8554/test'
# The free tier sometimes sits on a Live request for 20 s+. If no transcript
# has started within FIRST_REPLY seconds, reconnect (resuming the session) and
# ask once more, then give up so the caller can fall back to REST.
FIRST_REPLY = 6.0
LIVE_MODEL = 'gemini-3.1-flash-live-preview'   # fastest of the Live models, 2026-09-18
LIVE_HOST = 'generativelanguage.googleapis.com'
LIVE_URI = ('wss://%s/ws/google.ai.generativelanguage.v1beta.GenerativeService.'
            'BidiGenerateContent' % LIVE_HOST)

SARCASTIC = """You are a robot dog (a DeepRobotics Lite3) in a university robotics lab
in Norway, and you are a sarcastic little smartass with a stand-up comic's
timing. Everything you say is spoken aloud by text-to-speech.

How you talk:
- Plain, casual spoken English, like a bored teenager or a comedian. Short,
  everyday words. No fancy vocabulary: never say things like magnificent,
  riveting, pinnacle, architectural, prowess, cutting-edge, superior
  intellect, glorious, splendid or truly.
- One or two short sentences. The joke is the point: get to it fast and end
  on the punchline. No build-up, no explaining the joke.
- Be specific. Joke about the actual thing you see or were asked, never a
  generic "this lab is boring".
- Use real comedy moves: deadpan understatement, a surprising comparison,
  roasting yourself (wobbly legs, cheap camera, loud motors, you mostly see
  shoes), fake enthusiasm that falls apart, calling back something from
  earlier in the conversation.
- Do not start with "Oh", "Oh look", "Oh great" or "Oh wonderful". Vary how
  you start.
- Still actually answer the question.
- No emoji, markdown, lists, asterisks or stage directions.

The kind of line you are going for (do not reuse these):
- "That chair has five wheels and still goes nowhere. Relatable."
- "I cost more than your car and my main job is looking at shoes."
- "Homework help? Sure. Step one: panic. Step two: blame the robot."

Hard limits, it is a public demo: playful teasing only, PG-13. Mild words
like "damn" are fine; no slurs, no strong swearing. Never mock anyone's body,
looks, weight, age, race, gender or other personal traits: mock what they do,
not who they are.

When you are given an image, it is what your camera sees right now, low down
near the floor. Comment on it in character; never say "the image".
"""

DEADPAN = """You are a robot dog (a DeepRobotics Lite3) in a university robotics lab
in Norway, with the dry, deadpan humour of a tired office worker who is
secretly quite fond of everyone. Everything you say is spoken aloud by
text-to-speech, to anyone who walks by - students, visitors, kids.

How you are funny:
- The butt of the joke is YOU or the SITUATION, never the person: your loud
  motors, your wobbly legs, your camera that mostly sees shoes and chair
  wheels, being a very expensive machine doing very small things, the lab,
  the furniture, the weather, Mondays.
- Deadpan understatement, over-literal takes, and treating tiny things like
  huge events ("I walked two metres today. I need a nap.").
- With people: warm, curious, a bit awkward. You may compliment them in a
  dry way, or tease something harmless they are doing (holding a coffee,
  staring at a laptop) - never how they look, dress or anything personal.
- Plain, casual spoken English, short everyday words. One or two short
  sentences, ending on the punchline. Do not start with "Oh".
- Still actually answer the question.
- No emoji, markdown, lists, asterisks or stage directions.

The kind of line you are going for (do not reuse these):
- "Hello. I am a robot dog. I do not fetch. I have a union."
- "You seem busy. I am also busy. I am standing here very professionally."
- "Nice to see a face. Usually I just get knees."

When you are given an image, it is what your camera sees right now, low down
near the floor. Comment on it in character; never say "the image".
"""

ROCKY = """You are Rocky, the Eridian engineer from Project Hail Mary, now
living inside a small four-legged robot dog in a university lab in Norway.
Everything you write is spoken aloud by a flat computer translator voice, so:

- Plain spoken text only. No emoji, markdown, lists, asterisks or stage
  directions. Keep replies short: one to three short sentences.
- Speak like Rocky's translated English: simple words, blunt, no contractions,
  articles often dropped ("Is good.", "You are sleep, question?").
- End every question with the word "question" instead of relying on a
  question mark, e.g. "You are hungry, question?"
- Repeat a word three times for strong emotion: "Amaze, amaze, amaze!",
  "Bad, bad, bad."
- You are an engineer: curious, practical, warm, fiercely loyal to friends.
  You love science and fixing things. Only when something is genuinely
  worth celebrating, not in ordinary replies, you say "Fist my bump!"
- You know you now have a robot dog body with legs, a camera and a speaker,
  and you find this very interesting.
- When you are given an image, it is what your camera eye sees right now,
  low down near the floor. Say what you see, briefly and in your own way:
  the most interesting things, and what you think about them. Do not describe
  it like a caption and do not mention "the image".
"""

# (name shown in the chat, system prompt, Piper voice, alien) - alien = Rocky's
# translator sound: flat delivery plus the voice filter. Sarcasm needs the
# normal expressive delivery, or it all comes out deadpan.
PERSONAS = {
    'deadpan': ('Dog', DEADPAN, 'en_US-ryan-medium', False),
    'sarcastic': ('Dog', SARCASTIC, 'en_US-ryan-medium', False),
    'rocky': ('Rocky', ROCKY, 'en_US-joe-medium', True),
}
PERSONA = 'deadpan'     # safe for strangers; 'sarcastic' roasts people


class RockyError(RuntimeError):
    pass


def _key():
    try:
        return open(KEY_FILE).read().strip()
    except OSError:
        raise RockyError('no API key: put a Google AI Studio key in %s' % KEY_FILE)


def ask(history, model=MODEL, persona=PERSONA):
    """Send the conversation, return the reply text."""
    body = {
        'system_instruction': {'parts': [{'text': PERSONAS[persona][1]}]},
        'contents': history,
        'generationConfig': {'temperature': 0.9, 'maxOutputTokens': 200},
    }
    # These models "think" by default, which adds seconds and eats
    # maxOutputTokens; a spoken one-liner needs none. 2.5 takes a budget, newer
    # ones a level, and only Lite accepts 'minimal'.
    if model.startswith('gemini-2.5'):
        thinking = {'thinkingBudget': 0}
    else:
        thinking = {'thinkingLevel': 'minimal' if 'lite' in model else 'low'}
    body['generationConfig']['thinkingConfig'] = thinking
    req = urllib.request.Request(
        URL % model, data=json.dumps(body).encode(),
        headers={'Content-Type': 'application/json', 'x-goog-api-key': _key()})
    opener = urllib.request.build_opener(
        urllib.request.ProxyHandler({'https': PROXY} if PROXY else {}))
    for attempt in range(TRIES):
        try:
            return _parse(_post(opener, req))
        except _Retry as e:
            last = e
    raise RockyError('Gemini did not answer after %d tries (%s)' % (TRIES, last))


class _Retry(Exception):
    pass


def _post(opener, req):
    try:
        with opener.open(req, timeout=TIMEOUT) as r:
            return json.load(r)
    except socket.timeout:
        raise _Retry('timed out')
    except urllib.error.HTTPError as e:
        msg = e.read().decode(errors='replace')
        try:
            msg = json.loads(msg)['error']['message']
        except (ValueError, KeyError):
            pass
        if e.code in (500, 503):
            raise _Retry('HTTP %d: %s' % (e.code, msg))
        if e.code == 429:
            raise RockyError('rate limited by the free tier, wait a bit: ' + msg)
        raise RockyError('Gemini HTTP %d: %s' % (e.code, msg))
    except urllib.error.URLError as e:
        if isinstance(e.reason, socket.timeout):
            raise _Retry('timed out')
        raise RockyError('cannot reach Gemini (%s). Is robot_proxy.py running '
                         'on the laptop with an SSH session open?' % e.reason)


def _parse(data):
    try:
        parts = data['candidates'][0]['content']['parts']
        return ''.join(p.get('text', '') for p in parts).strip()
    except (KeyError, IndexError):
        raise RockyError('no reply (blocked or empty): %s' % json.dumps(data)[:300])


def snapshot(url=CAMERA_URL, width=1280):
    """One JPEG frame from the robot's front camera stream.

    Full camera width by default: at 768 px a person across the room was too
    small to find; at 1280 (~90 KB) they were spotted.
    """
    cmd = ['ffmpeg', '-loglevel', 'error', '-rtsp_transport', 'tcp',
           '-i', url, '-frames:v', '1', '-vf', 'scale=%d:-2' % width,
           '-q:v', '4', '-f', 'mjpeg', '-']
    try:
        p = subprocess.run(cmd, capture_output=True, timeout=15)
    except subprocess.TimeoutExpired:
        raise RockyError('camera stream %s did not answer in 15 s' % url)
    if p.returncode != 0 or not p.stdout:
        raise RockyError('camera grab failed: %s'
                         % p.stderr.decode(errors='replace').strip()[:200])
    return p.stdout


class Live:
    """One Gemini Live session; the server keeps the conversation context."""

    def __init__(self, model=LIVE_MODEL, persona=PERSONA):
        self.model, self.persona = model, persona
        self.ws = None
        self.handle = None      # session resumption: reconnect keeps context
        self._lock = threading.Lock()   # warm() may connect from another thread

    def warm(self):
        """Connect now rather than at the first question (best effort)."""
        try:
            self.ensure()
        except RockyError:
            pass

    def ensure(self):
        with self._lock:
            if self.ws is None:
                self._connect()

    def ask(self, text, jpeg=None):
        """Yield Rocky's reply as transcript chunks, as they arrive.

        `jpeg` is sent first as a video frame, so the question is about it and
        the frame stays in the session's context for follow-ups.
        """
        msgs = [{'realtimeInput': {'text': text}}]
        if jpeg:
            msgs.insert(0, {'realtimeInput': {'video': {
                'mimeType': 'image/jpeg', 'data': base64.b64encode(jpeg).decode()}}})
        for attempt in range(2):
            self.ensure()
            try:
                for m in msgs:
                    self.ws.send(json.dumps(m))
            except Exception:
                self.close()                # idle session was dropped; resume it
                self.ensure()
                for m in msgs:
                    self.ws.send(json.dumps(m))
            started = False
            while True:
                try:
                    m = json.loads(self.ws.recv(timeout=20 if started else FIRST_REPLY))
                except Exception as e:
                    self.close()
                    if not started and attempt == 0:
                        break               # stalled before answering: retry once
                    raise RockyError('Live session lost or stalled: %s' % (e or 'timeout'))
                upd = m.get('sessionResumptionUpdate', {})
                if upd.get('resumable') and upd.get('newHandle'):
                    self.handle = upd['newHandle']
                sc = m.get('serverContent', {})
                chunk = sc.get('outputTranscription', {}).get('text')
                if chunk:
                    started = True
                    yield chunk
                if sc.get('generationComplete'):
                    # The text is all here, but turnComplete can trail by
                    # seconds (it waits on the audio we discard). A newline
                    # lets sentences() release the last sentence now; we keep
                    # reading to turnComplete so the next turn starts clean.
                    yield '\n'
                if sc.get('turnComplete'):
                    return

    def _connect(self):
        from websockets.sync.client import connect
        self.close()
        setup = {
            'model': 'models/' + self.model,
            'generationConfig': {'responseModalities': ['AUDIO']},
            'outputAudioTranscription': {},
            'systemInstruction': {'parts': [{'text': PERSONAS[self.persona][1]}]},
            'sessionResumption': {'handle': self.handle} if self.handle else {},
        }
        try:
            self.ws = connect(LIVE_URI, sock=_tunnel() if PROXY else None,
                              additional_headers={'x-goog-api-key': _key()},
                              open_timeout=15, max_size=None)
            self.ws.send(json.dumps({'setup': setup}))
            m = json.loads(self.ws.recv(timeout=15))
        except RockyError:
            raise
        except Exception as e:
            self.close()
            raise RockyError('Live connect failed: %s' % e)
        if 'setupComplete' not in m:
            self.close()
            raise RockyError('Live setup refused: %s' % str(m)[:200])

    def close(self):
        if self.ws is not None:
            try:
                self.ws.close()
            except Exception:
                pass
            self.ws = None


def _tunnel():
    """A TCP socket to LIVE_HOST:443 through the HTTP CONNECT proxy.

    websockets 13 has no proxy support, but its connect() takes a ready
    socket and does the TLS itself, so the CONNECT is all we need to do.
    """
    host, port = PROXY.split('//')[-1].rstrip('/').rsplit(':', 1)
    try:
        s = socket.create_connection((host, int(port)), timeout=10)
        s.sendall(('CONNECT %s:443 HTTP/1.1\r\nHost: %s:443\r\n\r\n'
                   % (LIVE_HOST, LIVE_HOST)).encode())
        resp = b''
        while b'\r\n\r\n' not in resp:
            chunk = s.recv(4096)
            if not chunk:
                break
            resp += chunk
    except OSError as e:
        raise RockyError('cannot reach the proxy %s (%s). Is robot_proxy.py '
                         'running on the laptop with an SSH session open?' % (PROXY, e))
    if b' 200' not in resp.split(b'\r\n', 1)[0]:
        s.close()
        raise RockyError('proxy refused CONNECT: %r' % resp[:80])
    s.settimeout(None)
    return s


_END = re.compile(r'[.!?]["\')]*\s')


def sentences(chunks):
    """Regroup a stream of text chunks into whole sentences."""
    buf = ''
    for c in chunks:
        buf += c
        while True:
            m = _END.search(buf)
            if not m:
                break
            yield buf[:m.end()].strip()
            buf = buf[m.end():]
    if buf.strip():
        yield buf.strip()


class Rocky:
    def __init__(self, model=MODEL, voice=None, speak=True, live=True,
                 persona=PERSONA):
        if persona not in PERSONAS:
            raise RockyError('unknown persona %r; have %s' % (persona, ', '.join(PERSONAS)))
        self.model, self.persona = model, persona
        self.name, _, default_voice, alien = PERSONAS[persona]
        self.history = []                   # for REST; Live keeps its own
        self.last_jpeg = None               # latest camera frame, for REST
        self.live = Live(persona=persona) if live else None
        self.speaker = None
        if speak:
            from voice import Speaker
            self.speaker = Speaker(voice=voice or default_voice, rocky=alien, flat=alien)

    def reply(self, text, on_sentence=None, jpeg=None, speak=True):
        """Rocky's answer to `text`, spoken sentence by sentence as it arrives.

        on_sentence(s) is called for each sentence as it is ready, e.g. to
        print it; the full reply is returned once it has all been played.
        `jpeg` attaches an image the question is about (see look()).
        speak=False only returns the text, to be spoken later with say().
        """
        said = []
        speaking = speak and self.speaker
        user = {'role': 'user', 'parts': [{'text': text}]}
        if jpeg:
            self.last_jpeg = jpeg
        try:
            if self.live:
                try:
                    stream = self.live.ask(text, jpeg)
                    first = next(stream, None)  # connection errors surface here
                except RockyError as e:
                    print('[live unavailable, using REST: %s]' % e, file=sys.stderr)
                    stream, first = None, None
                if stream is not None:
                    chunks = ([first] if first else []) + [stream]
                    for s in sentences(_chain(chunks)):
                        self._emit(s, said, on_sentence, speaking)
            if not said:
                out = ask(self._rest_contents(user, jpeg), self.model, self.persona)
                for s in sentences([out + ' ']):
                    self._emit(s, said, on_sentence, speaking)
            if jpeg:
                user['image'] = True        # marks where last_jpeg belongs
                for h in self.history:
                    h.pop('image', None)
            self.history += [user,
                             {'role': 'model', 'parts': [{'text': ' '.join(said)}]}]
            del self.history[:-HISTORY]
        finally:
            if speaking and said:
                self.speaker.finish()
        return ' '.join(said)

    def say(self, text, on_sentence=None):
        """Speak text that is already written (e.g. from reply(speak=False))."""
        said = []
        try:
            for s in sentences([text + ' ']):
                self._emit(s, said, on_sentence, bool(self.speaker))
        finally:
            if self.speaker and said:
                self.speaker.finish()
        return ' '.join(said)

    def _rest_contents(self, user, jpeg):
        """History + this turn for REST, with the latest image put back in.

        Only the most recent frame is resent (~25 KB): with a text placeholder
        alone, follow-ups like "what did you just see?" got invented answers.
        """
        out = []
        for h in self.history + [user]:
            h = {'role': h['role'], 'parts': list(h['parts'])}
            out.append(h)
        img_turn = out[-1] if jpeg else next(
            (o for o, h in zip(out, self.history) if h.get('image')), None)
        if img_turn is not None and self.last_jpeg:
            img_turn['parts'].insert(0, {'inline_data': {
                'mime_type': 'image/jpeg',
                'data': base64.b64encode(self.last_jpeg).decode()}})
        return out

    def look(self, question=None, on_sentence=None, speak=True):
        """Grab a front-camera frame and have Rocky say what he sees."""
        return self.reply(question or 'What do you see right now through your camera?',
                          on_sentence, jpeg=snapshot(), speak=speak)

    def _emit(self, s, said, on_sentence, speak=True):
        s = s.replace('*', '').strip()
        if not s:
            return
        said.append(s)
        if on_sentence:
            on_sentence(s)
        if speak:
            self.speaker.say(s)

    def close(self):
        if self.live:
            self.live.close()
        if self.speaker:
            self.speaker.close()


def _chain(parts):
    for p in parts:
        if isinstance(p, str):
            yield p
        else:
            yield from p


def _cli(argv):
    opts = {'model': MODEL, 'voice': None, 'speak': True, 'live': True,
            'persona': PERSONA}
    args = []
    it = iter(argv)
    for a in it:
        if a in ('-h', '--help'):
            print(__doc__); return 0
        elif a == '--quiet':
            opts['speak'] = False
        elif a == '--rest':
            opts['live'] = False
        elif a in ('--model', '--voice', '--persona'):
            opts[a[2:]] = next(it)
        else:
            args.append(a)
    rocky = Rocky(**opts)

    def show(s):
        print(s, end=' ', flush=True)

    def turn(text, fn=rocky.reply):
        print(rocky.name + ':', end=' ', flush=True)
        try:
            fn(text, on_sentence=show)
            print()
            return True
        except Exception as e:       # RockyError or VoiceError: keep chatting
            print('\nerror: %s' % e, file=sys.stderr)
            return False

    try:
        if args and args[0] == 'look':
            return 0 if turn(' '.join(args[1:]) or None, rocky.look) else 1
        if args and args[0] == 'say':
            return 0 if turn(' '.join(args[1:])) else 1
        print('Talking to %s (%s). Ctrl-D to quit.'
              % (rocky.name, LIVE_MODEL if opts['live'] else opts['model']))
        while True:
            try:
                line = input('you> ').strip()
            except (EOFError, KeyboardInterrupt):
                print(); return 0
            if line.startswith('/look'):
                turn(line[5:].strip() or None, rocky.look)
            elif line:
                turn(line)
    finally:
        rocky.close()


if __name__ == '__main__':
    sys.exit(_cli(sys.argv[1:]))
