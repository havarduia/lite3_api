#!/usr/bin/env python3
"""talk - everything the robot says: its speaker, and the Gemini chat behind it.

The speaker is NOT on this computer (the Jetson). It hangs off an ES8388
codec on the MOTION computer (protocol.MOTION_IP), and "playing a sound" is
running aplay over there via ssh. Three layers, lowest first:

    Voice    one sound at a time: a built-in clip, a file, a URL, or text
             through Piper TTS
    Speaker  streaming Piper: speak sentence by sentence as text arrives
    Talker   Gemini writes the line, Speaker says it; look() adds a camera
             frame. Personality comes from PERSONAS: 'deadpan' (default: dry
             humour aimed at himself, fine for visitors), 'sarcastic' (roasts
             people) or 'rocky' (Rocky from Project Hail Mary, with the alien
             voice filter).

    from robot.talk import Voice, Talker
    Voice().play('OKstandup')                  # one of the robot's ~33 clips
    Voice().say('watch your step')             # plain text-to-speech
    with Talker() as t:
        t.ask('say hello to the lab')          # Gemini answers, out loud
        t.look('is anyone here?')              # ...about a camera frame

From the shell:

    python3 -m robot.talk                      # chat: type a line, he answers aloud
    python3 -m robot.talk ask "hello"          # one Gemini reply, then exit
    python3 -m robot.talk look ["question"]    # describe what the front camera sees
    python3 -m robot.talk say "hello there"    # speak exactly this text
    python3 -m robot.talk play OKstandup       # a built-in clip
    python3 -m robot.talk file ~/alarm.wav     # any local audio
    python3 -m robot.talk url <youtube url>    # a video's audio
    python3 -m robot.talk clips | voices       # list clips / Piper voices
  options: --persona NAME  --voice PIPER_VOICE  --alien  --quiet (chat, text
  only)  --rest (skip Gemini Live)  --model NAME
  (in chat, type  /look  or  /look <question>)

Two backends, both spoken sentence by sentence through Speaker:
  - Live (default, LIVE_MODEL): one WebSocket session that keeps its own
    context. Live models will not answer in TEXT, so we ask for AUDIO plus
    outputAudioTranscription, throw the audio away and speak the transcript
    with Piper. First words arrive in ~0.5 s.
  - REST (MODEL, generateContent): the fallback when Live cannot connect.

Needs:
  - Passwordless SSH from here to the motion computer. Already set up.
  - ffmpeg here, for format conversion. Present.
  - ysc in the `audio` group on the motion computer, since aplay goes to
    DEVICE directly. Done 2026-09-15.
  - Piper in ~/piper (installed 2026-09-18), else espeak-ng, for speech.
  - ~/.gemini_key: a Google AI Studio API key (free tier is fine), and
    websockets 13.1 for Live (pip --user, the last release for Python 3.8).
  - Internet. The robot has its own (eduroam via the motion computer), so
    requests go direct. If it ever has none again, run robot_proxy.py on the
    laptop with an SSH session open and set TALK_PROXY=http://localhost:3128.

Audio is streamed to the robot as raw S16LE 48 kHz stereo - the format its
clips already use and the codec is known to accept - so there is no wav
header travelling down the pipe to be misread.
"""
import base64
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import threading
import urllib.error
import urllib.request

from .protocol import CAMERA_URL, MOTION_IP

# Empty = direct. See "Internet" above.
PROXY = os.environ.get('TALK_PROXY', '')


class TalkError(RuntimeError):
    pass


# --- speaker ------------------------------------------------------------------
HOST = 'ysc@' + MOTION_IP          # motion computer, where the speaker is
CLIP_DIR = '/home/ysc/lite3_voice'  # DEEP's own voice clips live here
RATE, CHANNELS = 48000, 2           # the codec's known-good playback format
# The ES8388 amp unmutes when a stream starts and that takes a moment: audio
# beginning at sample zero is swallowed, leaving an audible pop and nothing
# else. Measured 2026-09-21 - a 1.1 s clip was completely inaudible, the same
# clip with this lead-in was clear. The robot's own clips carry their own
# silence, which is why play() always worked. Raise it if the first syllable
# is ever clipped.
LEAD_IN_MS = 800
# Straight to the ES8388. ysc's PulseAudio default is a null sink, so a
# plain `aplay` "succeeds" silently. Needs ysc in the audio group on .120.
DEVICE = 'plughw:0,0'

# play_url(): standalone aarch64 binaries copied in from the laptop (no pip
# or github from here). yt-dlp goes out via TALK_PROXY when one is set.
YTDLP = os.path.expanduser('~/.local/bin/yt-dlp')
DENO = os.path.expanduser('~/.local/bin/deno')    # yt-dlp's JS runtime for YouTube

# say(): Piper, the standalone aarch64 release copied in from the laptop
# (2026-09-18). ~/piper is not on PATH for non-interactive ssh, so it is found
# here by absolute path. Voices are ~/piper/<name>.onnx (+ .onnx.json).
PIPER = os.path.expanduser('~/piper/piper')
PIPER_DIR = os.path.expanduser('~/piper')
PIPER_VOICE = 'en_US-lessac-medium'
# Low noise = flat intonation and even timing: the "translation computer" sound.
PIPER_ARGS = ['--noise_scale', '0.1', '--noise_w', '0.1', '--length_scale', '1.1']
# alien=True: thin, slightly raised, faintly metallic small-speaker voice.
# Loudness for Speaker (2026-09-18, user asked for louder): the codec's PCM
# volume goes to max (192 = 0 dB, was 180 = -6 dB) each time the speaker
# opens, and speech gets a software gain with a limiter so peaks don't clip.
# Output 1/2 stay at 0 dB; pushing the amp past that on this small speaker
# distorts. Set SOFT_GAIN = 1.0 to undo the software part.
HW_PCM = 192
SOFT_GAIN = 1.6
ALIEN_FILTER = ('rubberband=pitch=1.12,highpass=f=300,lowpass=f=5000,'
                'aecho=0.8:0.6:12:0.2')



class Voice:
    """Sound out of the robot. One SSH hop per sound; nothing stays running."""

    def __init__(self, host=HOST, clip_dir=CLIP_DIR):
        self.host = host
        self.clip_dir = clip_dir
        # No interactive prompt: rely on the installed key, and fail fast if
        # it ever stops working rather than hang on a password ask.
        self._ssh = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', host]

    # --- the built-in clips ------------------------------------------------
    def clips(self):
        """Names (without .wav) of the clips already on the robot."""
        out = subprocess.run(self._ssh + ['ls', '-1', self.clip_dir],
                             capture_output=True, text=True)
        if out.returncode != 0:
            raise TalkError('could not list %s on %s: %s'
                             % (self.clip_dir, self.host, out.stderr.strip()))
        return sorted(f[:-4] for f in out.stdout.split() if f.endswith('.wav'))

    def play(self, name, wait=True):
        """Play one of the robot's own clips by name, e.g. play('okstop').

        Case-insensitive, and the .wav is optional. Plays entirely on the
        robot - no audio crosses the network - so it is the cheapest sound.
        """
        clips = self.clips()
        key = name[:-4] if name.lower().endswith('.wav') else name
        match = next((c for c in clips if c.lower() == key.lower()), None)
        if match is None:
            raise TalkError('no clip %r. Available: %s'
                             % (name, ', '.join(clips)))
        path = '%s/%s.wav' % (self.clip_dir, match)
        # -q so aplay does not print the format banner on every call.
        return self._run_remote(self._ssh + ['aplay', '-q', '-D', DEVICE, path], wait,
                                'play %s' % match)

    # --- arbitrary local audio ---------------------------------------------
    def play_file(self, path, wait=True):
        """Play any local audio file (wav/mp3/ogg/...) out of the robot.

        ffmpeg here transcodes it to the codec's exact format and streams the
        raw samples over ssh into aplay - no temp file on either machine, no
        wav header to be misread on a pipe. Needs ffmpeg on THIS machine.
        """
        return self._stream(self._ffmpeg_cmd(path), wait, 'play_file %s' % path)

    # --- speech ------------------------------------------------------------
    def say(self, text, wait=True, engine=None, voice=None, alien=False):
        """Speak `text` out of the robot, using a TTS engine on THIS machine.

        There is deliberately no fallback voice baked in: a wrong-but-silent
        beep would be worse than a clear error. The lookup order is Piper
        (good neural voice, ~/piper, flattened by PIPER_ARGS) then espeak-ng
        (robotic but real). Piper runs at about real time on this CPU, so a
        sentence takes a few seconds before it starts playing.

        `engine` forces one ('piper' or 'espeak-ng') instead of autodetecting.
        `voice` picks a Piper model by name (see voices()); default PIPER_VOICE.
        `alien=True` adds ALIEN_FILTER on top.
        """
        gen = self._tts_cmd(text, engine, voice)
        # TTS engine -> wav on stdout -> ffmpeg reformats -> ssh -> aplay.
        af = ALIEN_FILTER if alien else None
        return self._stream(self._ffmpeg_cmd('-', pre=gen, af=af), wait,
                            'say %r' % (text[:40]))

    # --- internet audio ----------------------------------------------------
    def play_url(self, url, wait=True):
        """Play the audio of a YouTube (or any yt-dlp-supported) URL.

        Streams bestaudio straight through ffmpeg to the speaker, no temp
        file.
        """
        if not os.path.exists(YTDLP):
            raise TalkError('yt-dlp not found at %s' % YTDLP)
        ytdlp = [YTDLP] + (['--proxy', PROXY] if PROXY else []) + [
                 '--js-runtimes', 'deno:' + DENO,
                 '--no-playlist', '-f', 'bestaudio', '-q', '--no-warnings',
                 '-o', '-', url]
        return self._stream(self._ffmpeg_cmd('-', pre=ytdlp), wait,
                            'play_url %s' % url)

    # --- plumbing ----------------------------------------------------------
    def voices(self):
        """Names of the installed Piper voices."""
        import glob
        return sorted(os.path.basename(f)[:-5]
                      for f in glob.glob(os.path.join(PIPER_DIR, '*.onnx')))

    def _ffmpeg_cmd(self, src, pre=None, af=None):
        """(producer_cmd_or_None, ffmpeg_cmd) that emits raw S16LE 48k stereo.

        src '-' means read stdin (from `pre`); otherwise it is a file path.
        """
        if not shutil.which('ffmpeg'):
            raise TalkError('ffmpeg not found on this machine - needed to '
                             'convert audio to the robot codec format')
        ff = ['ffmpeg', '-loglevel', 'error', '-i', src]
        lead = 'adelay=%d|%d' % (LEAD_IN_MS, LEAD_IN_MS)
        ff += ['-af', lead + ',' + af if af else lead]
        ff += ['-f', 's16le', '-ar', str(RATE), '-ac', str(CHANNELS), '-']
        return (pre, ff)

    def _tts_cmd(self, text, engine, voice=None):
        """A command that writes a wav of `text` to stdout, or raise."""
        piper = shutil.which('piper') or (PIPER if os.access(PIPER, os.X_OK) else None)
        espeak = shutil.which('espeak-ng') or shutil.which('espeak')
        if engine == 'piper' or (engine is None and piper):
            if not piper:
                raise TalkError('piper requested but not found at %s' % PIPER)
            model = os.path.join(PIPER_DIR, (voice or PIPER_VOICE) + '.onnx')
            if not os.path.exists(model):
                raise TalkError('no piper voice %r; installed: %s'
                                 % (voice or PIPER_VOICE, ', '.join(self.voices())))
            return ['bash', '-c',
                    'echo %s | %s -q --model %s %s --output_file -'
                    % (_shq(text), _shq(piper), _shq(model), ' '.join(PIPER_ARGS))]
        if engine in (None, 'espeak-ng', 'espeak') and espeak:
            return [espeak, '--stdout', text]
        raise TalkError(
            'no TTS engine available. Install one, then say() works:\n'
            '    sudo apt-get install -y espeak-ng      # robotic, instant\n'
            'Piper (good neural voice) is blocked on this box - see '
            'say()\'s docstring.')

    def _stream(self, plan, wait, label):
        """Run (producer | ffmpeg | ssh aplay -). Producer may be None."""
        pre, ff = plan
        sink = self._ssh + ['aplay', '-q', '-D', DEVICE, '-f', 'S16_LE',
                            '-r', str(RATE), '-c', str(CHANNELS), '-']
        p_pre = subprocess.Popen(pre, stdout=subprocess.PIPE) if pre else None
        p_ff = subprocess.Popen(
            ff, stdin=(p_pre.stdout if p_pre else None),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if p_pre:
            p_pre.stdout.close()      # let ffmpeg own the read end
        p_ap = subprocess.Popen(sink, stdin=p_ff.stdout,
                               stderr=subprocess.PIPE)
        p_ff.stdout.close()
        if not wait:
            return None
        ap_err = p_ap.communicate()[1]
        ff_err = p_ff.communicate()[1]
        # Check the producer first: when yt-dlp or the TTS engine dies, ffmpeg
        # only sees empty input and its error would hide the real cause.
        if p_pre and p_pre.wait() != 0:
            raise TalkError('%s: %s failed (exit %d), see its message above'
                             % (label, os.path.basename(pre[0]), p_pre.returncode))
        if p_ap.returncode != 0:
            raise TalkError('%s: aplay on robot failed: %s'
                             % (label, ap_err.decode(errors='replace').strip()))
        if p_ff.returncode not in (0, None):
            raise TalkError('%s: ffmpeg failed: %s'
                             % (label, ff_err.decode(errors='replace').strip()))
        return True

    def _run_remote(self, cmd, wait, label):
        if not wait:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
            return None
        out = subprocess.run(cmd, capture_output=True, text=True)
        if out.returncode != 0:
            raise TalkError('%s failed: %s' % (label, out.stderr.strip()))
        return True


class Speaker:
    """Streaming Piper: speak sentence by sentence as text arrives.

    say() queues ONE sentence and returns at once; finish() blocks until all
    of it has been played. Piper stays running between calls (its model load
    is ~1.4 s), and synthesises each line to its own wav in --output_dir,
    printing the path when done - so the first sentence plays while the next
    is still being generated. The speaker is opened lazily at the first say()
    and released in finish(), so between replies jy_exe can use it again.

        with Speaker(alien=True) as sp:
            sp.say('Hello friend.'); sp.say('You are well, question?')
            sp.finish()
    """

    def __init__(self, voice=None, alien=False, flat=True, host=HOST):
        """flat=True uses PIPER_ARGS (monotone); False is Piper's own
        expressive delivery."""
        import json, tempfile, threading
        model = os.path.join(PIPER_DIR, (voice or PIPER_VOICE) + '.onnx')
        if not os.access(PIPER, os.X_OK) or not os.path.exists(model):
            raise TalkError('Speaker needs piper at %s and voice %s' % (PIPER, model))
        self.rate = json.load(open(model + '.json'))['audio']['sample_rate']
        self.af = ALIEN_FILTER if alien else None
        self._ssh = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', host]
        self._dir = tempfile.mkdtemp(prefix='speaker-')
        self._piper = subprocess.Popen(
            [PIPER, '-q', '--model', model] + (PIPER_ARGS if flat else [])
            + ['--output_dir', self._dir],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, universal_newlines=True, bufsize=1)
        self._cv = threading.Condition()
        self._pending = 0
        self._player = None
        self._error = None
        threading.Thread(target=self._pump, daemon=True).start()

    def say(self, sentence):
        text = ' '.join(sentence.split())      # piper reads one line per utterance
        if not text:
            return
        with self._cv:
            if self._error:
                raise TalkError(self._error)
            if self._player is None:
                self._player = self._open_player()
            self._pending += 1
        self._piper.stdin.write(text + '\n')
        self._piper.stdin.flush()

    def finish(self, timeout=120):
        """Wait until everything said so far has played, then free the speaker."""
        with self._cv:
            if not self._cv.wait_for(lambda: self._pending == 0 or self._error, timeout):
                self._error = 'speaker timed out'
            player, self._player = self._player, None
            err = self._error
        if player:
            ff, ap = player
            ff.stdin.close()
            ff.wait()
            if ap.wait() != 0 and not err:
                err = 'aplay on the robot failed (exit %d)' % ap.returncode
        if err:
            raise TalkError(err)

    def close(self):
        try:
            self._piper.stdin.close()
        except OSError:
            pass
        self._piper.wait()
        shutil.rmtree(self._dir, ignore_errors=True)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _open_player(self):
        ff = ['ffmpeg', '-loglevel', 'error', '-f', 's16le', '-ar', str(self.rate),
              '-ac', '1', '-i', '-']
        gain = 'volume=%.2f,alimiter=limit=0.95' % SOFT_GAIN
        # Same amp lead-in as _ffmpeg_cmd: the ES8388 unmutes when the stream
        # opens and swallows whatever is already playing. One stream serves a
        # whole reply, so this costs LEAD_IN_MS once per reply, not per line.
        lead = 'adelay=%d|%d' % (LEAD_IN_MS, LEAD_IN_MS)
        ff += ['-af', ','.join(f for f in (self.af, gain, lead) if f)]
        ff += ['-f', 's16le', '-ar', str(RATE), '-ac', str(CHANNELS), '-']
        p_ff = subprocess.Popen(ff, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        p_ap = subprocess.Popen(
            self._ssh + ['amixer', '-q', '-c', '0', 'sset', 'PCM', str(HW_PCM), ';',
                         'aplay', '-q', '-D', DEVICE, '-f', 'S16_LE', '-r', str(RATE),
                         '-c', str(CHANNELS), '-'], stdin=p_ff.stdout,
            # the gap between sentences makes aplay print "underrun!!!", which
            # is expected here; real failures show up as its exit code.
            stderr=subprocess.DEVNULL)
        p_ff.stdout.close()
        return p_ff, p_ap

    def _pump(self):
        """Feed each finished wav into the open player, in order."""
        import wave
        for line in self._piper.stdout:
            path = line.strip()
            try:
                with wave.open(path) as w:
                    frames = w.readframes(w.getnframes())
                os.remove(path)
                with self._cv:
                    player = self._player
                if player:
                    player[0].stdin.write(frames)
                    player[0].stdin.flush()
            except (OSError, EOFError, wave.Error) as e:
                with self._cv:
                    self._error = 'speaker: %s' % e
            with self._cv:
                self._pending -= 1
                self._cv.notify_all()
        with self._cv:
            if self._pending:
                self._error = 'piper exited unexpectedly'
            self._cv.notify_all()



# --- Gemini -------------------------------------------------------------------
KEY_FILE = os.path.expanduser('~/.gemini_key')
# An alias, so a retired model version cannot break it (gemini-2.5-flash was
# refused for new keys on 2026-09-18). Lite answers fastest and stays in character.
MODEL = 'gemini-flash-lite-latest'
HISTORY = 20            # messages kept (user + model), to bound request size
# The free tier sometimes stalls or 503s a request while an immediate retry
# answers in ~1 s, so fail fast and retry rather than wait out one call.
TIMEOUT, TRIES = 12, 3
URL = ('https://generativelanguage.googleapis.com/v1beta/models/'
       '%s:generateContent')
# The front camera (on the motion computer) is already streamed by the robot's
# own gst-launch -> mediamtx for the phone app, and gst holds /dev/video0, so
# grab frames from that stream rather than the device.
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

# Personas that are not for the public demo live in persona_private.py, which
# is gitignored - the robot has them, GitHub does not. Absent is fine.
try:
    from .persona_private import PRIVATE_PERSONAS
except ImportError:
    pass
else:
    PERSONAS.update(PRIVATE_PERSONAS)
PERSONA = 'deadpan'     # safe for strangers; 'sarcastic' roasts people


def _key():
    try:
        return open(KEY_FILE).read().strip()
    except OSError:
        raise TalkError('no API key: put a Google AI Studio key in %s' % KEY_FILE)


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
    raise TalkError('Gemini did not answer after %d tries (%s)' % (TRIES, last))


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
            raise TalkError('rate limited by the free tier, wait a bit: ' + msg)
        raise TalkError('Gemini HTTP %d: %s' % (e.code, msg))
    except urllib.error.URLError as e:
        if isinstance(e.reason, socket.timeout):
            raise _Retry('timed out')
        raise TalkError('cannot reach Gemini (%s). Is the robot online? '
                        '(TALK_PROXY=%r)' % (e.reason, PROXY))


def _parse(data):
    try:
        parts = data['candidates'][0]['content']['parts']
        return ''.join(p.get('text', '') for p in parts).strip()
    except (KeyError, IndexError):
        raise TalkError('no reply (blocked or empty): %s' % json.dumps(data)[:300])


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
        raise TalkError('camera stream %s did not answer in 15 s' % url)
    if p.returncode != 0 or not p.stdout:
        raise TalkError('camera grab failed: %s'
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
        except TalkError:
            pass

    def ensure(self):
        with self._lock:
            if self.ws is None:
                self._connect()

    def ask(self, text, jpeg=None):
        """Yield the reply as transcript chunks, as they arrive.

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
                    raise TalkError('Live session lost or stalled: %s' % (e or 'timeout'))
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
        except TalkError:
            raise
        except Exception as e:
            self.close()
            raise TalkError('Live connect failed: %s' % e)
        if 'setupComplete' not in m:
            self.close()
            raise TalkError('Live setup refused: %s' % str(m)[:200])

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
        raise TalkError('cannot reach TALK_PROXY %s (%s). Is robot_proxy.py '
                        'running on the laptop with an SSH session open?' % (PROXY, e))
    if b' 200' not in resp.split(b'\r\n', 1)[0]:
        s.close()
        raise TalkError('proxy refused CONNECT: %r' % resp[:80])
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


class Talker:
    """A conversation with one persona, spoken through Speaker as it streams."""

    def __init__(self, model=MODEL, voice=None, speak=True, live=True,
                 persona=PERSONA):
        if persona not in PERSONAS:
            raise TalkError('unknown persona %r; have %s' % (persona, ', '.join(PERSONAS)))
        self.model, self.persona = model, persona
        self.name, _, default_voice, alien = PERSONAS[persona]
        self.history = []                   # for REST; Live keeps its own
        self.last_jpeg = None               # latest camera frame, for REST
        self.live = Live(persona=persona) if live else None
        self.speaker = None
        if speak:
            self.speaker = Speaker(voice=voice or default_voice, alien=alien, flat=alien)

    def ask(self, text, on_sentence=None, jpeg=None, speak=True):
        """The persona's answer to `text`, spoken sentence by sentence as it arrives.

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
                except TalkError as e:
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
        """Speak text that is already written (e.g. from ask(speak=False))."""
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
        """Grab a front-camera frame and have him say what he sees."""
        return self.ask(question or 'What do you see right now through your camera?',
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

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def _chain(parts):
    for p in parts:
        if isinstance(p, str):
            yield p
        else:
            yield from p


# --- helpers ---------------------------------------------------------------
def _shq(s):
    """Single-quote a string for a bash -c payload."""
    return "'" + s.replace("'", "'\\''") + "'"


def _cli(argv):
    opts = {'model': MODEL, 'voice': None, 'speak': True, 'live': True,
            'persona': PERSONA}
    alien, args, it = False, [], iter(argv)
    for a in it:
        if a in ('-h', '--help', 'help'):
            print(__doc__); return 0
        elif a == '--quiet':
            opts['speak'] = False
        elif a == '--rest':
            opts['live'] = False
        elif a == '--alien':
            alien = True
        elif a in ('--model', '--voice', '--persona'):
            opts[a[2:]] = next(it)
        else:
            args.append(a)
    cmd, rest = (args[0], ' '.join(args[1:])) if args else ('chat', '')

    try:
        v = Voice()
        if cmd == 'say':
            v.say(rest, voice=opts['voice'], alien=alien); return 0
        if cmd == 'play':
            v.play(rest); return 0
        if cmd == 'file':
            v.play_file(rest); return 0
        if cmd == 'url':
            v.play_url(rest); return 0
        if cmd == 'clips':
            print('\n'.join(v.clips())); return 0
        if cmd == 'voices':
            print('\n'.join(v.voices())); return 0
        if cmd not in ('chat', 'ask', 'look'):
            print('unknown command %r - see --help' % cmd, file=sys.stderr)
            return 2
    except TalkError as e:
        print('error: %s' % e, file=sys.stderr)
        return 1

    talker = Talker(**opts)

    def show(s):
        print(s, end=' ', flush=True)

    def turn(text, fn=talker.ask):
        print(talker.name + ':', end=' ', flush=True)
        try:
            fn(text, on_sentence=show)
            print()
            return True
        except TalkError as e:       # keep chatting
            print('\nerror: %s' % e, file=sys.stderr)
            return False

    try:
        if cmd == 'look':
            return 0 if turn(rest or None, talker.look) else 1
        if cmd == 'ask':
            return 0 if turn(rest) else 1
        print('Talking to %s (%s). Ctrl-D to quit.'
              % (talker.name, LIVE_MODEL if opts['live'] else opts['model']))
        while True:
            try:
                line = input('you> ').strip()
            except (EOFError, KeyboardInterrupt):
                print(); return 0
            if line.startswith('/look'):
                turn(line[5:].strip() or None, talker.look)
            elif line:
                turn(line)
    finally:
        talker.close()


if __name__ == '__main__':
    sys.exit(_cli(sys.argv[1:]))
