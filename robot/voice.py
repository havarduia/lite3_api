#!/usr/bin/env python3
"""Play audio - and eventually speech - out of the Lite3's speaker.

The speaker is NOT on this computer (the Jetson). It hangs off an ES8388
codec on the MOTION computer, 192.168.1.120 - the same box you send stand/
walk UDP to. DEEP's own firmware plays its canned lines by literally running
`aplay .../lite3_voice/NAME.wav` over there, so that is all "playing a sound"
is: run aplay on 192.168.1.120. This module does exactly that from here.

    from robot.voice import Voice
    v = Voice()
    v.play('OKstandup')            # one of the robot's own ~33 clips
    v.play_file('siren.mp3')       # any local audio, converted and streamed
    v.say('watch your step')       # text -> speech (needs a TTS engine, see below)
    v.play_url('https://www.youtube.com/watch?v=...')  # a video's audio
    print(v.clips())               # list the built-in clip names

Or from the shell:

    python3 voice.py list
    python3 voice.py play OKstandup
    python3 voice.py file ~/alarm.wav
    python3 voice.py say "hello there"
    python3 voice.py voices
    python3 voice.py say --voice en_US-ryan-medium --rocky "amaze amaze amaze"

Requirements:
  - Passwordless SSH from here to ysc@192.168.1.120. Already set up.
  - ffmpeg on THIS machine, for play_file/say format conversion. Present.
  - ysc in the `audio` group on the motion computer, since aplay goes to
    DEVICE directly. Done 2026-09-15.
  - A TTS engine for say() on THIS machine: Piper in ~/piper (preferred,
    installed 2026-09-18), else espeak-ng (installed 2026-09-15).

Audio is streamed to the robot as raw S16LE 48 kHz stereo - the format its
clips already use and the codec is known to accept - so there is no wav
header travelling down the pipe to be misread.
"""
import os
import shutil
import subprocess
import sys

HOST = 'ysc@192.168.1.120'          # motion computer, where the speaker is
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
# or github from here). The robot has no internet, so yt-dlp goes out via
# the laptop's robot_proxy.py, which the SSH RemoteForward puts on :3128.
YTDLP = os.path.expanduser('~/.local/bin/yt-dlp')
DENO = os.path.expanduser('~/.local/bin/deno')    # yt-dlp's JS runtime for YouTube
PROXY = 'http://localhost:3128'

# say(): Piper, the standalone aarch64 release copied in from the laptop
# (2026-09-18). ~/piper is not on PATH for non-interactive ssh, so it is found
# here by absolute path. Voices are ~/piper/<name>.onnx (+ .onnx.json).
PIPER = os.path.expanduser('~/piper/piper')
PIPER_DIR = os.path.expanduser('~/piper')
PIPER_VOICE = 'en_US-lessac-medium'
# Low noise = flat intonation and even timing: the "translation computer" sound.
PIPER_ARGS = ['--noise_scale', '0.1', '--noise_w', '0.1', '--length_scale', '1.1']
# rocky=True: thin, slightly raised, faintly metallic small-speaker voice.
# Loudness for Speaker (2026-09-18, user asked for louder): the codec's PCM
# volume goes to max (192 = 0 dB, was 180 = -6 dB) each time the speaker
# opens, and speech gets a software gain with a limiter so peaks don't clip.
# Output 1/2 stay at 0 dB; pushing the amp past that on this small speaker
# distorts. Set SOFT_GAIN = 1.0 to undo the software part.
HW_PCM = 192
SOFT_GAIN = 1.6
ROCKY_FILTER = ('rubberband=pitch=1.12,highpass=f=300,lowpass=f=5000,'
                'aecho=0.8:0.6:12:0.2')

# ssh with no interactive prompt: rely on the installed key, fail fast if it
# ever stops working rather than hanging on a password ask.
_SSH = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', HOST]


class VoiceError(RuntimeError):
    pass


class Voice:
    """Sound out of the robot. One SSH hop per sound; nothing stays running."""

    def __init__(self, host=HOST, clip_dir=CLIP_DIR):
        self.host = host
        self.clip_dir = clip_dir
        self._ssh = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', host]

    # --- the built-in clips ------------------------------------------------
    def clips(self):
        """Names (without .wav) of the clips already on the robot."""
        out = subprocess.run(self._ssh + ['ls', '-1', self.clip_dir],
                             capture_output=True, text=True)
        if out.returncode != 0:
            raise VoiceError('could not list %s on %s: %s'
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
            raise VoiceError('no clip %r. Available: %s'
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
    def say(self, text, wait=True, engine=None, voice=None, rocky=False):
        """Speak `text` out of the robot, using a TTS engine on THIS machine.

        There is deliberately no fallback voice baked in: a wrong-but-silent
        beep would be worse than a clear error. The lookup order is Piper
        (good neural voice, ~/piper, flattened by PIPER_ARGS) then espeak-ng
        (robotic but real). Piper runs at about real time on this CPU, so a
        sentence takes a few seconds before it starts playing.

        `engine` forces one ('piper' or 'espeak-ng') instead of autodetecting.
        `voice` picks a Piper model by name (see voices()); default PIPER_VOICE.
        `rocky=True` adds ROCKY_FILTER on top.
        """
        gen = self._tts_cmd(text, engine, voice)
        # TTS engine -> wav on stdout -> ffmpeg reformats -> ssh -> aplay.
        af = ROCKY_FILTER if rocky else None
        return self._stream(self._ffmpeg_cmd('-', pre=gen, af=af), wait,
                            'say %r' % (text[:40]))

    # --- internet audio ----------------------------------------------------
    def play_url(self, url, wait=True):
        """Play the audio of a YouTube (or any yt-dlp-supported) URL.

        Streams bestaudio straight through ffmpeg to the speaker, no temp
        file. Needs the laptop: robot_proxy.py running there AND an SSH
        session from it to this box open (that is what forwards :3128).
        With either missing, yt-dlp fails with a connection error.
        """
        if not os.path.exists(YTDLP):
            raise VoiceError('yt-dlp not found at %s' % YTDLP)
        ytdlp = [YTDLP, '--proxy', PROXY, '--js-runtimes', 'deno:' + DENO,
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
            raise VoiceError('ffmpeg not found on this machine - needed to '
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
                raise VoiceError('piper requested but not found at %s' % PIPER)
            model = os.path.join(PIPER_DIR, (voice or PIPER_VOICE) + '.onnx')
            if not os.path.exists(model):
                raise VoiceError('no piper voice %r; installed: %s'
                                 % (voice or PIPER_VOICE, ', '.join(self.voices())))
            return ['bash', '-c',
                    'echo %s | %s -q --model %s %s --output_file -'
                    % (_shq(text), _shq(piper), _shq(model), ' '.join(PIPER_ARGS))]
        if engine in (None, 'espeak-ng', 'espeak') and espeak:
            return [espeak, '--stdout', text]
        raise VoiceError(
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
            raise VoiceError('%s: %s failed (exit %d), see its message above'
                             % (label, os.path.basename(pre[0]), p_pre.returncode))
        if p_ap.returncode != 0:
            raise VoiceError('%s: aplay on robot failed: %s'
                             % (label, ap_err.decode(errors='replace').strip()))
        if p_ff.returncode not in (0, None):
            raise VoiceError('%s: ffmpeg failed: %s'
                             % (label, ff_err.decode(errors='replace').strip()))
        return True

    def _run_remote(self, cmd, wait, label):
        if not wait:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
            return None
        out = subprocess.run(cmd, capture_output=True, text=True)
        if out.returncode != 0:
            raise VoiceError('%s failed: %s' % (label, out.stderr.strip()))
        return True


class Speaker:
    """Streaming Piper: speak sentence by sentence as text arrives.

    say() queues ONE sentence and returns at once; finish() blocks until all
    of it has been played. Piper stays running between calls (its model load
    is ~1.4 s), and synthesises each line to its own wav in --output_dir,
    printing the path when done - so the first sentence plays while the next
    is still being generated. The speaker is opened lazily at the first say()
    and released in finish(), so between replies jy_exe can use it again.

        with Speaker(rocky=True) as sp:
            sp.say('Hello friend.'); sp.say('You are well, question?')
            sp.finish()
    """

    def __init__(self, voice=None, rocky=False, flat=True, host=HOST):
        """flat=True uses PIPER_ARGS (monotone); False is Piper's own
        expressive delivery."""
        import json, tempfile, threading
        model = os.path.join(PIPER_DIR, (voice or PIPER_VOICE) + '.onnx')
        if not os.access(PIPER, os.X_OK) or not os.path.exists(model):
            raise VoiceError('Speaker needs piper at %s and voice %s' % (PIPER, model))
        self.rate = json.load(open(model + '.json'))['audio']['sample_rate']
        self.af = ROCKY_FILTER if rocky else None
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
                raise VoiceError(self._error)
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
            raise VoiceError(err)

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


# --- helpers ---------------------------------------------------------------
def _shq(s):
    """Single-quote a string for a bash -c payload."""
    return "'" + s.replace("'", "'\\''") + "'"


def _cli(argv):
    v = Voice()
    if not argv or argv[0] in ('-h', '--help', 'help'):
        print(__doc__)
        return 0
    cmd, rest = argv[0], argv[1:]
    try:
        if cmd == 'list':
            print('\n'.join(v.clips()))
        elif cmd == 'play':
            v.play(rest[0]); print('played %s' % rest[0])
        elif cmd == 'file':
            v.play_file(rest[0]); print('played %s' % rest[0])
        elif cmd == 'voices':
            print('\n'.join(v.voices()))
        elif cmd == 'say':
            rocky = '--rocky' in rest
            voice = None
            if '--voice' in rest:
                i = rest.index('--voice'); voice = rest[i + 1]; del rest[i:i + 2]
            rest = [w for w in rest if w != '--rocky']
            v.say(' '.join(rest), voice=voice, rocky=rocky); print('said it')
        else:
            print('commands: list | play <name> | file <path> | voices | '
                  'say [--voice NAME] [--rocky] <text>')
            return 2
    except VoiceError as e:
        print('error: %s' % e, file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(_cli(sys.argv[1:]))
