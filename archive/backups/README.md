# Backups

The `.bak` files that used to sit loose in `~`, filed by the script they belong
to. Nothing references them, so moving or removing any of them breaks nothing.

Restore one with a plain copy, e.g.

    cp ~/archive/backups/lite3/lite3.py.pre-arm.bak ~/lite3.py

**Timestamps on these files are meaningless.** The perception computer's clock
is skewed, so `ls -l` shows dates like "Aug 24" or "Sep 15 2026" that have
nothing to do with when the change was made. Use the dates below instead; they
come from the project notes, not the filesystem.

Each file is the state of the script **immediately before** the change named.

## lite3/ — the robot API

| file | taken before |
|---|---|
| `lite3.py.pre-arm.bak` | the `stand()` arming fix: re-sending the toggle that jy_exe swallows to arm itself (2026-09-21) |
| `lite3.py.pre-autoexit.bak` | `auto()` sending POSTURE_EXIT first, so a robot left in posture mode stops silently ignoring velocity (2026-09-18) |
| `lite3.py.pre-sidecheck.bak` | `side_clear()` and `turn(guard=True)` (2026-09-18) |
| `lite3.py.pre-tilt.bak` | `tilt()` and posture ("twist body") mode (2026-09-18) |
| `lite3.py.pre-steer.bak` | `steer(control)`, the per-cycle driving used by person following (2026-09-18) |
| `lite3.py.pre-goal.bak` | `goto(heading_deg=None)` stopping wherever he arrives, instead of circling the goal (2026-09-18) |
| `lite3.py.pre-piper.bak` | `bot.say()` gaining the `voice=` / `rocky=` passthrough (2026-09-18) |
| `lite3.py.bak` | unrecorded — predates the project notes. The oldest thing here. |

## voice/ — audio output and TTS

| file | taken before |
|---|---|
| `voice.py.pre-volume.bak` | the louder-speech change: ES8388 PCM to 192 plus ffmpeg gain and a limiter (2026-09-18) |
| `voice.py.pre-flat.bak` | splitting flat vs expressive Piper delivery, so sarcasm is not deadpan (2026-09-18) |
| `voice.py.pre-stream.bak` | `Speaker`: one long-lived Piper process streaming into ffmpeg, instead of a whole WAV per line (2026-09-18) |
| `voice.py.pre-piper.bak` | Piper TTS replacing espeak-ng (2026-09-18) |
| `voice.py.pre-url.bak` | `play_url()` and the yt-dlp path (2026-09-15) |
| `voice.py.pre-tts.bak` | `say()` working at all, via espeak-ng, and the stale docstrings being corrected (2026-09-15) |
| `voice.py.bak` | `DEVICE = 'plughw:0,0'` — the fix for aplay "playing" into a null sink and reporting success (2026-09-15) |

## rocky/ — the talking persona

| file | taken before |
|---|---|
| `rocky.py.pre-deadpan.bak` | `deadpan` becoming the default persona (2026-09-18) |
| `rocky.py.pre-funny.bak` | the sarcastic persona rewrite — plainer words, jokes that land (2026-09-18) |
| `rocky.py.pre-persona.bak` | personas existing at all (2026-09-18) |
| `rocky.py.pre-look.bak` | `look()`: grabbing a frame and sending it to Gemini (2026-09-18) |
| `rocky.py.pre-live.bak` | Gemini Live over WebSocket, with the REST path kept as fallback (2026-09-18) |

## rocky_walk/ — walking and commenting

| file | taken before |
|---|---|
| `rocky_walk.py.pre-arm.bak` | removing the redundant `time.sleep(3.0)` after `heartbeat_start()`, once `stand()` handled the swallowed toggle itself (2026-09-21) |
| `rocky_walk.py.pre-persona.bak` | personas (2026-09-18) |

## run_robot/ — the Mission wrapper

| file | taken before |
|---|---|
| `run_robot.py.pre-remote.bak` | `play_url(remote=True)`, which hands control back to the handheld during a track (2026-09-15) |
| `run_robot.py.pre-url.bak` | `play_url()` (2026-09-15) |
| `run_robot.py.pre-speak.bak` | the `speak()` wiring (2026-09-15) |
| `run_robot.py.bak` | unrecorded — predates the notes. |

## person/

| file | what it is |
|---|---|
| `person.py.yolo-jetson.bak` | **Not a rollback.** The older working implementation, using the Jetson's own YOLOv8 TensorRT engine (`~/lite_cog_ros2/track`) instead of the RK3588 `track` binary on the motion computer. Kept because it still works; it just needs ~1.5 min to load the engine. |

---

New backups belong in here too, in the matching folder, rather than loose in `~`.
