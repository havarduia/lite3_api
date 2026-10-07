# lite3-robot

A Python API and a set of behaviours for a **DEEP Robotics Lite3 Pro** quadruped
("robot dog"), running on the robot's own perception computer (NVIDIA Jetson
Xavier NX, Ubuntu 20.04, **ROS2 Foxy**, Python 3.8).

The vendor gives you a robot that can be driven with a handheld remote, plus a
collection of ROS2 nodes and undocumented UDP ports. This repository turns that
into something you can program in a few lines:

```python
from robot.lite3 import Lite3

with Lite3() as bot:
    bot.stand()
    bot.walk(1.0)              # stops by itself if the depth camera sees an obstacle
    bot.turn_deg(90)           # +ve = left
    print(bot.scan_text())     # what the depth camera sees, as a bar chart
    bot.nav_start()            # launch Nav2
    bot.goto(2.0)              # plan a route 2 m ahead, around obstacles
```

On top of that API sit: person following, a talking personality (Google Gemini
+ on-device text-to-speech through the robot's speaker), a scripted "tour"
program, keyboard teleoperation, and a web control panel (HMI) for a phone or
laptop.

---

## Contents

1. [The two-minute explanation](#1-the-two-minute-explanation)
2. [The hardware and the two computers](#2-the-hardware-and-the-two-computers)
3. [How a command reaches the legs](#3-how-a-command-reaches-the-legs)
4. [Repository layout](#4-repository-layout)
5. [The core: `Lite3` (`robot/lite3.py`)](#5-the-core-lite3-robotlite3py)
6. [Seeing: depth camera and sonars](#6-seeing-depth-camera-and-sonars)
7. [Navigation: mapless Nav2](#7-navigation-mapless-nav2)
8. [Finding and following people](#8-finding-and-following-people)
9. [Talking: speaker, TTS and Gemini](#9-talking-speaker-tts-and-gemini)
10. [Programs you run](#10-programs-you-run)
11. [The environment and our patches to vendor code](#11-the-environment-and-our-patches-to-vendor-code)
12. [Measured numbers and calibration](#12-measured-numbers-and-calibration)
13. [Design principles](#13-design-principles)
14. [Known limitations](#14-known-limitations)
15. [Running things](#15-running-things)
16. [Things that will bite you](#16-things-that-will-bite-you)
17. [Questions you are likely to be asked](#17-questions-you-are-likely-to-be-asked)
18. [History](#18-history)
19. [Glossary](#19-glossary)

---

## 1. The two-minute explanation

If you have to explain the project out loud, this is the story:

> The Lite3 has two computers. A **motion computer** runs the vendor's
> closed-source controller (`jy_exe`) that actually balances and walks the
> robot. A **perception computer** (a Jetson) runs ROS2, the depth camera and
> anything we write. The two talk over UDP.
>
> Out of the box, controlling the robot from code is full of *silent*
> failures: commands that arrive and are simply ignored because the robot is
> in the wrong mode, lying down, low on battery, or not "armed". None of these
> produce an error message.
>
> So the heart of this project is **one Python class, `Lite3`**, that hides
> the ROS plumbing and turns every one of those silent failures into either an
> automatic fix or a clear exception. It runs a ROS node in a background
> thread so the robot's state (battery, pose, posture, sonar, tilt) is always
> available as plain attributes. Motion calls like `walk()` and `turn()` block
> until done, check safety every 50 ms (obstacle ahead, someone grabbed the
> remote, body tilting over), and are guaranteed to leave the robot stopped,
> even if the program crashes or you press Ctrl-C.
>
> On top of that we added **obstacle detection** from the depth camera's point
> cloud, **navigation** using the standard ROS2 Nav2 stack without a map, the
> two **ultrasonic sensors** fed into Nav2's costmaps, **person following**
> using the robot's built-in person detector, and a **voice**: Gemini writes
> what to say, Piper text-to-speech says it through the robot's speaker.
> Finally there are user programs: a guided tour that walks and comments on
> what it sees, keyboard teleop, and a web control panel with live cameras,
> a joystick and an emergency stop.
>
> Every constant in the code was measured on the real robot (for example: the
> leg odometry under-counts distance by 15%, so we scale it by 1.15; turns
> coast 0.17 s after the stop command, so we stop early), and the comments
> record what was measured and when.

---

## 2. The hardware and the two computers

The Lite3 Pro uses a "split brain": an RK3588 ARM motion host running the
high-rate (1 kHz) joint control loop, and a Jetson Xavier NX perception host
for SLAM, navigation and vision
([DEEP Robotics](https://www.deeprobotics.us/products/lite-3/)).

```
                        ┌───────────────────────────────────────────┐
  Handheld remote ─────►│  MOTION computer  (RK3588, 192.168.1.120) │
  (sticks, heartbeat)   │                                           │
                        │  jy_exe      the vendor locomotion        │
                        │              controller. UDP :43893 in,   │
                        │              telemetry out to :43897      │
                        │  track       person detector (YOLOv5s on  │
                        │              the NPU). UDP :43901, JSON   │
                        │  front wide-angle camera → RTSP :8554     │
                        │  speaker (ES8388 codec), played by aplay  │
                        │  front + rear ultrasonic sensors          │
                        └──────────────────┬────────────────────────┘
                                   Ethernet │ UDP
                        ┌──────────────────▼────────────────────────┐
                        │  PERCEPTION computer (Jetson Xavier NX)   │
                        │  Ubuntu 20.04, ROS2 Foxy, CycloneDDS      │
                        │                                           │
  RealSense D435i ─USB─►│  realsense_ros2.service  depth + colour   │
  (depth camera,        │  transfer_ros2.service   UDP <-> ROS2     │
   pitched 20° down)    │         (Jetson2Motion, patched by us)    │
                        │  voa_ros2.service  vendor obstacle        │
                        │                    avoidance (optional)   │
                        │  Nav2 (mapless)    started by our code    │
                        │  ► THIS REPOSITORY ◄                      │
                        └───────────────────────────────────────────┘
                                        │ Tailscale (VPN)
                                        ▼
                              laptop / phone → web HMI :8080
```

Where each sensor lives matters in the code:

| Thing | Lives on | How our code reaches it |
|---|---|---|
| Walking, posture, tricks | motion (`jy_exe`) | UDP packets from `protocol.py`, or `/cmd_vel` via `transfer_ros2` |
| Robot state, battery, leg odometry, IMU, sonars | motion → republished on Jetson | ROS2 topics from `transfer_ros2` |
| Depth camera (RealSense D435i) | Jetson, USB | ROS2 topic `/camera/depth/color/points` |
| Front wide-angle camera | motion | RTSP stream, grabbed with `ffmpeg` |
| Person detector | motion (NPU) | UDP + JSON on port 43901 |
| Speaker | motion | `ssh` to the motion computer, run `aplay` |

---

## 3. How a command reaches the legs

There are **two paths** into `jy_exe`, and the code uses both.

### Path A: "simple" UDP frames (posture, mode, heartbeat, tricks)

`robot/protocol.py` sends 12-byte packets `<code:uint32, value:int32, 0>`
directly to `192.168.1.120:43893`. Examples:

| Code | Meaning |
|---|---|
| `0x21010202` | stand ↔ lie. **A toggle**: sending it to a standing robot lies it down |
| `0x21010C03` | auto mode. Without it, velocity commands are silently ignored |
| `0x21010C02` | manual mode (the handheld sends this when a stick moves) |
| `0x21040001` | heartbeat. The controller keepalive, needed ≥ 2 Hz |
| `0x21010D05` / `0D06` | enter / leave posture ("twist body") mode |
| `0x21010130` | pitch stick (in posture mode: tilts the body) |
| `0x21010506` etc. | built-in tricks (`hello`, `dance`, …) |

Some codes come from DEEP Robotics' official source, some from a community
SDK, and several were captured from the handheld and phone app by us.
`protocol.py` records which is which.

### Path B: ROS2 velocity (`/cmd_vel`)

```
our code ──/cmd_vel (Twist)──► Jetson2Motion (transfer_ros2) ──UDP cmd 320──► jy_exe
Nav2's controller ──/cmd_vel──┘
voa ──/cmd_vel_corrected──────┘  (if the vendor obstacle avoider is running)
```

`Jetson2Motion` is the vendor's bridge node. In the other direction it takes
the robot's UDP telemetry (port 43897) and publishes it as ROS topics:

| Topic | Contents |
|---|---|
| `/robot_state_debug` | `Int32MultiArray`: basic_state, gait, …, **battery** (we added battery, error, charging) |
| `/leg_odom2` | odometry from leg kinematics (we scale it by 1.15) |
| `/imu/data` | body IMU |
| `/us_publisher/ultrasound_distance` | rear sonar |
| `/us_publisher/ultrasound_front` | front sonar (we added this; stock only published one) |
| `/handle_state` | the handheld's stick positions |

### The interlock ("arming")

`jy_exe` ignores every network command unless a controller heartbeat keeps
arriving. Normally the handheld sends it. `Lite3.heartbeat_start()` sends it
from Python instead (2 Hz), which means the robot obeys **with no handheld
powered on**. That is convenient and also the most important safety caveat in
the project: the terminal (Ctrl-C) becomes your stop button.

Even with the heartbeat, a freshly powered robot is "settled but not armed":
the **first real command is swallowed** by `jy_exe`'s own arming sequence.
`stand()` detects this and re-sends the toggle (see §5.3). Measured
2026-09-21: the heartbeat alone does not arm him, and neither do mode
commands; he sits at 98 indefinitely until a real command arrives, so no
amount of waiting after `heartbeat_start()` helps.

---

## 4. Repository layout

```
robot/                 the library: `from robot.lite3 import Lite3`
  lite3.py             THE API: state, heartbeat, posture, tricks, motion, e-stop
  nav.py               Nav2 start/stop, costmap reading, goto()      (mixed into Lite3)
  depth.py             point cloud → obstacle ranges                 (mixed into Lite3)
  protocol.py          every UDP code and address; no ROS; also a raw-code CLI
  sonar_range.py       ROS node: sonars → sensor_msgs/Range for Nav2
  person.py            person detection (built-in tracker) and following
  talk.py              speaker, Piper TTS, YouTube audio, Gemini chat, personas
  hmi.py               web control panel server (aiohttp)
  hmi_static/index.html  the control panel page
  hmi_static/lib/      three.js, for the panel's 3D view
  rs_stream.py         RealSense colour → H.264 on mediamtx  (started by hmi.py)
  udp_relay.py         WebRTC video between the tailnet and mediamtx (started by hmi.py)
  rviz.py              the live robot in rviz2 on the laptop (run it on the laptop, §10.6)
bin/                   things you run
  tour.py              walk / navigate a route, stop, look, talk
  teleop.py            drive from the keyboard over SSH
demos/                 teaching and regression scripts
  demo.py              guided tour of the API in 10 lessons
  shake.py             end-to-end regression test          (MOVES THE ROBOT)
  estop_nav.py         proves e-stop works under Nav2      (MOVES THE ROBOT)
  check_motion.py      the motion loop's promises against a fake robot (no robot needed)
env/                   environment, launchers, and patches to vendor code
  lite3_env.sh         source this first: ROS workspaces, CycloneDDS, PYTHONPATH
  start_nav2_mapless.sh   launches sonar node + Nav2 (called by nav_start())
  camera.launch.py     RealSense D435i launch: depth + colour + IMU at 30 fps
  start_realsense_v4.sh   starts it (the realsense_ros2 systemd unit calls this)
  lite3.rviz           rviz2 layout used by robot/rviz.py
  *.patch              our changes to vendor code (see §11)
urdf/                  Lite3 model and meshes for rviz (vendor's, joints renamed, see the file's header)
  sudoers-lite3-camera, voa_ros2-override.conf   system config
```

The package deliberately has no re-exports in `robot/__init__.py`: importing
`robot.talk` or `robot.protocol` must not drag in `rclpy`, so those modules
work on a machine without ROS.

---

## 5. The core: `Lite3` (`robot/lite3.py`)

### 5.1 Structure

```python
class Lite3(Nav, Depth):     # Nav from nav.py, Depth from depth.py
```

`Lite3` is one class assembled from three files using **mixins**: `Nav` adds
`nav_start/nav_stop/cost_ahead/goto`, `Depth` adds `scan/clearance/side_clear`.
They share one ROS node and one set of helpers.

When you create a `Lite3()`:

1. `RMW_IMPLEMENTATION` is forced to CycloneDDS **before `rclpy` is
   imported**. (With the default FastRTPS the node discovers nothing and hangs
   with no error.)
2. `rclpy.init()`, then an internal node `_Node` subscribes to everything
   interesting: state, odometry, point cloud, global costmap, both sonars, IMU
   and handheld sticks.
3. A **background thread spins the node** (`spin_once` every 50 ms). This is
   the key usability trick: callbacks just store the latest message, so
   `bot.battery`, `bot.pose`, `bot.standing` are always-current attributes.
   User code never writes a ROS callback or spin loop.
4. A Ctrl-C handler is installed that runs `estop()`.
5. It waits up to 10 s for `/robot_state_debug`; if nothing arrives it raises
   with the likely cause (transfer service down, or another program holding
   UDP port 43897).
6. It puts the robot in auto mode.

Used as a context manager (`with Lite3() as bot:`), `close()` always runs:
halt, drop the heartbeat, then tear down ROS in a specific order (spin thread
first, then action clients, then executor and node). Each step of that order
fixes a crash that actually happened (segfault, `InvalidHandle`).

### 5.2 State

| Attribute | Source | Meaning |
|---|---|---|
| `state` | `/robot_state_debug` | dict: `basic, gait, policy, motion, task, need_move, zero_flag, battery, error, charging` |
| `battery` | same | percent |
| `standing` | same | `basic == 6` |
| `pose` | `/leg_odom2` | `(x, y, yaw)` in the odom frame |
| `attitude` | `/imu/data` | `(roll, pitch)` in degrees (quaternion → Euler in `_imu_cb`) |
| `ultrasound` | sonar topics | `(front, rear)` metres |
| `nav_running` | `pgrep bt_navigator` | is Nav2 up |
| `status()` | all of the above | one-line summary |

**`basic_state`** is the number everything depends on. Values observed on
this robot:

| basic | Meaning | Can we command it? |
|---|---|---|
| 98 | just powered on | settled, **not armed** (first command is eaten) |
| 8 | interlock dropped (no heartbeat) | settled, **not armed** |
| 9 | arming, in transition 8→9→1 | no, transitional |
| 1 | lying down, armed | yes |
| 17, 4, 5, 7 | standing up / lying down in progress | no, transitional |
| 6 | standing | yes, the only state that accepts velocity |

Toggling from a transitional state can lie the robot back down mid-move, so
the code only toggles from a *settled* lying state (`LYING = (1, 8, 98)`), and
`wait_ready()` waits for a settled state (`READY_STATES = (1, 6, 98)`).

### 5.3 Posture: `stand()`, `sit()`, `tilt()`, `action()`

- **`stand()`** is idempotent even though the raw command is a toggle:
  it returns at once if already standing, refuses if the state is
  transitional, checks battery, sends the toggle, and watches `basic_state`.
  A toggle that is acted on leaves the lying set within ~25 ms. If after
  `ARM_GRACE` (2 s) the robot is *still* in a lying state, the toggle was
  swallowed by arming, so it sends it **once more**. That retry is the fix
  for "the first stand after boot does nothing".
- **`sit()`**: halt, toggle, wait for a lying state. Idempotent.
- **`upright(heartbeat=True)`** is a context manager for the whole routine:
  start the heartbeat (unless a handheld holds the interlock), wait for
  ready, stand, and sit on the way out whatever happens. The heartbeat is
  left to `close()`, so he always sits first and is disarmed second.
  `bin/tour.py`, `bin/teleop.py` and lesson 10 use it.
- **`tilt(pitch_deg)`** is a context manager that pitches the body nose-up
  while standing still (used so the front camera sees a face, not knees). It
  enters posture mode, re-sends the pitch stick at 10 Hz like the phone app
  does, and on exit levels the body, leaves posture mode and settles. Full
  scale is about 14°; *negative* stick value = nose up (verified with the
  camera).
- **`action(name)`** plays a built-in trick (`hello`, `dance`, `twist`, …)
  from `protocol.ACTIONS`, each tagged with the posture it must start from.
  Sent 3× at 1 Hz like the vendor's own examples. Only `dance` has been
  verified on this robot.
- **`auto()`** sends "leave posture mode" and then "auto mode". Leaving
  posture mode first matters: a robot left in it reports standing but ignores
  every velocity (seen 2026-09-18, probably after the app's posture control).

### 5.4 Motion: `walk()`, `turn()`, `strafe()`, `steer()`

All motion goes through one loop, `_loop()`. `_run(vx, vy, wz, done, limit,
force, abort)` feeds it a fixed velocity; `steer()` one recomputed every cycle:

```
pre-checks:  battery ≥ 20%  →  standing  →  auto mode  →  have a pose
loop every 50 ms (20 Hz), until deadline (≤ 30 s HARD_TIMEOUT):
    _safety():  handheld stick pushed since we started?  → stop
                body roll/pitch > 30°?                  → stop
    publish Twist(vx, vy, wz)
    abort():    caller-specific check (obstacle, lost camera) → stop
    done():     target reached? → stop
finally:
    halt()  = 20 zero Twists over 0.4 s (best-effort QoS can drop one)
return {'reason': ..., 'start': pose, 'end': pose}
```

The **`finally: halt()`** is the guarantee: exception, Ctrl-C or normal exit,
the last thing published is zero velocity. The returned `reason` tells the
caller *why* it stopped (`'target reached'`, `'obstacle at 0.58 m'`,
`'handheld took over - stopped'`, …), because a walk that stopped short is
normal, not an error.

The tilt limit is 30° (`TILT_LIMIT_DEG`). Walking on the flat stays within a
few degrees; the robot is rated for 40° slopes, so raise it before trying one.
A non-finite velocity (NaN) is sent as zero: Python's `max`/`min` would
otherwise pass it through as the full limit.

**`walk(distance, speed=0.15, stop_distance=0.6)`**
- Distance is measured from odometry (straight-line from the start pose).
- Before moving it checks `clearance()`; if an obstacle is already inside
  `stop_distance` it refuses to start.
- While walking it re-checks clearance at 4 Hz (the point cloud walk is
  too expensive for 20 Hz) and stops if something is inside `stop_distance`,
  or if the depth stream is lost ("stopped rather than walking blind").
- Walking backwards is unguarded (the camera only faces forward) and says so.

**`turn(radians, rate=0.4)`** / **`turn_deg(deg)`**
- Relative to the heading at call time, positive = left, any size (even > 360°).
- Heading is **accumulated per sample**: each 50 ms step is wrapped to
  [-π, π] and added up. Wrapping the *total* instead saturates at π, which
  once made a 180° request spin ~409° until the timeout.
- **Coast compensation**: the robot keeps turning ~0.17 s after the zero
  command, which overshot every turn by 3.5–4.5°. So it stops
  `rate × 0.17 s` early, waits 0.5 s to settle, and reports the settled
  angle. After the fix, ±30° requests settle at 29.3–30.3°.
- **Side guard**: while turning, it checks the side it is turning toward.
  The body corner sweeps a circle of ~0.36 m radius; anything nearer than
  `TURN_SWEEP = 0.43 m` on that side stops the turn.

**`strafe(distance)`**: sideways, positive = left. No obstacle guard.

All four share one 4 Hz sampler for the depth checks (`depth.sampled`), and
the camera only sees ±45°: something directly beside or behind him is
invisible to the turn guard.

**`steer(control)`**: the closed-loop version. `control()` is called every
cycle and returns `(vx, wz)` to keep going or a string to stop. Same guards,
same clamping to `MAX_SPEED = 0.6 m/s` and `MAX_YAW_RATE = 1.6 rad/s` (a
commanded rate: he delivers 0.7 of it, section 12; it was 0.8 until that was
measured), same
guaranteed halt. Person following, teleop and the web joystick are all
built on `steer()`.

### 5.5 Emergency stop

Ctrl-C alone is not a stop once Nav2 is driving: Nav2's `controller_server`
is a **separate process** publishing its own `/cmd_vel`, so zero commands from
our process just race it. `estop()` therefore does, in order:

1. Ignore further Ctrl-C (so a second press cannot abort the stop halfway).
2. Cancel any active Nav2 goal.
3. Zero-velocity burst.
4. **Kill the Nav2 stack** (the step that matters).
5. Another zero burst, now that nothing else is publishing.

`estop(disarm=True)` also stops the heartbeat (robot then ignores all network
commands). That is opt-in because what a *standing* robot does when the
interlock drops has never been observed. `demos/estop_nav.py` fires a real
SIGINT mid-goal and measures the coast; it measured **0.000 m**.

### 5.6 Voice pass-through

`bot.say(text)` and `bot.play(clip)` forward to `talk.Voice`, imported
lazily so `lite3.py` still loads if the audio side is missing.

### 5.7 Command line

```
python3 -m robot.lite3 status | stand | sit | walk 0.5 | turn 90 | scan |
                       nav-start | nav-stop | goto 1.5 | cost | estop | action dance
```

---

## 6. Seeing: depth camera and sonars

### 6.1 Depth camera → obstacle ranges (`robot/depth.py`)

The RealSense D435i publishes a point cloud (`/camera/depth/color/points`),
already thinned to one point per 5 cm cube by our camera patch. `scan()` turns
it into "nearest obstacle per direction":

1. **Frame conversion.** Points arrive in the camera *optical* frame (x right,
   y down, z forward). Convert to a camera body frame (x forward, y left,
   z up): `(zo, -xo, -yo)`.
2. **Undo the 20° pitch.** The camera is mounted nose-down by 20°
   (0.349 rad, from the vendor's launch file). Rotate about y by that angle and
   add the mounting offset (0.255 m forward, 0.072 m up) to get `base_link`
   coordinates. Forgetting the pitch makes the floor look like a wall ~0.54 m
   ahead.
3. **Height filter.** `base_link` is 0.33 m above the floor when standing.
   Keep only points between 8 cm and 60 cm above the floor: drop the floor
   itself and anything higher than the robot (table tops it can walk under).
4. **Bin by bearing.** ±45° field of view in 5° bins (18 bins); keep the
   nearest range in each. Every other point is sampled for speed.

Returns `[(bearing_deg, range_m or None), ...]`, positive bearing = left.

Derived checks:

- **`clearance(half_width=0.225)`**: nearest obstacle *in the robot's path*.
  It uses **lateral offset** `r·sin(bearing)`, not raw range, so a wall to the
  side doesn't count as "ahead". Returns forward distance `r·cos(bearing)`, or
  `inf`. Measured from body centre; the nose is ~0.33 m ahead of that.
- **`side_clear()`**: nearest obstacle in the left half and right half, for
  the turn guard.
- **`scan_text()`**: ASCII bar chart for humans.

### 6.2 Sonars → Nav2 (`robot/sonar_range.py`)

The robot has front and rear ultrasonic sensors. `Jetson2Motion` publishes
them as bare numbers (`Float64`) at ~160 Hz; Nav2's range layer needs
`sensor_msgs/Range` in a TF frame. This small node:

- republishes both at 20 Hz as `/sonar/front` and `/sonar/rear`;
- publishes static transforms `base_link → sonar_front` (x +0.23 m) and
  `sonar_rear` (x −0.31 m, rotated 180°);
- stamps each `Range` with the latest odometry timestamp, because the robot's
  TF uses the steady clock and a mismatched stamp makes the transform fail;
- clamps readings: the front is cut to **0.8 m max**, the rear keeps 4.0 m.
  A reading at or past the maximum is published *as* the maximum, which the
  range layer reads as "nothing there".

Measured 2026-09-27/28 with tape and the depth camera: readings are good to
~2 cm; 0.28 m is the minimum ("something within ~0.3 m") and, seen lying
down, also "no echo", which is why `Lite3` exposes the rear reading without
using it as a guard; 4.5 (sometimes ~4.7) means no echo; a tilted surface can
deflect the pulse and read far. Objects ~1 m away at ±34° are caught
intermittently and at 27–32° steadily, hence the ±33° beam. The front sonar's
position comes from the camera-sonar offset; the rear is not measured and is
placed at the back of the body (nose 0.274 m ahead of `base_link`, body
0.61 m long).

Why cut the front? The beam was measured at ~±33° wide. Nav2's range layer
spreads a reading over the whole cone, so a chair 30° off to the side at 1 m
became a phantom wall across the aisle. The depth camera handles that range
well; the front sonar is kept for what the camera misses up close (< ~0.6 m,
glass). The rear sonar is the *only* sensor behind the robot, so it keeps its
full range.

It writes `/tmp/sonar_range.ready` on first publish, and
`start_nav2_mapless.sh` waits for that file before launching Nav2, otherwise
Nav2 complains "No range readings received" on every start.

---

## 7. Navigation: mapless Nav2

`robot/nav.py`, plus the vendor's `dr_nav2_mapless` package with our config
patch (`env/nav2-mapless-lite3.patch`).

### 7.1 What "mapless" means

There is no pre-built map and no SLAM. Everything is in the **odom frame**
(leg odometry), and the costmaps are built live from sensors:

| | Global costmap | Local costmap |
|---|---|---|
| Size | 30 × 30 m | 8 × 8 m (rolling) |
| Resolution | 10 cm | 10 cm |
| Layers | STVL (depth camera) → range layer (sonars) → inflation | same |

- **STVL** (Spatio-Temporal Voxel Layer) stores depth-camera points in a 3D
  voxel grid that **decays** over time. We raised decay from 2 s to **15 s**:
  at 2 s, obstacles beside the robot (out of the camera's narrow view) were
  forgotten and it scraped a chair while turning. 5–15 s is the range the
  STVL author recommends
  ([STVL README](https://github.com/SteveMacenski/spatio_temporal_voxel_layer)).
- **Range layer**: the sonars (§6.2).
- **Inflation**: 0.45 m (was 0.30; at 0.30 the planner failed next to box
  corners in 2 of 3 runs).
- Planner **NavFn**; controller **DWB**. Goal tolerance 0.25 m in xy and
  **3.14 rad in yaw**, i.e. "any heading". Letting Nav2 rotate a legged robot
  at the goal made it step and drift around the goal instead of stopping, so
  `goto()` does any final heading itself with `turn()`.

### 7.2 Starting and stopping Nav2 safely

`nav_start()`:

1. Refuses unless standing **and the pose has settled** (hasn't moved 2 cm in
   1.5 s). Standing up jumps leg odometry by ~1.3 m in one step; a costmap
   built across that jump shows a clear path straight into a real wall.
2. Kills any leftover stack.
3. Launches `env/start_nav2_mapless.sh` via **`setsid`**, so the whole stack
   is in its own process group.
4. Waits for `bt_navigator` and for a costmap to arrive.

`nav_stop()` kills by **process group** (SIGINT → SIGTERM → SIGKILL). Why not
`pkill`? Nav2's child processes don't carry the launch name, so killing by name
misses them; widening the pattern hits `static_transform_publisher`, which
the camera and transfer services also use, and ROS launch then takes *those*
services down. So: find process groups containing a Nav2 marker, exclude any
group containing a system-service process (`NAV_NEVER`), kill the rest.
`NAV_NEVER` matches executables, not words, and launch arguments (`foo:=bar`)
are dropped before matching: the Nav2 launch line itself contains
`launch_realsense:=false`, and a bare "realsense" once made the guard exclude
the very group it was meant to kill. A
group counts as gone once only zombies are left in it: the dead leader of a
stack this process launched stays a zombie, and waiting on it cost 8 s a stop.

### 7.3 Reading the costmap and choosing goals

- `cost_at(x, y)`: cost of one cell (0 free … 99 lethal, `None` off-map).
- `cost_ahead(out_to, step, settle)`: costs along the current heading. With
  `settle=N` it re-reads until two profiles a second apart agree. This is
  needed because the costmap starts empty and, with `track_unknown_space`
  off, **unseen cells read as free**: an immediate read says "clear forever",
  you pick the furthest cell, and `goto()` then refuses it as LETHAL once
  real observations land. The giveaway is cost 0 at distance 0, the robot's
  own cell, which is never really free on a populated map.
- `nav.free_ahead(profile)`: how far such a profile stays plannable, i.e. the
  last distance before the first cell `goto()` would refuse. Free means below
  LETHAL, not "low": 50 was tried first and wrongly refused open floor between
  two tables, whose inflation spreads 50–90 over the whole gap.
  `nav.goal_ahead(profile)` is that distance minus a margin, or `None` when
  too little is left; the demos and the tour's `approach` pick goals with it.

### 7.4 `goto(forward, heading_deg=None)`

1. Checks standing, battery, Nav2 running, goal ≥ 0.2 m away (a goal at his
   own position gives the planner nothing to plan and aborts; use
   `turn_deg()` to rotate in place).
2. Computes the goal `forward` metres along the current heading.
3. **Refuses a LETHAL goal cell**: NavFn cannot plan into one and would abort.
4. Sends a `NavigateToPose` action goal, stores the handle so `estop()` can
   cancel it, waits for the result.
5. If it succeeded and a heading was requested, corrects it with `turn()`.

Returns the action status: 4 = succeeded, 6 = aborted, 5 = cancelled.
`goal_send(gx, gy, gyaw)` is step 4 on its own: a goal at an odom point,
returned as soon as Nav2 accepts it, replacing any goal under way. No checks.
`goal_active()` says whether Nav2 is still working on it.
`person.follow_nav()` uses it to keep moving the goal.

`goto_cancel()`, from another thread, ends a goal and leaves Nav2 up; `estop()`
ends it and kills Nav2. Either way `goto()` returns 5 and he ends halted.

`bin/tour.py` adds a smarter goal chooser: if the requested spot is inside an
obstacle, pick the *closest free distance* on that line (Nav2 then routes
around), and an `approach` verb that stops short of the *first* obstacle.

---

## 8. Finding and following people

`robot/person.py`. The motion computer already runs a person detector
(YOLOv5s on the RK3588's NPU, with tracking; it's what the phone app's
"follow" button uses). We decoded its UDP protocol from a capture of the app:
a 12-byte header `<code, json_length, 1>` then JSON, on port 43901.

- **`PersonDetector`** (a thread) switches detection on and receives
  ~27 target lists/s: `{"targets":[{"id", "bbox":[x1,y1,x2,y2]}]}` in
  1280×720 pixels. It locks onto the **largest box's track id** (usually the
  nearest person) and keeps following that id, re-locking only if it vanishes
  for over a second. Detections older than 0.7 s are ignored. With nobody in
  view the tracker sends nothing at all (measured 2026-10-06), so the detector
  also sends a state query (`0x21013305`) and takes its answer as proof the
  tracker is there; only no answer of either kind in 5 s is an error.
- We **don't use the built-in follow**: it drives at up to 1.0 m/s with no
  depth check. Driving goes through `bot.steer()` instead.
- **The controller** (`_controller`), every 50 ms:
  - turn rate = `−5.0 × (x − 0.5)`, clamped to 1.6 rad/s commanded, which
    is about 1.1 rad/s (64°/s) delivered (a proportional controller keeping
    the person centred in the image). It was 2.4 and 0.6 until 2026-10-06:
    0.6 commanded is 24°/s, far too slow to keep up with someone walking past;
  - but don't turn toward a side with something inside `TURN_SWEEP`;
  - walk forward while the person is within the middle half of the picture
    (`|x − 0.5| < 0.25`), turning as he goes. It was 0.12, which had him stop
    to re-aim at every sidestep; the job is keeping up, not pointing;
  - slow down linearly over the last 0.8 m before `stop_distance` (clearance
    is only sampled at 4 Hz, so full speed would overshoot), min 0.12 m/s;
  - no turning at all for an offset under 0.10: a small velocity is still a
    velocity, and any velocity keeps him stepping;
  - standing in front of the person in `follow()`, he stays put until they
    are 0.5 m beyond the stop distance or 0.25 off-centre, and once he turns
    he turns until they are within 0.10 again. Without those margins he
    shuffled on the spot, chasing depth and detection noise (2026-10-06);
  - if the person is lost: when they were last seen off-centre they walked
    out of the picture on that side, so turn that way at 1.0 rad/s commanded (half a
    turn in the 5 s below); lost near the middle, stand and wait;
  - stop if the person is lost for 5 s (`approach()`) or 15 s (the follows:
    close up the tracker often loses a person, seeing only legs, and a follow
    that ends because they stood near him for 5 s is no use), or the depth
    stream is lost.
- `approach()` ends at the stop distance; `follow()` holds position there,
  keeps turning to face the person, and walks again when they move away.
  `approach(near=fn)` calls `fn` ~1 m before arrival, which the tour uses to
  start writing its spoken line early so it can speak the moment it arrives.
  Both take `abort=fn`: a reason returned from it ends the move at the next
  cycle, which is how the panel's Cancel and E-STOP reach them.
- **`follow_nav()`** is a hybrid, for a room with furniture. It walks
  straight at the person with `follow()`'s own controller (quick: a decision
  every 50 ms, the speed asked for) while the depth camera shows the way to
  them free, and hands over to Nav2 goals while it does not. Nav2 alone was
  tried first and was slow: about 3 s to get going after each goal and
  0.3–0.4 m/s once moving (2026-10-06). "Free" means nothing more than 0.5 m
  nearer than the person, either straight ahead of him or within 0.3 m of
  the line to them. The costmap was tried for this and called a free line
  blocked at once and kept flipping near the person: it holds their own
  trail and their inflated outline. The way has to be blocked for 0.3 s
  before Nav2 takes over and free for 1 s before he walks straight again.
  What follows is the Nav2 part:
  `follow()` only ever steers straight at the person, `follow()` only ever steers straight at the person, so a
  chair on that line stops him for good. Every cycle it works out where the
  person is: bearing = `−(x − 0.5) × CAM_FOV` from the tracker's box; range
  from two sources. The box's height gives a rough one (`box_range()`: a
  1.87 m person (`PERSON_HEIGHT`, set to who he follows most), picture 73°
  top to bottom, so about 10 % off for someone 1.7 m, and nothing when the box is cut off because they are close). The
  depth scan gives an exact one, but of the nearest thing at that bearing,
  which among furniture is often a chair: one trace had a 1.9 m person read
  as 0.52 m, which ended the goal (2026-10-06). So the depth return nearest
  the box's estimate is used if one is within 35 % of it, else the box's
  estimate itself. It keeps a Nav2 goal
  (`goal_send()`, section 7.4) 0.6 m short of them on that line, replaced
  when they have moved 0.5 m (at most once a second) or every 2 s. Where they
  are is smoothed over a few fixes: single fixes jumped by 0.5 m and more,
  and each new goal makes Nav2 plan again, which was most of his hesitation
  (five goals in 1.4 s in one trace, 2026-10-06). Nav2 plans round what is in its
  costmap. Things to know:
  - beside him (more than 40° round, outside the depth camera's view) they
    get no goal: Nav2's costmap cannot see them either, and a goal there
    could walk him into them. A goal under way is dropped (it leads to
    where they were) and he turns to them;
  - a goal under way also survives a missed reading. The first version
    cancelled it on one, and stood (2026-10-06);
  - within 0.9 m (stop distance + Nav2's 0.25 m goal tolerance) he stands
    until they are 1.1 m away;
  - standing with no goal, he turns to them only once they are more than 30°
    round, at up to 1.2 rad/s commanded (0.84 delivered), and stops at 15°: enough to keep
    them in the depth view, not a correction at every arrival. He has no goal
    once Nav2 reports the last one finished (`goal_active()`); before that
    was checked he never turned after arriving, stood side-on and lost them;
  - a goal cell that is LETHAL or off the map is pulled back toward him in
    0.1 m steps to the first free one (NavFn would abort otherwise): the
    person's own trail stays in the costmap for a while after they move on;
  - if the person is lost while a goal is under way he finishes it, which
    brings him to where they were last seen, then gives up after 5 s;
  - something between him and the person at the same bearing reads as the
    person: he then comes to rest beside it until he sees them clear of it;
  - the speed argument is for the straight part; Nav2 sets its own pace;
  - the result's reason ends with how many goals were sent and the seconds
    spent in each state (`straight: walking / turning / standing / not seen`, `goal`, `with them`, `beside him`, `not seen`,
    `goal blocked`, `no range on them`), and every change of state is written to
    `/tmp/follow_nav.log`. Read that first when he stands about.

---

## 9. Talking: speaker, TTS and Gemini

`robot/talk.py`, three layers.

### 9.1 `Voice`: one sound at a time

The speaker is on the **motion computer**, so every sound is a pipeline
ending in `ssh ysc@192.168.1.120 aplay`:

```
text ─► Piper TTS ─► ffmpeg (to raw S16LE 48 kHz stereo, + 800 ms silence) ─► ssh ─► aplay ─► ES8388 ─► speaker
```

- `play('OKstandup')`: one of the robot's ~33 built-in clips; plays entirely
  on the robot.
- `say(text)`: Piper (neural TTS on the Jetson), through `Speaker` below, so
  it sounds like a persona's reply. No Piper, no speech: there is no fallback.
- `play_file(path)`, `play_url(youtube_url)` (via `yt-dlp`).

Details that were learned the hard way:

- `aplay` must target the hardware device `plughw:0,0`. The default device is
  a null sink, where `aplay` "succeeds" silently.
- The amplifier un-mutes when a stream starts and swallows the first
  moments, so 800 ms of silence is prepended (`LEAD_IN_MS`). A 1.1 s clip was
  completely inaudible without it.
- Raw samples, not WAV, go down the pipe, so there is no header to misread.
- Loudness (2026-09-18): the codec's PCM volume is set to max (192 = 0 dB,
  was 180 = −6 dB) each time the speaker opens, and speech gets a software
  gain of 1.6 with a limiter. Pushing the amp itself past 0 dB distorts on
  this small speaker.

What it needs: passwordless SSH from the Jetson to the motion computer; `ysc`
in the `audio` group there (done 2026-09-15); `ffmpeg` on the Jetson; Piper in
`~/piper` (the standalone aarch64 release, voices as `~/piper/<name>.onnx`);
for `play_url`, standalone `yt-dlp` and `deno` binaries in `~/.local/bin`;
for Gemini, a key in `~/.gemini_key`, `websockets` 13.1 (the last release for
Python 3.8) and internet. The robot has its own internet (eduroam via the
motion computer). If it ever has none, run an HTTP CONNECT proxy on the
laptop, forward its port over SSH and set `TALK_PROXY=http://localhost:3128`.

### 9.2 `Speaker`: streaming, sentence by sentence

One long-running Piper process (model load is ~1.4 s) writes one WAV per
sentence; a pump thread feeds them into a single open `ffmpeg | ssh aplay`
stream. The first sentence plays while the next is still being synthesised.
It adds software gain with a limiter and sets the codec volume to max.

### 9.3 `Talker`: Gemini answers, in its own voice

- **Gemini Live** (default): one WebSocket session that keeps its own
  conversation context. Live models answer in speech, with a transcript
  alongside. **The speech is what you hear**: `Line` plays the samples
  (24 kHz) on the robot's speaker as they arrive, and the transcript only
  feeds the chat display. Session resumption handles dropped idle
  connections; if no reply starts within 6 s it reconnects once.
- **Why not Piper for replies** (it was, until 2026-10-07): the wait was on
  our side, not Gemini's. Measured that day on the Jetson: Gemini's whole
  reply in 0.5 s, then Piper 1.7-2.0 s to make each sentence, then 0.8 s of
  lead-in silence, about 3 s before a sound; the first question after a
  start also paid 3.9 s to load Piper and 1.9 s to open the session. Now
  `Line` is opened before the question is sent, so the ssh to the speaker
  and the lead-in pass while Gemini thinks, and the panel loads everything
  at start: **sound begins 1.0-1.6 s after asking** (timed into a sink in
  place of the speaker; the real speaker adds its ssh, about 0.4 s, where
  that is not already hidden by the lead-in).
- **Voices.** 30 of Gemini's (`python3 -m robot.talk gvoices`); the default
  is `GEMINI_VOICE`, the panel's Voice box changes it, `--gvoice NAME` on
  the command line. A session keeps the voice it was opened with, so a
  change reopens it. Rocky's alien filter is applied to whichever voice.
- **Piper still speaks** what is not a Live reply: "Say it" (exact text),
  "Look and comment" and its follow-ups (REST, below), and any reply that
  fell back to REST. Those are in his old voice, so the two differ.
- **REST** (`generateContent`) is the fallback if Live fails. It keeps the
  last 20 messages and re-sends the most recent camera frame so follow-ups
  like "what did you just see?" work. **Image turns always go over REST**,
  and so do follow-ups while the look is still in the history: measured
  2026-10-05, Live answered about a frame it had plainly not read (invented a
  person, misread text). The free tier sometimes stalls or 503s while an
  immediate retry answers in ~1 s, so requests time out at 12 s and retry
  three times. The model name is an alias (`gemini-flash-lite-latest`) so a
  retired version cannot break it, and "thinking" is turned down: it adds
  seconds a spoken one-liner does not need.
- `sentences()` regroups streamed text chunks into whole sentences so
  speaking starts before the reply is complete.
- `look(question)` grabs one 1280-px frame from the front camera (RTSP via
  `ffmpeg`) and asks about it.
- **Personas** (system prompts + voice): `deadpan` (default, safe for
  visitors; jokes about the situation), `sarcastic` (teases people, with hard
  limits: PG-13, never about looks or personal traits), `rocky` (Rocky from
  *Project Hail Mary*, with an "alien" audio filter), `observer` (neutral, no
  jokes; just says what is there). Private personas can go
  in the git-ignored `persona_private.py`.
- The API key is read from `~/.gemini_key` (git-ignored).

CLI: `python3 -m robot.talk` (chat), `... ask "hi"`, `... look`, `... say "text"`,
`... play OKstandup`, `... url <link>`, `... clips`, `... voices`, `... gvoices`.

---

## 10. Programs you run

### 10.1 `bin/tour.py`: walk a route and comment on it

```
python3 bin/tour.py "walk 1.5, turn 90, goto 2 -90, person, follow 60"
```

Verbs: `walk <m>`, `turn <deg>`, `goto <m> [<deg>]` (Nav2), `approach <m>`,
`person` (walk up to the nearest person), `follow <s>`. Flow: heartbeat →
wait for the interlock → stand → (if needed) wait for the pose to settle and
start Nav2 → for each step: move, stop, **look and talk** (it only talks while
standing still; the motors are too loud while walking) → stop Nav2 → sit →
drop heartbeat. A failed step is reported and skipped. `--finale "..."` ends
with a custom line, optionally tilting the body up (`--look-up 12`) so the
camera sees a face. An `Odometer` thread sums the actual path length for the
closing sentence (summing start-to-end per step counted a 14 m follow as
7.2 m).

`walk` steps add half a second of travel to `--stop`, because `walk()` samples
the depth camera at 4 Hz and has no slow-down ramp. When the last step is
`person`, the finale line is written during the last metre so he can speak on
arrival (frame grab plus Gemini take 3–4 s), but only without `--look-up`:
that frame is taken before the tilt and shows knees and shoes, so with
`--look-up` the finale is done live after tilting, at the cost of ~3 s of
silence.

### 10.2 `bin/teleop.py`: keyboard driving over SSH

`w/s/a/d`, space to stop, `+/-` speed. Keys are momentary: it stops 0.5 s
after the last keypress, so a dropped SSH connection stops the robot.
**No obstacle guard**. `--selftest` checks the key mapping without a robot.

### 10.3 `robot/hmi.py`: web control panel

```
python3 -m robot.hmi     # then open http://lite3-perception:8080
```

An `aiohttp` server bound **only to the Tailscale VPN address**, so nothing on
the public Wi-Fi can reach it. One self-contained page (phone or laptop, no
CDN: even its typeface, B612, is embedded) with:

- live status (battery, posture, Nav2, camera, heartbeat, sonars, tilt, pose)
  pushed over a WebSocket twice per second, and the depth scan drawn as a
  top-down fan;
- the robot **in 3D**, live (see "Him, in 3D" below);
- two camera views (front wide-angle, RealSense colour) as H.264 over WebRTC,
  running **only while someone is watching** (see "Cameras" below);
- stand/sit, Nav2 on/off, camera/voa on/off, "goto N m";
- a **hold-to-drive** joystick (and WASD) built on `steer()`: the page sends
  the stick ~10×/s and the robot halts 0.3 s after the last message (a
  "dead-man" switch: released, tab closed, Wi-Fi lost → stop). Forward stops
  at camera clearance ≤ 0.6 m, backward at rear sonar < 0.5 m. Driving is
  refused while Nav2 runs;
- **person detection** on/off (Systems): the built-in tracker of section 8,
  looking only. The row says whether someone is in view and on which side,
  and the box is drawn on the front camera view, which is the camera the
  tracker looks through. The switch itself only looks;
- **Walk up** and **Follow** (Navigate): `person.approach()` and
  `person.follow()` of section 8, stopping 0.6 m short. They switch detection
  on if it is off (with Nav2 on, Follow is `follow_nav()` and goes round
  obstacles), and walk at the Drive section's top-speed slider as it is
  when pressed (a Go does not: Nav2's speed is in its own config). One press
  of Follow lasts 30 s, the ceiling on any single
  move. Cancel and E-STOP end either within one 50 ms cycle (the `abort`
  argument of both functions), and so does the page going away, as for a Go;
- say / ask / look with a persona picker, all three in that persona's voice;
- an always-visible **E-STOP** (also the space bar) that bypasses the command
  queue and calls `Lite3.estop()` directly;
- **Cancel**, shown while a Go is under way: ends it and leaves Nav2 running.
  A Go does not outlive its operator either: 3 s after the last page went
  away it is cancelled, and if Nav2 will not let go, e-stopped.

Robot commands run in a single-worker thread pool so only one thing moves the
robot at a time; talking has its own pool so it can happen alongside. On
shutdown it sits the robot down before dropping the heartbeat.

The timings: the drive halts 0.3 s after the last stick message (`DEADMAN_S`);
a silent page is noticed within about 7.5 s (`PING_S` × 1.5); a Go is cancelled
3 s after the last page went (`GONE_S`, the page itself reconnects in 1.5 s)
and becomes an E-STOP if Nav2 has not let go 2 s later (`LET_GO_S`). A
malformed message from a page is logged and ignored. A browser button is not
a hardware stop: keep the handheld at hand. For HTTPS, run
`sudo tailscale serve --bg 8080` once and open
`https://lite3-perception.<tailnet>.ts.net`; the server also listens on
localhost for that.

**Him, in 3D.** A third monitor under Cameras: the robot model posed from
his joint angles, on a floor grid he walks across, with the two sonar beams
and the depth scan of "Ahead of him" laid on the floor. Drag to look around
him, scroll to move closer; on a phone a sideways drag turns the view and an
up-or-down one still scrolls the page.
- The browser does the drawing. It reads the same `urdf/lite3.urdf` and
  small meshes as rviz (served under `/model`, 3.4 MB, once) with a short
  URDF reader in the page, and `three.js` (r160, MIT, 670 kB) from
  `hmi_static/lib/`, kept in the repo because the panel must work with no
  internet. Both are fetched only when the monitor is opened.
- The robot's part is small: `hmi.py` listens to `/joint_states` (10 Hz) and,
  only to pages with the monitor open, sends the 12 angles, the pose and the
  body height (`rviz.height()`, the same sum rviz uses) 10 times a second,
  about 150 bytes each. Sonars, tilt and scan come from the status message
  the page gets anyway. The page eases between messages so he moves, not
  jumps.
- **What he intends** is drawn on the floor: Nav2's path as a line, its goal
  as a ring with a stroke the way he will end up facing, and the person he
  sees as a column of a person's size. They come in the status message
  (`goal`, `plan`, `them`, all in odom) and go when the goal is over or the
  person is no longer seen. The person's place is one fix from the box in
  the front camera and the depth scan (`person.where()`), good to about
  half a metre, and only as fresh as the scan (1 Hz). The path is Nav2's
  `/plan` thinned to about 40 points.
- The panel also puts the goal and the person on `/intent/goal` and
  `/intent/person` (2 Hz, while there is one) so rviz and recordings get
  them: section 10.6.
- Not in it: the full depth cloud and the costmaps. Those cost the robot
  real work per viewer; rviz on the laptop (section 10.6) has them.

**Cameras.** Both views come from the motion computer's mediamtx as WebRTC.
The front camera is the robot's own H.264 stream (1280×720, 30 fps). The
RealSense colour image is put there by `robot/rs_stream.py`, which runs only
while a page shows that view and for 10 s after. A browser cannot reach the
motion computer, so `hmi.py` rewrites the WebRTC answer to point at
`robot/udp_relay.py` on the Jetson, which forwards the video. If WebRTC does
not connect, the front camera falls back to MJPEG (ffmpeg, 640 px, 8 fps); the
RealSense view has no fallback.

Why they are built this way (all measured 2026-10-05, at 1280×720):

- `rs_stream.py` reads the camera's **JPEG** topic (needs
  `ros-foxy-compressed-image-transport`): raw frames do not get through ROS
  into Python fast enough, a subscriber that did nothing received 8 of 30 a
  second.
- It uses the Jetson's **hardware** H.264 encoder: libx264 ultrafast managed
  23 fps on 1.5 cores. But it decodes with `jpegdec`, not `nvjpegdec`: the
  hardware JPEG decoder kept handing out its first frame, so the video was a
  still. Baseline profile (WebRTC cannot take B-frames) with a keyframe a
  second, so a new viewer starts at once.
- Both are **processes of their own**. Encoding inside the HMI stuttered
  (22 fps with 150–200 ms gaps against a steady 29 fps). The relay inside the
  HMI shared the interpreter with the ROS threads, was starved and dropped
  packets (its receive queue was full).

### 10.4 `demos/`

- `demo.py`: ten lessons from "connect" to "run with no handheld". Lessons
  that move the robot need `--move`. Lesson 4 is worth showing: it tries unsafe
  things and prints the API's refusals.
- `shake.py`: end-to-end regression: stand, walk, turn ±90, Nav2 goal, sit,
  summary table.
- `estop_nav.py`: sends a real SIGINT 6 s into a Nav2 goal and measures the
  coast.
- `check_motion.py`: what the motion code promises (a halt on every way out,
  clamping, the time ceiling, handheld takeover, the ways a Go ends, a quick
  `nav_stop()`) against a fake robot and a pretend Nav2. Sends nothing, so it
  runs anywhere; run it after changing the motion code.

### 10.5 `robot/protocol.py` as a CLI

`python3 -m robot.protocol stand_toggle`, `... dance`, `... 0x21010300`,
`... camera on`, `... voa off`. Sends a raw code with **no checks**; for
debugging and capturing unknown codes only.
### 10.6 `robot/rviz.py`: the live robot in rviz2 on the laptop

    python3 -m robot.rviz                # on the laptop, in ~/lite3_api: the model only
    python3 -m robot.rviz sensors        # plus depth cloud, sonars, costmaps, Nav2 path, goal, person
    python3 -m robot.rviz /some/topic    # plus any topics you name
    python3 -m robot.rviz --record walk.rec sensors    # the same, and kept in walk.rec
    python3 -m robot.rviz --play walk.rec              # walk.rec again, with no robot
    python3 -m robot.rviz --play walk.rec 0.25         # at quarter speed (2 = twice as fast)

Opens rviz2 with the Lite3 model moving its legs as the robot does. Close
rviz (or Ctrl-C) to stop everything, including the robot end.

**How.** The laptop starts `python3 -m robot.rviz --send ...` on the robot
over ssh. That end subscribes to `/joint_states`, `/tf`, `/tf_static`, `/leg_odom2` and
the topics you name, and writes the messages, undecoded, down the ssh pipe;
the laptop end republishes them unchanged. `robot_state_publisher` on the
laptop turns `/joint_states` plus `urdf/lite3.urdf` into the leg frames.

**Record and replay.** `--record FILE` writes every message that comes down
the pipe to FILE, each with the time it arrived, while showing it as usual.
`--play FILE` feeds rviz from FILE with the same timing and no connection to
the robot, so a walk or a failed follow can be looked at again afterwards,
slowed down if need be. What is in the file is what was relayed: name the
topics you will want when recording (`sensors`, a costmap, ...).
- Size: the model alone is about 10 kB/s; with `sensors` the depth cloud
  dominates, 3.5 MB for 23 s on a relayed link (2026-10-07), more on a
  direct one. `*.rec` is in `.gitignore`.
- A replay runs once and then leaves rviz open. The messages keep the
  robot's own time stamps, and rviz ignores frames older than ones it
  already has, so looping in the same rviz would show a frozen robot. Run
  it again to see it again.
- There is no pause or step. Slow it down with the speed number.
- It replays into rviz exactly what the live view does, the laptop-made
  `odom -> base_link` and sonar cones included, because those are made
  from the recorded messages on the way through.

**Why not plain DDS.** The laptop only reaches the robot over Tailscale,
which carries no multicast, and the robot's Foxy CycloneDDS talks on one
interface only (`eth0`, which the bridge to the motion computer needs).
Moving it to Tailscale means reconfiguring every node on the robot. The
pipe needs nothing changed on the robot and no open ports.

**The model.** `urdf/lite3.urdf` with the small meshes in `urdf/meshes`
(3.4 MB, in git) always works. On its first run the laptop end also
downloads the vendor's dense meshes (about ten times the triangles, 57 MB)
from `DeepRoboticsLab/Lite3_rl_training` and Intel's D435 mesh (16 MB, in
place of the grey box where the camera sits) into `urdf/meshes_hd/`, which
git ignores, and uses those from then on. The vendor publishes no Lite3 Pro
model, so the computer box on the back is not shown. No network on that first run: it says
so and shows the small ones. Delete the folder to go back to them.

**What to expect.**
- `sensors` also shows **what he intends**: Nav2's path (`/plan`, and the
  controller's `/local_plan`), its goal as an arrow on the floor and the
  person he sees as a column. Goal and person come from the panel
  (`/intent/goal`, `/intent/person`), so the panel has to be running; the
  laptop end draws them as markers that fade 1.5 s after the last one, so
  a reached goal or a lost person disappears. (Markers are made on the
  laptop because the Marker message changed after Foxy and would not
  survive the relay.)
- `sensors` shows what fits the 3D view: the depth cloud; the two sonars
  as cones (made on the laptop from
  the bare readings, the way `robot/sonar_range.py` does on the robot);
  the global and local costmaps once Nav2 is up. There is no lidar on this
  robot (`/rslidar_points` has no publisher) and the IMU has no display.
- Each topic is capped at 20 Hz (`MAX_HZ`), messages over 20 kB (the
  cloud) at 5 Hz; latched topics (`/tf_static`, costmaps) are passed
  whole. A topic that appears later (Nav2 started after rviz) is picked up
  within 2 s.
- The link decides the rest. When Tailscale cannot connect the laptop and
  robot directly it relays through a server (`tailscale status` says
  `relay`), and everything shares roughly 300 kB/s. Each topic then waits
  four times as long as its last message took to send, so a big topic
  cannot starve the model. Measured 2026-10-07 on a relayed link, with the
  160 kB camera picture also going (since removed): cloud 2 Hz, sonars
  11 Hz.
- Every extra topic is one more subscriber on a loaded Jetson (section 12).
  The depth cloud is the heavy one: name it when you want it, not always.
- A message type the laptop does not have (`transfer_interfaces`, the
  RealSense extras) is skipped with a line saying so.
- Fixed frame is `odom`, so the grid is the floor and the robot walks and
  turns across it. The laptop end makes `odom -> base_link` itself: x, y
  and heading from `/leg_odom2`, height from the joint angles (the lowest
  foot, knee or the belly rests on the floor, body taken as level). The
  odometry's own height stays at standing height (0.32 m) when the robot
  lies down, which left the model floating. The robot's own
  `odom -> base_link` (Nav2's `odom_to_tf.py`) is dropped for that reason.
  `python3 -m robot.rviz --check` tests the height sum.
- Time stamps are the robot's steady clock, untouched, so the cloud and the
  frames agree with each other but not with the laptop's clock.
- The laptop end runs with `ROS_LOCALHOST_ONLY=1` and without the laptop's
  own `CYCLONEDDS_URI`, so nothing leaves the laptop. To use `ros2 topic`
  beside it, set the same two things in that shell.
- After a cold boot (basic state 98) the thigh angles the robot reports are
  not yet meaningful: lying in the ready pose it reports about ±0.3 rad and
  the model's thighs hang straight down. Once it has stood up the angles
  are right (standing: thigh 0.67, knee -1.35 on `/joint_states`).
- If the link drops (the laptop changes network), the laptop end notices
  within about 6 s and reconnects every 2 s until the robot answers.

---

## 11. The environment and our patches to vendor code

### 11.1 `env/lite3_env.sh`

Source it before anything. It sets CycloneDDS, puts the repo root on
`PYTHONPATH`, and sources five ROS workspaces (ROS Foxy, RealSense, Nav2, the
vendor nav package, the vendor transfer bridge). The RealSense workspace is
needed even when Nav2 doesn't launch the camera, because the launch file looks
the package up before evaluating the condition.

Dependencies come from these ROS workspaces, not PyPI; `pyproject.toml` is
metadata only.

### 11.2 Patches (`env/*.patch`)

The vendor code lives outside this repo, so our changes are kept here as
diffs against the vendor originals.

| Patch | What it changes | Why |
|---|---|---|
| `transfer-jetson2motion.patch` | the UDP↔ROS bridge | **caps what it republishes**: odometry, sonars, state and handheld at 50 Hz, joint states at 10 Hz, instead of every one of the ~160 packets a second (parameters `state_hz`, `handle_hz`, `joint_hz`; 0 = every packet; the IMU stays at 160 Hz because VOA pairs it with each point cloud). Each subscriber pays per message and together they had the Jetson at 3% idle (section 12); publishes the **front** sonar (stock published only the rear); adds **battery**, error, charging to the state array (a flat battery used to be invisible: the robot just silently refused to stand); scales leg odometry x/y by **1.15**; fixes a race where every velocity reached the robot **twice** (raw and obstacle-corrected), so the obstacle avoider could never veto anything |
| `voa-lite3.patch` | vendor obstacle avoider | odometry averaging window 10 → 3 samples, the same ~60 ms now that odometry comes at 50 Hz; the idle handheld publishes zeros at ~160 Hz, which overwrote every ROS velocity; now ignored. A dead sender's last command times out after 500 ms. The config file named a node that doesn't exist, so **every parameter was silently ignored**: fixed |
| `rtsp-stream-push.patch` | front camera push (`~/rtsp_stream/push_video.sh` on the **motion** computer) | **retries**. The boot script starts the stream server and the camera push side by side; when the push got there first (2026-10-07) it failed to connect and exited for good, so the front camera had no picture until the next boot (`mediamtx` logs "no one is publishing to path 'test'"). Now it is a loop that tries again every 2 s. Backup beside the file as `push_video.sh.pre-retry` |
| `nav2-mapless-lite3.patch` | Nav2 config | range layer for the sonars, voxel decay 2→15 s, inflation 0.30→0.45 m, footprint padding 0.10→0.05, yaw tolerance "any" |
| `realsense-ros-4.58.3-lite3.patch` | camera driver | re-adds the vendor's two cloud changes to the newer driver: steady-clock timestamps (so TF lookups work) and a 5 cm PCL voxel filter |

### 11.3 Camera stack

`env/camera.launch.py` runs the newer realsense-ros 4.58.3 (with a patched
kernel USB video module) as a drop-in for the vendor's old 3.2.3 fork, with
the same topic names. It also streams colour and the camera's IMU; depth +
colour at 30 fps no longer crashes the USB bus (verified 2026-09-24). Colour
is **off unless someone has the panel's RealSense view open**: `rs_stream.py`
sets the driver's `enable_color` when it starts and clears it when it stops.
The driver takes that while running and the depth cloud carries on (longest
gap between clouds 0.24 s, measured 2026-10-07). Nothing else reads colour,
and off takes the driver from 54% of a core to 48%. Depth
stays at the vendor's 424×240, which Nav2 and voa expect; colour is 1280×720,
since it is only looked at and never crosses the Wi-Fi raw. Depth is clipped
at 5 m before the voxel grid: far points make PCL's 5 cm grid overflow its
index, and it then silently passes the whole raw cloud through. The cloud's
steady-clock stamp and the voxel filter are our patch to
`pointcloud_filter.cpp`; stock 4.58.3 ignores those two parameters.

### 11.4 Services

`camera()` / `voa()` in `protocol.py` start and stop the systemd units
directly via `sudo -n systemctl`, allowed for exactly those four commands by
`env/sudoers-lite3-camera`. `env/voa_ros2-override.conf` binds voa to the
camera: voa on brings the camera up; camera off takes voa down.

---

## 12. Measured numbers and calibration

Everything here was measured on the robot. How and when is recorded in this
README, next to the feature it belongs to; the code keeps only a one-line
reason beside each constant.

| Quantity | Value | Where it's used |
|---|---|---|
| Leg odometry under-count | ×1.15 (1.712 m by odometry = 1.960 m by tape) | Jetson2Motion patch |
| Turn coast after stop | 0.17 s (3.5–4.5° overshoot at 0.4 rad/s) | `TURN_COAST_S` |
| CPU of our ROS nodes | Jetson2Motion publishes odometry, IMU, state, both sonars and the handheld at 160 Hz each. Taking every message in rclpy cost `sonar_range` 68% of a core and a bare `Lite3` 35–46%. With queues of 1 read at a fixed rate: `sonar_range` 14% (20 Hz), `Lite3` 35% at 50 Hz, 25% at 25 Hz, 20% at 15 Hz. An empty `spin_once(timeout_sec=0)` costs as much as a full one, hence `take_ready()` (2026-10-06, Nav2 and the panel running, machine at 3–10% idle) | `lite3.SPIN_HZ`, `CLOUD_HZ`, `sonar_range.RATE_HZ` |
| Turn rate delivered | 0.7 × commanded, the same by odometry and gyro: 0.4→0.28, 0.8→0.56, 1.2→0.84, 1.6→1.12 rad/s. Starts ~0.25 s after the command, coasts 2°, 5°, 8°, 11° after the stop, body tilt under 2° throughout (2026-10-06, turning in place for 2 s each way) | `MAX_YAW_RATE`, `person.MAX_TURN` |
| Stand-up odometry jump | ~1.3 m in one step | `nav_start()` settle check |
| Toggle reaction time | ~25 ms; arming transition ~2 s | `ARM_GRACE` |
| Camera pitch | 20° nose down | `depth.CAM_PITCH` |
| Standing body height | 0.33 m | `depth.STAND_HEIGHT` |
| Body corner turning radius | ~0.36 m → guard 0.43 m | `TURN_SWEEP` |
| Nose ahead of body centre | ~0.33 m | stop distances |
| Sonar min / no-echo | 0.28 m / 4.5 m, accurate to ~2 cm | `sonar_range.py` |
| Sonar beam width | ~±33° | `FOV = 1.15 rad` |
| Pitch stick full scale | ~14° | `MAX_TILT_DEG` |
| Velocity failsafe | robot stops ~1.5 s after commands stop, ~8 cm coast | background for dead-man design |
| Battery below which it won't stand | ~20% | `BATTERY_REFUSE` |
| Speaker amp wake-up | needs ~800 ms lead-in | `LEAD_IN_MS` |
| E-stop mid-Nav2-goal | 0.000 m coast | `estop_nav.py` |

---

## 13. Design principles

These are the ideas behind the code; useful if asked "why is it built like this?"

1. **Turn silent failures into loud ones.** The robot's worst habit is
   ignoring commands without saying why. Every known case is either handled
   automatically (auto mode, posture mode exit, swallowed arming toggle) or
   raises `Lite3Error` with a message that says what to do. `demo.py` lesson 4
   calls the refusals "the docs".
2. **Guarantee the stop.** Every motion call ends in `finally: halt()`. The
   e-stop kills the *other* publisher (Nav2), not just our own. Remote driving
   uses dead-man timeouts, and a Go from the web panel is cancelled when no
   page is left to stop it. Shutdown order is always "stop moving, then
   disarm".
3. **State as attributes, not callbacks.** A background spin thread keeps the
   latest message of each topic, so behaviours read like ordinary sequential
   Python.
4. **Blocking calls that report what happened.** `walk()` returns how far it
   actually moved and why it stopped, because stopping short for an obstacle
   is normal operation.
5. **Measure, don't guess.** Constants come from tape measures, camera checks
   and logs; this README records the measurement and date.
6. **Keep ROS optional where possible.** `protocol.py` and `talk.py` don't
   import `rclpy`; `lite3.py` imports `talk` lazily.
7. **Change vendor code minimally, and keep the diff.** Every vendor change is
   a patch file in `env/` with the reasoning inside.

---

## 14. Known limitations

Worth being upfront about:

- **No map, no localisation.** Everything is relative to leg odometry, which
  drifts. Good for "go 3 m that way around the chair"; not for "go to room 204".
- **Software e-stop only.** The handheld (and the power switch) are the only
  hardware stops. With `heartbeat_start()` the handheld is not in the loop.
- **Blind spots.** The depth camera sees ±45° forward only; nothing covers the
  sides; the rear sonar is a single narrow-ish cone. Walking backwards in
  `walk()` and all of `teleop.py` are unguarded.
- **Unknown space is treated as free** in the costmap (vendor setting), which
  is why goals are chosen from a *settled* costmap.
- **`estop(disarm=True)`** while standing is untested.
- Most built-in tricks other than `dance` have not been run on this robot.
- The Gemini features need internet and an API key; the free tier sometimes
  stalls (handled by timeouts and retries).

---

## 15. Running things

```bash
source ~/robot/env/lite3_env.sh              # always first

python3 -m robot.lite3 status                # one-line state, no movement
python3 -m robot.lite3 scan                  # depth camera bar chart
python3 demos/demo.py read                   # all lessons that don't move
python3 demos/demo.py 6 --move               # walk and turn lesson
python3 bin/tour.py "walk 1.5, turn 90"      # a tour with commentary
python3 bin/teleop.py                        # keyboard driving
python3 -m robot.hmi                         # web panel on the tailnet
python3 -m robot.talk                        # chat with the robot, spoken aloud
python3 -m robot.person                      # print where the detected person is
python3 -m robot.protocol camera on          # start the depth camera service
```

In your own code:

```python
from robot.lite3 import Lite3, Lite3Error

with Lite3() as bot:
    bot.heartbeat_start()        # only if no handheld is on - YOU are now the stop
    bot.wait_ready()
    bot.stand()
    try:
        r = bot.walk(2.0, speed=0.3)
        print('moved %.2f m: %s' % (r['moved'], r['reason']))
    except Lite3Error as e:
        print('refused:', e)
    finally:
        bot.sit()
```

Scripts in `bin/` and `demos/` add the repo root to `sys.path` themselves, but
still need the ROS workspaces, so source the env file anyway. All paths are
resolved relative to the repo, so it can be cloned anywhere.

---

## 16. Things that will bite you

- **The handheld is the only physical e-stop.** `heartbeat_start()` makes this
  process the controller, so the robot obeys with no handheld at all; then the
  terminal is your stop. `Lite3` installs a Ctrl-C e-stop by default.
- **Stand before starting Nav2**, and let the pose settle (`nav_start()`
  enforces this).
- **`realsense_ros2.service` does not come back after a reboot.** Start it
  (`python3 -m robot.protocol camera on`), or anything using depth raises
  "no point cloud".
- **Only one program may hold the robot at a time.** The HMI holds a `Lite3`
  with the heartbeat running; stop it before running other scripts.
- **`transfer_ros2` and the vendor's `Lite_motion` fight over UDP 43897.** If
  `Lite3()` times out waiting for state, that's the usual cause.
- **Touching a handheld stick** silently puts the robot in manual mode; motion
  calls detect this and stop with "handheld took over".
- **`shake.py` and `estop_nav.py` move the robot.** `demo.py` needs `--move`
  for lessons 5 and up.
- Below **~0.2 m/s** the gait shuffles and drifts sideways.
- The perception computer's clock is skewed; file timestamps are meaningless.

---

## 17. Questions you are likely to be asked

**Why not just use ROS directly?**
You can, but you'd have to rediscover every silent failure: wrong DDS
implementation, not in auto mode, not standing, low battery, posture mode,
unarmed robot, Nav2 outliving your process. `Lite3` encodes all of that once.

**How does it avoid obstacles?**
Three layers. (1) `walk()`/`turn()` check the depth camera at 4 Hz and stop.
(2) Nav2 plans around obstacles in a costmap built from the depth camera
(3D voxels that fade after 15 s) and the two sonars. (3) Optionally, the
vendor's `voa` node corrects velocities in real time (we fixed it so it
actually can).

**How do you convert the point cloud into obstacles?**
Transform from the camera's optical frame to the robot body frame, correcting
for the 20° downward tilt; keep points 8–60 cm above the floor; bin by bearing;
take the nearest per bin. "In the path" is judged by lateral offset against
the robot's half-width.

**How accurate is odometry?**
Raw leg odometry under-counted by 15%; after scaling, walks agree with tape.
Turns are accurate to about ±1° after coast compensation. It still drifts
over long runs because nothing corrects it (no SLAM).

**How does person following work?**
The robot's own NPU detector gives bounding boxes over UDP. A proportional
controller turns to centre the person, walks when centred, slows near the
target, and uses the depth camera for the actual stopping distance.

**What happens if the program crashes or Wi-Fi drops?**
Motion calls always publish zero velocity on exit. Remote driving (teleop,
web) uses dead-man timeouts of 0.3–0.5 s, and a Go started from the web panel
is cancelled 3 s after its last page disconnects. Beyond that, the robot's
own failsafe stops it ~1.5 s after velocity commands stop arriving.

**Why does the robot talk only when standing still?**
The motors are too loud to hear the speaker while walking.

**Why Gemini Live but Piper for the voice?**
Live gives the fastest first words (~0.5 s) and keeps context, but only
outputs audio; we take its transcript and speak it with Piper so the robot has
one consistent voice and the audio filters (e.g. the "alien" voice) apply.

**What did you change in vendor code?**
Four patches (§11.2); the most important fixed a bug where every velocity
command reached the robot twice, which made the vendor obstacle avoider
useless, and one where its configuration was silently never loaded.

---

## 18. History

| Date | Change |
|---|---|
| 2026-09-14 | Loose scripts in `~` replaced by one API (`lite3.py`); old scripts archived |
| 2026-09-15 | Initial git import; package structure; costmap-settle fix; heartbeat in `shake.py` |
| 2026-09-18 | Posture/tilt, `steer()`, side-check on turns, person following, Piper TTS, Gemini Live, personas |
| 2026-09-21 | Arming fix in `stand()`; speaker lead-in fix |
| 2026-09-24 | `lite3.py` split into `protocol`/`nav`/`depth`; tricks; voice+rocky merged into `talk.py` |
| 2026-09-25 | New camera stack (realsense-ros 4.58), sonars, safety stops, voa fixes |
| 2026-09-27 | Camera independent of voa; Nav2 config tuned and recorded as a patch |
| 2026-09-28 | Odometry ×1.15; turn coast fix; sonars in Nav2; web HMI |

The pre-git history (`.bak` files and the superseded loose scripts) was kept
in `archive/` until commit `c4cfb84`, and `robot/run_robot.py` (`Mission`, a
scripted-run wrapper nothing used) with it: `git show c4cfb84:archive/backups/README.md`.

---

## 19. Glossary

| Term | Meaning |
|---|---|
| **jy_exe** | the vendor's closed-source locomotion controller on the motion computer |
| **basic_state** | `jy_exe`'s posture/state number (6 = standing; see §5.2) |
| **heartbeat / interlock** | keepalive packet without which `jy_exe` ignores commands |
| **transfer_ros2 / Jetson2Motion** | vendor bridge between the robot's UDP protocol and ROS2 |
| **voa** | vendor "visual obstacle avoidance" node: corrects `/cmd_vel` → `/cmd_vel_corrected` |
| **Twist / `/cmd_vel`** | ROS velocity message / topic: `linear.x` forward, `linear.y` left, `angular.z` turn |
| **odom frame** | coordinate frame fixed where odometry started; drifts over time |
| **base_link** | coordinate frame attached to the robot body centre |
| **TF** | ROS's system for transforms between frames |
| **Nav2** | the standard ROS2 navigation stack |
| **costmap** | grid of traversal costs (0 free … 99 lethal) Nav2 plans on |
| **STVL** | Spatio-Temporal Voxel Layer: costmap layer of decaying 3D voxels |
| **inflation** | costmap layer that adds cost near obstacles, keeping the robot away from them |
| **NavFn / DWB** | Nav2's global path planner / local trajectory controller |
| **CycloneDDS** | the DDS middleware ROS2 uses here (the default FastRTPS doesn't work on this robot) |
| **RMW** | ROS middleware interface; selects the DDS implementation |
| **Piper** | offline neural text-to-speech |
| **Gemini Live** | Google's streaming WebSocket model API |
| **MJPEG** | video as a stream of JPEG images; plays in any `<img>` tag |
| **Tailscale** | VPN; the HMI is only reachable through it |
| **dead-man switch** | control that stops the machine when the operator stops actively holding it |
