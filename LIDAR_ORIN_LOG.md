# Lidar, switch and Orin Nano: what we did on 2026-10-08

A step-by-step record of one working session: a switch, a lidar and an Orin
Nano were added to the Lite3, and by the end FAST-LIO2's pose reaches the
robot's own ROS graph. Written in the order things happened, including the
mistakes. How the code works is in `README.md` §10.7; this file is the
story. Logins are left out on purpose.

## Where we ended up

| Thing | State |
|---|---|
| Network | All four devices reach each other, no configuration needed |
| Lidar driver on the Orin | Running, 10 Hz cloud and 200 Hz IMU |
| FAST-LIO2 on the Orin | Installed; runs only during a Live run from its web panel |
| `robot/lio_relay.py` | Publishes `/lio_odom` and `/odom_fused` on the robot's graph |
| Nav2 and `Lite3` | Code switched to `/odom_fused` (section 10). Deployed on the robot: see section 11 |

## 1. The hardware

A TP-Link TL-SG105E switch now rides on the robot.

| Switch port | Device | Address |
|---|---|---|
| 1 | Perception computer (Jetson Xavier NX) | 192.168.1.103 |
| 4 | Livox MID-360 lidar | 192.168.1.102 |
| 5 | Orin Nano, on top of the robot | 192.168.1.5 |

The motion computer (192.168.1.120) and the perception computer are joined
by a cable inside the robot. That cable is always there; the switch does
not replace it.

## 2. Can everything talk to each other?

Goal: every device reaches every other one.

1. ssh to the perception computer over Tailscale. It stopped on a Tailscale
   check that had to be approved in a browser.
2. From the perception computer: pinged the motion computer, the lidar and
   the Orin. All answered.
3. From the motion computer (hopping through the perception computer):
   pinged the perception computer, the lidar and the Orin. All answered.
4. From the Orin, once its login was known: pinged the other three. All
   answered.

Result: nothing needed setting up. Everything is on 192.168.1.x and the
switch just passes traffic. The laptop's ssh key was added to the Orin, so
this now works without a password:

```
ssh -J ysc@lite3-perception lite3@192.168.1.5
```

Also seen: the Orin has its own campus Wi-Fi and its own Tailscale address,
which is not in the laptop's tailnet list.

## 3. Is the lidar publishing on the Orin?

Rule set here: nothing gets changed on the Orin without asking first.

What was found, read-only:

- ROS 2 **Humble** (not Foxy), Fast DDS, `ROS_DOMAIN_ID=42`.
- JetPack 6 line: L4T 36.5.2, Ubuntu 22.04.5, Orin Nano Super developer
  kit. JetPack 6 is Ubuntu 22.04, which is the Humble release; Foxy would
  need JetPack 5.
- `livox-driver.service` running, publishing `/livox/lidar` (PointCloud2)
  and `/livox/imu`. `lidar-web.service` is a web panel on port 8000.
- The sensor was sending about 3 MB/s to the Orin.

### The problem I caused

A new subscriber on the Orin got no messages, although the topics were
listed and the driver was busy.

- Cause: the driver runs as user `lite3`. When the last `lite3` login
  session ends, Ubuntu deletes that user's shared-memory files, and ROS
  uses those files to pass messages between programs on the same machine.
- My first ssh login ended at 15:43:11. After that the driver was holding
  deleted files (`/dev/shm/fastrtps_port17915_el (deleted)`).
- Any ssh logout as `lite3` would have done the same.
- Only programs on the Orin itself were affected. Over the network the
  data kept flowing.

### The fix

Two commands on the Orin, run by the user (Claude Code's permission check
blocked me from running the first, because it changes an account setting):

```
sudo loginctl enable-linger lite3
sudo systemctl restart livox-driver.service
```

Verified afterwards: `Linger=yes`, 200 Hz and 10 Hz for a local subscriber,
and still arriving in a second login after the first had logged out.

## 4. Can Foxy and Humble talk?

Tested from the perception computer (Foxy), listening on the Orin's
domain 42:

| DDS on the perception computer | Result |
|---|---|
| Fast DDS | Works: `/livox/imu` 200 Hz, `/livox/lidar` 10.0 Hz |
| CycloneDDS (what the robot's nodes use) | Every `ros2` command segfaults on meeting the Orin's node |

Control: the same CycloneDDS command on an empty domain (43) did not crash.

What follows from it:

- Nothing is at risk as things are: the robot's nodes are on domain 0, the
  Orin on 42, and they do not see each other.
- The Orin must not be moved to domain 0. The robot's CycloneDDS nodes
  could crash the same way, including the bridge to the motion computer.

## 5. FAST-LIO2 on the Orin

- It is installed (`~/livox_ws/src/FAST_LIO`, patched) but is not a
  service. It starts when Live is pressed in the web panel, or after a
  recording.
- The Orin's own notes: `~/bin/README.md` (mapping) and
  `~/robot_pc/README.md` (a navigation plan that expects the perception
  computer on domain 42 with Fast DDS). That plan was never copied to the
  perception computer, and it does not match this repo's setup (CycloneDDS,
  domain 0).
- It publishes the lidar's pose at 10 Hz on `/Odometry`. It has no loop
  closure and cannot find itself in a saved map; every run starts at its
  own origin.

## 6. Getting the pose to Nav2: the choice

Two uses were on the table:

1. Better odometry: replace the leg odometry Nav2 uses. Nav2 stays mapless.
2. Navigating in a saved map: needs localization, which FAST-LIO2 does not
   do.

Chosen: 1.

How to carry it across: a relay on the perception computer. Two programs
joined by a pipe, the idea `robot/rviz.py` already uses. One listens on
Fast DDS, domain 42; the other publishes on the robot's own graph. Nothing
changes on the Orin or in the robot's services. The alternative, moving
the robot's whole ROS stack to Fast DDS, would touch the vendor services.

## 7. The relay, first version: `/lio_odom`

`robot/lio_relay.py` turns FAST-LIO2's lidar pose into the body's pose
(`odom -> base_link`), stamps it like `/leg_odom2`, and fills in speed
from consecutive poses.

Tests, in order:

1. The maths self-check (`python3 -m robot.lio_relay check`).
2. On the robot from a temporary folder, with a made-up pose published on
   domain 42 (lidar 1 m along its own x): `/lio_odom` showed x 0.937,
   z -0.349 at 10.0 Hz. That is the right answer for the 20.4° mount
   pitch.
3. Committed (`ea54663`) and pulled on the robot.
4. With a real Live run, robot still on a table: 10 Hz, position steady
   within a few millimetres, speed noise about ±0.01 m/s.

Should FAST-LIO2 always run? Not yet. It must start with the robot standing
still, it has diverged once on the current build for an unknown reason, and
today only the mapping panel starts it.

## 8. The walked loops

A logger recorded `/lio_odom` and `/leg_odom2` side by side while the user
drove a closed loop with the handheld. On a closed loop, the distance
between end and start is the drift.

**First loop: not usable.** The robot was already walking when logging
started, and the loop was not finished when the three minutes ran out.
What it did show: no dropouts or jumps, and the two agreed closely on
heading and distance.

**Second loop: clean.** Standing still on a floor mark at the start,
parked on the same mark at the end.

| | FAST-LIO2 (`/lio_odom`) | Leg odometry (`/leg_odom2`) |
|---|---|---|
| Distance walked | 10.40 m | 10.19 m |
| Gap between end and start | 0.05 m | 0.21 m |
| Heading compared with start | -5.5° | -5.2° |

FAST-LIO2 was clearly better on this loop. Both gave the same heading, so
the 5° is how the robot was parked. A 5 cm gap also leaves little room for
a wrong lidar mount offset. It is one 10 m loop: it says nothing about a
long walk, or about standing up from a sit.

## 9. The relay, second version: `/odom_fused`

Goal: an odometry Nav2 can drive on, which falls back to the legs if
FAST-LIO2 jumps or goes silent, without the pose ever jumping.

How it works:

- The fused pose is the leg pose moved by a correction. FAST-LIO2 updates
  the correction ten times a second; the legs supply the 50 Hz in between.
- A FAST-LIO2 step is believed only if it agrees with the legs' step over
  the same stretch (within 0.15 m and 0.2 rad). Otherwise the correction
  is kept and FAST-LIO2 is re-anchored to where the legs put the robot.
- So with the Orin silent, `/odom_fused` is `/leg_odom2` exactly.
- Not caught: FAST-LIO2 drifting slowly.

Measured for it: FAST-LIO2's pose arrives roughly 0.05 s late (from the
second loop's log), so each pose is matched with the leg pose from that
long ago.

Tests:

1. Self-check: agreement, legs under-counting, FAST-LIO2 shooting away, a
   silence followed by a new run.
2. On the robot, standing still with the Live run on: `/odom_fused` at
   50.0 Hz, within 3 mm of `/leg_odom2`, and the relay reported "corrected
   by FAST-LIO2".
3. Cost: about 25% of a core for the relay plus 8% for its listening
   child, averaged over the first 28 s.

## 10. Switching everything to `/odom_fused`

Nav2, `Lite3`, the sonar node and the rviz relay pass goals and obstacle
positions to each other in one odom frame, so they had to switch together.
Two ways were on the table: the relay always running as a service, or only
while Nav2 runs. Chosen: always, as a service. With FAST-LIO2 off,
`/odom_fused` is leg odometry exactly, so nothing changes until a Live run
is on.

What changed in the repo:

- `robot/lite3.py`: reads `/odom_fused`. New stop: if the newest pose is
  older than 0.5 s, a motion under way stops ("odometry stopped
  arriving") and a new one refuses to start. Needed because the relay is
  now a process that can die while the bridge still takes velocities.
- `robot/sonar_range.py`, `robot/rviz.py`: read `/odom_fused`. Old rviz
  recordings still play.
- `env/nav2-mapless-lite3.patch`: `odom_to_tf.py` makes
  `odom -> base_link` from `/odom_fused`.
- `env/lio_relay.service`: the systemd unit.
- `demos/check_motion.py`: two new checks on the fake robot (the relay
  dying mid-walk; setting off without odometry). Passes.

**Starting FAST-LIO2 from code.** The Orin's panel turned out to have an
HTTP API (`/api/live/start`, `/stop`, `/state`), reachable from the
perception computer. So nothing on the Orin changed:
`python3 -m robot.lio_relay start|stop|state`, or `lio_relay.live(...)`.
Only `state` was tried, because a Live run was on. Nothing calls it
automatically yet.

## 11. Deploying it on the robot

Order matters: once the new `Lite3` is pulled, it will not move without
the service.

1. Commit and push; pull in `~/robot`.
2. Install and start `lio_relay.service`.
3. Apply the `odom_to_tf.py` change in the vendor tree (source and
   installed copy).
4. Restart the web panel.
5. Walk a loop on `/odom_fused`, then try Nav2 on it.

Done so far: none of these.

## 12. Still open

- `/odom_fused` has not been walked yet, and Nav2 has not driven on it.
- FAST-LIO2 is not started automatically with Nav2.
- The relay costs about a third of a core on the perception computer.
- The lidar's x/y/z position on the body is still the Orin's guess
  (0.20, 0, 0.10 m); only the pitch is measured.
- The relay assumes the robot stands level when the Live run starts.
