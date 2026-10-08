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
| Nav2 and `Lite3` | Switched to `/odom_fused` and deployed on the robot (sections 10, 11). Walked by handheld; `walk()`, `turn()` and one Nav2 goal run on it |

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

Done 2026-10-08, with the robot standing and a Live run on:

1. Pushed (`70682f8`) and pulled.
2. Service installed, enabled and started. Its log said "/odom_fused is
   corrected by FAST-LIO2"; `/odom_fused` at 49.9 Hz.
3. `odom_to_tf.py` changed in both places, backups beside them as
   `odom_to_tf.py.pre-fused`. The repo's patch hunk reverses cleanly
   against the result, so the two match.
4. The panel had been restarted after the pull, before the service
   existed. It needed no second restart: it only waits for the topic.

5. Walked loop on `/odom_fused` (handheld, mark to mark, 96 s):

| | Fused | FAST-LIO2 | Leg odometry |
|---|---|---|---|
| Distance walked | 9.50 m | 9.44 m | 9.05 m |
| Gap between end and start | 0.033 m | 0.032 m | 0.073 m |
| Heading compared with start | -3.3° | -3.2° | -1.9° |

   The fused pose stayed within 0.029 m of FAST-LIO2 all the way and
   ended 0.001 m from it. The relay never left "corrected by FAST-LIO2",
   and no step between samples was larger than walking speed.

6. The robot driving itself on it, from the command line, user watching:

| Command | Result | Measured by the fused odometry |
|---|---|---|
| `walk 0.5` | target reached | moved 0.52 m |
| `turn 90` | target reached | turned 90.0° |
| `turn -90` | target reached | turned -89.2° |

   The relay stayed on "corrected by FAST-LIO2" throughout.

7. Nav2 on it (the user started Nav2 from the panel; I sent the goal):

- `odom -> base_link` came from the fused odometry: `tf2_echo` showed
  the same position as `/odom_fused`.
- `goto 1.5`: Nav2 reported the goal reached. The fused pose moved
  1.32 m.
- The relay stayed on "corrected by FAST-LIO2" and did not restart.
- In the Nav2 log: 38 dropped camera frames inside one second, and a few
  "Range sensor layer can't transform from odom to sonar_front/rear".
  The dropped frames came while I was starting several check commands on
  the computer; it was quiet before and after. The load average was
  about 9 on 6 cores.

So it drives. The open worry is the computer's load, not the odometry:
the odom transform now passes through two Python programs (the relay,
then `odom_to_tf.py`), and the relay itself takes about a third of a
core.

## 12. Nav2's margins

The user watched the 1.5 m goal: it went forward but curved, ended angled
to the right, and looked nervous on an easy path.

- The odometry itself recorded the turn (heading -179° to 142°, 39° to
  the right), so the robot turned on purpose; it was not the fused
  odometry misreading the heading.
- The costmap showed why: a solid obstacle from 0.5 m to the left of the
  straight line for the whole first metre (a table or its chairs), free
  floor on the right. The space straight ahead was clear, as the user said.
- Nav2 keeps a 0.45 m zone of raised cost around obstacles. With
  `cost_scaling_factor` at 3.0 that cost hardly fades inside the zone
  (54 or more out of 99, then zero), so the middle of an aisle looks
  almost as costly as the edge. Nav2 also accepts any final heading, so
  it stayed angled.

Changed, with the user's go-ahead: `cost_scaling_factor` 3.0 to 8.0 in
both costmaps (vendor config on the robot, source and installed copy,
backups `*.pre-csf8`; same change in `env/nav2-mapless-lite3.patch`).
The 0.45 m radius stays: it was raised from 0.30 because the planner
failed next to box corners. Nav2 restarted; both costmaps report 8.0.

### Driving with the new value

The user picked the robot up and put it in place (it detects the lift and
tucks its legs; it is made for that).

1. First goal: **aborted, the robot did not move.** The planner could not
   make a path: the robot's own cell and the cells just ahead of it were
   marked as obstacles, left from the carry. They had not faded after two
   minutes. The relay stayed on "corrected by FAST-LIO2" through the
   carry.
2. Nav2 restarted for a fresh costmap: own cell free, cost along the line
   0 to 29.
3. Second goal, 1.5 m ahead: reached. 1.28 m forward, 0.14 m to the left,
   **heading 40° to the right again.**

So the cost scaling change did not stop the turning: same goal, same 40°
as before it. The cause is still unknown. Next step is to record what the
controller commands (`/cmd_vel`) next to the pose during one drive, to
see whether Nav2 asks for the turn or the robot yaws on its own. Not done:
the battery was at 20%.

Also seen in this drive: 87 camera frames dropped for lack of a transform,
with the load average between 8.5 and 11.

## 13. After the battery change: FAST-LIO2 starting by itself

The battery went to 20% and was changed. After the reboot, as expected:
the relay service was up by itself, FAST-LIO2 was not running, and
`/odom_fused` was plain leg odometry.

The user offered to let me change code on the Orin so FAST-LIO2 would not
need starting by hand. It turned out not to need any change there:

- FAST-LIO2 must start with the robot standing still. The Orin does not
  know the robot's posture; the perception computer does.
- So the relay does it: every 5 s, if the robot is standing, has been
  still for a second and no FAST-LIO2 pose is arriving, it calls the
  Orin panel's start (the API from section 10), then waits 30 s before
  asking again.
- It never stops a run and does not restart a diverged one.

Self-check added for the rule (standing and still: yes; walking, lying,
already running: no).

Run on the robot (commit `40d3897`, robot standing still, FAST-LIO2 off):
the service was restarted at 23:27:37, asked the Orin to start at
23:27:50 (answer `ok`), and reported "/odom_fused is corrected by
FAST-LIO2" at 23:27:52. `/odom_fused` at 50.0 Hz.

## 14. Why Nav2 turns near the goal

One more 1.5 m goal, with a recorder on `/cmd_vel` (what Nav2's controller
commands), the fused pose and the leg odometry, 20 times a second. This
one was in an open spot: zero cost along the whole line.

Result: 1.24 m forward, 0.03 m to the right, heading -5°. Nearly straight
this time. The recording shows what happens:

| Phase | Controller commands | Heading |
|---|---|---|
| First 2.5 s of driving | 0.30 to 0.36 m/s forward, turn rate about 0 | within 1.3° |
| From 0.65 m before the goal | slows to 0.09 to 0.19 m/s, turn rate jumps between -0.31 and +0.31 rad/s | swings: -6°, -10°, -1°, +3°, -3° |

- The turn command changed sign 6 times in the last 3.5 seconds.
- Adding up the commanded turn gives -5.4°; the measured change was -4.9°
  (fused) and -5.1° (legs). So the robot does what it is told. It is
  Nav2's controller asking for the turns, not the odometry and not the
  robot yawing by itself.
- In the two earlier runs the same wiggle happened to hold one direction:
  0.31 rad/s for a bit over two seconds is the 40°.

So the "nervous" look and the angled finish are one thing: the controller
(DWB) has no steady preference for a heading once it is close to the goal.
In the config its scoring list is
`PreferForward, GoalDist, PathDist, RotateToGoal, BaseObstacle`. The two
entries that reward pointing along the path (`PathAlign`, `GoalAlign`)
have their weights set in the file but are not in the list, and the entry
made to stop left-right flipping (`Oscillation`) is absent. It also
simulates 3 s ahead (`sim_time`), which at 0.65 m from the goal means
crawling. This list is the vendor's; our patch never touched it.

### First try: adding PathAlign and GoalAlign. No effect.

With the user's go-ahead: the two alignment entries added to the list
(vendor config on the robot, backups `*.pre-align`; same in the repo
patch), Nav2 restarted, and the controller confirmed to have loaded the
new list. The robot backed up 1.23 m by itself (`walk(-1.2)`, rear sonar
3.93 m) and got the same 1.5 m goal with the recorder on.

| | Before | With PathAlign + GoalAlign |
|---|---|---|
| Turn-command sign flips | 6 | 7 |
| Samples with a hard turn command | 66 of 113 | 59 of 110 |
| Heading swing during the drive | -12° to +5° | -15° to +1° |
| Final heading | -5° | -12° |

Same behaviour: straight until about 0.65 m from the goal, then the
forward speed drops and the turn rate jumps between -0.31 and +0.31.

### What the numbers point to instead: PreferForward

The hard turning starts at the exact moment the commanded forward speed
drops below 0.30 m/s (0.30 to 0.25), and the turn rate is always the
first sample above 0.30 rad/s (0.3077). The config has
`PreferForward.strafe_x: 0.3` and `PreferForward.strafe_theta: 0.3`. As
I understand that scoring rule, any candidate that moves forward slower
than `strafe_x` while turning slower than `strafe_theta` gets the full
penalty, as if it were strafing. So as soon as the controller has to
slow down for the goal, going straight is penalised and turning at just
over 0.3 rad/s is the cheapest thing left, in either direction. That
reading is from memory of how the rule works, not from its source on the
robot; the two thresholds matching the recording is the evidence.

### Second try: `PreferForward.strafe_x` 0.3 to 0.05. Fixed.

With the user's go-ahead: PathAlign and GoalAlign taken out again (they
changed nothing), `PreferForward.strafe_x` lowered to 0.05 (vendor config
on the robot, both copies; same in the repo patch, which reverses cleanly
against the robot's files). Nav2 restarted and the controller reported
0.05.

The user asked for the robot to be turned before backing up, so it was
turned back to its starting heading first (+12°), then backed. The rear
sonar read 1.29 m, so by the script's rule (keep 0.6 m) it backed only
0.72 m, not 1.2 m. Same 1.5 m goal, recorder on.

| | Original | + PathAlign, GoalAlign | `strafe_x` 0.05 |
|---|---|---|---|
| Turn-command sign flips | 6 | 7 | 0 |
| Samples with a hard turn command | 66 of 113 | 59 of 110 | 0 of 99 |
| Largest turn command (rad/s) | 0.31 | 0.35 | 0.06 |
| Heading swing during the drive | -12° to +5° | -15° to +1° | 0° to +3° |
| Final heading | -5° | -12° | +2° |
| Drive time | 5.9 s | 5.7 s | 5.2 s |

It now slows down in a straight line (0.30, 0.19, 0.14, 0.09 m/s) and
stops 0.25 m from the goal, which is the tolerance. One run in an open
spot; the aisle between the tables, where it ended 40° off, has not been
repeated.

A slip of mine on the way: the first attempt at this run failed before
the robot moved, with "2 nav2 process groups are running". My ssh command
line contained the name of one of Nav2's programs (for a parameter
check), and `nav_start()` counts any process whose command line contains
such a name as a Nav2 stack. One real stack was running; I reran without
the word.

### The aisle between the tables

The user carried the robot to the start of the aisle. Nav2 restarted for
a clean costmap (the carry left marks last time). The costmap before the
goal: obstacles from 0.5 m to the left and to the right of the line from
0.75 m on, a free lane about half a metre wide down the middle, cost 0
to 13 on the line itself.

Same 1.5 m goal: reached, 1.24 m forward, 0.07 m to the right, heading
-1°. Before today's two config changes the same aisle gave 39° and 40°.
In the recording: no turn-command sign flips, largest turn command
0.10 rad/s, heading within 2° of the start all the way, at most 0.10 m
off the line sideways.

Also seen in the relay's log around the carries and restarts: it dropped
to "leg odometry alone" five times and was back on FAST-LIO2 within a
second each time.

### Goal tolerance 0.25 m back to 0.15 m

Every run stopped 0.25 to 0.27 m short of the goal, because Nav2 counted
the goal as reached within 0.25 m (our patch had raised it from the
vendor's 0.15). The user asked whether a smaller margin would hurt longer
routes. It does not affect the route, only the last stretch: a few more
seconds at crawling speed, and a risk of a legged robot stepping around a
goal it cannot settle into. With the wiggle gone, 0.15 was worth trying.

Changed with the user's go-ahead in both places it is set (goal checker
and controller), on the robot (backups `*.pre-tol15`) and in the repo
patch. Nav2 restarted; same 1.5 m goal from where the robot stood.

| | Tolerance 0.25 | Tolerance 0.15 |
|---|---|---|
| Moved forward | 1.24 m | 1.36 m |
| Stopped this far from the goal | 0.27 m | 0.15 m |
| Drive time | 5.7 s | 6.7 s |
| Final heading | -1° | +1° |
| Backward commands | none | none |

No stepping around the goal: it crept the last 15 cm at 0.09 m/s and
stopped. One run.

Three places in the code were tuned around the old 0.25 (`tour.py`
`MIN_GOAL`, `nav.goal_ahead`, `person.NAV_NEAR`). Their values still
hold as they are (they are lower limits, and stay on the safe side); only
their comments were updated. One behaviour does shift: when following a
person with Nav2, the robot now ends about 0.1 m closer to them. Not
re-tested.

## 15. Lidar into Nav2's costmap, step 1 (started)

The plan agreed for "navigate a whole building", in three steps: (1) the
lidar as an obstacle source in today's mapless Nav2, (2) a building map
and localization in it, (3) named places as goals. Step 1 first; the
user allowed a new service on the Orin for it.

Done:

- **On the Orin:** `pointcloud_to_laserscan` was already installed and
  `~/bin/lidar_nav.launch.py` already existed. Run by hand it gave a
  720-beam scan at 10 Hz in `base_link`. Installed as
  `lidar-scan.service`, enabled at boot (copy in `env/lidar-scan.service`).
- **The relay** now also carries `/scan` across and stamps it with the
  perception computer's clock. Tested on a separate ROS domain so the
  running odometry service was not disturbed: 9.6 Hz, 720 beams.

A false alarm worth recording: the scan showed returns 0.64 m straight
ahead while the sonar read 2 m clear. Looking at the raw cloud, they were
a level surface 0.21 m below the lidar. That was the floor: the robot had
been laid down in the meantime (state 8), and the scan's height band
assumes it is standing. It also says the lidar sits about 0.21 m above
the floor lying down, which fits the Orin's own figure of 0.46 m
standing.

The user confirmed it: the lidar is angled down, the robot was lying,
it was seeing the floor. Measured in that state: with the mount
transform the floor comes out as a level plane (0.7° and 0.6° off, fore
and sideways), 0.10 m below `base_link`. So the 20.4° pitch is right to
within a degree. Standing lifts the body about 0.24 m, which puts the
floor about 0.19 m below the scan's lower edge.

Because of this the relay now only passes scans on while the robot is
standing, so a lying robot does not fill the costmap with floor.

### With the robot standing (commit `ece671e` pulled, relay restarted)

- The relay restarted and was back on FAST-LIO2 in 9 s.
- `/scan` on the robot's side at 10 Hz. Straight ahead it read 2.27 to
  2.31 m; the depth camera said 2.29 m and the front sonar 2.09 m. So no
  floor in it while standing. To the sides and diagonally behind it saw
  things 0.71 to 0.87 m away that neither camera nor costmap knew about.
- Behind, only a handful of returns: the lidar is tilted nose-down, so
  backwards it looks upward and only catches tall things.

### In the costmaps

Added a layer on `/scan` to both costmaps (marking and clearing, used out
to 3 m), on the robot (backups `*.pre-scan`) and in the repo patch. The
patch was regenerated on the robot this time: our patch undone on a copy
to get the vendor original back, then a fresh diff against the edited
file. It reverses cleanly.

First start: Nav2 came up, both costmaps subscribed to the scan, no
errors, and the layer marked nothing. Cause: this layer's per-source
height limit (`max_obstacle_height`) defaults to 0.0, so every point was
discarded silently. I had it in my first draft of the block and dropped
it. Set to 2.0.

Second start: the costmap shows the room all around. The tables to the
left from 0.75 m, something to the right behind, the wall 2 m ahead, an
obstacle 1 m behind. Check: of 403 scan returns within 2.5 m, 402 sat on
an obstacle cell.

The user confirmed the picture: a table on the left, the wall ahead, a
Husky robot under a table on the right behind, a chair behind on the left.

### First goal with the lidar layer, and a bug in the fused odometry

A 1.0 m goal with the recorder on: straight, no turn-command flips,
heading within 1°. Nav2 reported the goal reached.

But the recording shows the fused odometry was wrong on this drive:

| Source | Says the robot moved |
|---|---|
| Fused odometry | 0.86 m |
| Leg odometry | 0.62 m |
| Lidar: how much closer the wall got | 0.63 m |

The fused pose jumped forward 0.24 m within 0.4 s in the middle of the
drive, while the legs moved 0.08 m. So the robot stopped about a quarter
of a metre short of where it believed it was. The relay's log also showed
it flipping between "leg odometry alone" and "corrected by FAST-LIO2"
every few seconds since its last restart.

Measured, robot standing still, 25 s:

- FAST-LIO2's poses reach the perception computer at 7.9 a second, not 10.
- One in twenty arrives more than 0.35 s late; the worst was 0.78 s.
- By their own stamps the poses are up to 0.5 s apart, so some never
  arrive. FAST-LIO2's own log shows it getting 8.3 to 10 scans a second.
- The scan itself arrives steadily at 10 a second.

The mistake is in my fusion, and it was there from the start; the loops
earlier in the day just did not hit it hard. It paired each pose with
where the legs were when the pose ARRIVED, assuming a fixed 0.05 s delay.
A pose arriving half a second late was therefore compared with legs that
had walked on, and the 0.5 s gap limit treated every missing pose or two
as a new run.

Changed in `robot/lio_relay.py` (not deployed yet):

- Each pose is matched by its own time stamp to where the legs were then.
  The offset between the two computers' clocks is taken from the
  quickest pose seen so far.
- Leg poses are remembered for 2 s instead of 1 s.
- A gap has to be 1.0 s before it counts as a new run.

Self-check added.

### Side-by-side test on the robot

The new code was run beside the service, publishing under test names, and
the robot backed 0.6 m and walked 0.6 m forward (0.3 m/s) while old
fused, new fused, legs and the lidar's distance to the wall were recorded.

| | Old fused | New fused | Legs | Wall (lidar) |
|---|---|---|---|---|
| Backward leg | 0.65 m | 0.65 m | 0.67 m | 0.67 m |
| Forward leg | 0.67 m | 0.67 m | 0.71 m | 0.69 m |
| Net, start to end | +0.02 m | +0.02 m | +0.05 m | +0.02 m |
| Largest change in 0.2 s | 0.11 m | 0.11 m | 0.11 m | |

What it shows and does not show:

- The new code is right on this walk: it ends where the wall says, and
  never moves faster than the legs.
- The old code was also right on this walk. Its jump did not come back,
  so this test does not prove the fix; it only shows no harm.
- Where the two did differ: standing for 90 s, the old relay dropped to
  leg odometry 15 times, the new one once.

Two mistakes of mine in running it:

- The test script failed twice before the robot moved, because Python
  found the test copy of the `robot` package before the real one.
- My clean-up command after the first failure matched the real service's
  process as well and killed it. systemd restarted it within seconds
  (`NRestarts=1`), nothing was moving, but the odometry was away for
  about ten seconds and the fused pose lost the correction it had built
  up, so the odom frame shifted under Nav2's costmap.

Note for later: the Orin panel's Navigate tab starts the same launch
file itself and kills any copy it finds first. Using that tab would
fight with the new service.

### Deployed (commit `9c0814a`)

Relay restarted with the new matching, Nav2 restarted, the robot backed
0.37 m (rear sonar 0.96 m, so the script allowed no more) and got a
1.0 m goal with the recorder on.

| Source | Says the robot moved |
|---|---|
| Fused odometry | 0.83 m |
| Leg odometry | 0.89 m |
| Lidar: how much closer the wall got | 0.81 m |

No jump: the fused pose never changed more than 0.11 m in 0.2 s (legs
0.12 m). No turn-command flips, heading +2°. The relay dropped to leg
odometry once in the three minutes after the restart (while standing,
before the drive), against 15 times in 90 s before.

One drive. The same goal before the fix read fused 0.86, legs 0.62,
wall 0.63.

## 16. Spec for the rest

Steps 2 and 3 were specified with the spec command, question by question.
What the user decided:

- Mainly "send it to a place"; talking while it walks would be nice.
- From code, from the panel and by voice, all three.
- Anyone with access to the panel may use it.
- Waypoints as pins on the map, and also as physical tags in the
  building ("A for convenience, B for fun"); tags as a later phase.
- One floor at a time. Stairs and lifts are for later.
- Localization on the perception computer, with the Orin as the fallback
  if the load does not fit.
- The building (UiA): some glass, all doors opaque and sometimes closed,
  long featureless corridors.
- It is a project of a month or more.

Saved as `.planning/specs/building-navigation.md`, five phases: map and
localization, places from code, the panel, voice, tags.

## 17. Still open

- The wiggle fix (section 14) has two runs behind it: one in the open,
  one in the aisle.
- Lidar step 1: whether the floor leaks into the scan while trotting,
  and what the lidar layer costs the perception computer.
- Steps 2 and 3 of whole-building navigation (map + localization,
  named goals): not started; a short spec comes first.
- Why FAST-LIO2's poses reach the perception computer at about 8 a
  second and sometimes most of a second late.
- Marks left in the costmap by carrying the robot block planning until
  Nav2 is restarted.
- Nav2 has driven one 1.5 m goal on `/odom_fused`. Not more than that.
- The perception computer's load (about 9 on 6 cores with Nav2, the panel
  and the relay): camera frames get dropped for lack of a transform when
  it spikes. Not yet looked into.
- FAST-LIO2 is not started automatically with Nav2.
- The relay costs about a third of a core on the perception computer.
- The lidar's x/y/z position on the body is still the Orin's guess
  (0.20, 0, 0.10 m); only the pitch is measured.
- The relay assumes the robot stands level when the Live run starts.
