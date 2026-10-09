# Plan: Building navigation

**Spec**: .planning/specs/building-navigation.md
**Epic**: none
**Created**: 2026-10-09
**Status**: draft

Stack: Python on ROS 2 Foxy (perception computer), one HTML page for the
panel. Not Kotlin, not Next.js; the plan follows this repo's own patterns
(the `Nav` mixin, scripts and patches in `env/`, self-checks instead of a
test framework, README for the long explanations).

Phases 1 and 2 are planned to the task. Phases 3 to 5 are planned more
loosely on purpose: the load gate at the end of phase 1 decides where
localization runs, and that can move things.

---

## What was checked on the robot before planning (2026-10-09)

| Question | Answer |
|---|---|
| Are map server and AMCL installed? | Yes: `nav2_map_server`, `nav2_amcl` in `/opt/ros/foxy`, nothing to build. |
| Does the vendor map config (`dr_nav2`) give us AMCL? | No. `lite_nav2.yaml` has no `amcl` section, and the robot's `nav2_bringup/localization_launch.py` starts only `map_server`. The vendor localizes with `hdl_localization` (3D lidar) instead. So AMCL is launched and configured by us. |
| Anything reusable from `dr_nav2`? | The `static_layer` block and the `map` frame names. Its lidar layer is for another lidar. Its global costmap is a 30 m rolling window, which cannot plan the spec's 30 m routes. |
| AMCL settings to start from? | The Orin's `~/robot_pc/nav2_params.yaml` already has a block written for this robot (omnidirectional, wide motion noise, 0.4 to 30 m). |
| Are there maps? | Yes, on the Orin: `~/maps/<run>/slam_output/latest/walls/floorN.{pgm,yaml}`, Nav2 format, 0.05 m per cell, about 35 x 35 m. |
| Can the map be copied with the Orin's `robot_send.sh`? | Not as it is: `~/robot_pc.env` has no user filled in and it sends a whole bundle to `~/lite3_nav`. A two-file copy is simpler. |
| Clocks | Both computers are NTP-synced and agree. But everything on the robot's graph (`/odom_fused`, the TF, `/scan`) is stamped with the legs' steady clock, not wall time. Anything we publish with a stamp must use that clock too. |
| Room on the perception computer | 6 cores, load 4.1 lying down with no Nav2; 5.2 GB memory free. Load is the risk, memory is not. |

---

## Architecture

```
panel (hmi.py, index.html)      talk.py        demo scripts
            \                      |               /
             +---------- Lite3 (robot/nav.py) ----+
                  go_to, set_pose, places, localized
                       |                    |
                 places.json         /localized  <- robot/locate.py
                                           |          (in the Nav2 process group)
        Nav2 (vendor mapless launch + map server + AMCL), global frame "map"
              |                 |                  |
          /odom_fused         /scan             map.pgm
```

Decisions:

1. **One Nav2 config, not two.** Map mode's params are generated at start
   from the mapless yaml already on the robot (the one our patch tunes), by
   a small script that changes the global frame, adds the static layer and
   adds the AMCL and map server blocks. All tuning stays in one place.
2. **Map mode is a second start script** beside `start_nav2_mapless.sh`.
   `nav_start()` with no floor does exactly what it does today.
3. **Global costmap in map mode is not rolling**: it takes its size from
   the map. Update rate is lowered (5 Hz to 1 Hz) to pay for the size. The
   local costmap is untouched.
4. **One small node says "localized"** (`robot/locate.py`), started by the
   start script so it dies with the stack. It publishes the map pose, the
   spread of AMCL's estimate and how well the scan fits the map, at 2 Hz.
   Every `Lite3` (panel, CLI, tour) reads that one topic instead of each
   working it out. Spread alone is not enough: on the wrong floor AMCL
   still settles on something, so the scan fit is what catches edge cases
   2 and 8.
5. **Map pose without subscribing to `/tf`** in Python: `locate.py` pairs
   `/amcl_pose` with the odometry at the same stamp (the `leg_at` idea from
   `lio_relay.py`) and applies that offset to odometry read at a low fixed rate. Check 4
   settled it: a tf2 listener in Python costs 18.5% of a core here.
6. **Places are a plain JSON file** per floor, read and written by one
   module with no ROS in it.
7. **Panel map picture**: the raw grey bytes over HTTP, painted into a
   canvas. No image library on the robot.

Not doing: `hdl_localization` (needs a 3D map and more CPU), a second yaml,
a database, a places service.

### Components

| Component | Type | Purpose |
|---|---|---|
| `nav2_map_params.py` | script | mapless yaml in, map-mode yaml out |
| `nav2_map.launch.py` | launch | vendor mapless launch + map server + AMCL + their lifecycle manager |
| `start_nav2_map.sh` | script | sonar node, params, `locate`, launch; one process group |
| `get_map.sh` | script (laptop) | copy one floor map from the Orin to `~/lite3_maps/<floor>/` |
| `locate.py` | ROS node | publishes `/localized` |
| `places.py` | module | the places file |
| `Nav` (existing) | mixin | floors, pose, places, `go_to` |
| Map view | panel | map, robot, places, route |

### New files

| File | Location | Purpose |
|---|---|---|
| `nav2_map_params.py` | `env/` | generate the map-mode params, with a self-check |
| `nav2_map.launch.py` | `env/` | the map-mode launch |
| `start_nav2_map.sh` | `env/` | start it, as `start_nav2_mapless.sh` does |
| `get_map.sh` | `env/` | fetch a floor map |
| `locate.py` | `robot/` | localized state, with a self-check |
| `places.py` | `robot/` | places file, with a self-check |
| `check_map.py` | `demos/` | draw one live scan on the map at a given pose (check 1.3) |

### Files to change

| File | What changes | Why |
|---|---|---|
| `robot/nav.py` | `nav_start(floor=None)`; the frame goals and `cost_at` use; `floors`, `load_floor`, `set_pose`, `localized`, `map_pose`, `go_to`, `go_to_point`, places calls | spec phases 1 and 2 |
| `robot/lite3.py` | subscribe to `/localized`; CLI words `floors`, `load`, `here`, `places`, `save`, `go` | same |
| `demos/check_motion.py` | `go_to` and its refusals against the pretend Nav2 | spec "without the robot" |
| `robot/hmi.py` | map bytes route, new websocket messages, places | phase 3 |
| `robot/hmi_static/index.html` | Map view | phase 3 |
| `robot/talk.py` | "go to a place" as a tool the talker can call | phase 4 |
| `README.md`, `LIDAR_ORIN_LOG.md` | section 7 gets map mode; measurements go in the log | repo habit |

---

## Tasks

Each task is one commit and at most three files. "Robot" means it needs
the robot standing; the rest runs on the laptop.

### Phase 1a: checks, nothing built yet

| # | Task | Files | Needs |
|---|---|---|---|
| 1 | **Done 2026-10-09** (log section 17: 42% idle standing, 39% on a goal, no dropped camera messages after the first 20 s). Baseline for the load gate: mapless Nav2 standing still and on one 3 m goal; record load average, per-process CPU and camera messages dropped per minute from `/tmp/nav2.log`. | `LIDAR_ORIN_LOG.md` | Robot |
| 2 | **Done 2026-10-09** (0.20, 0, 0.14 m; `lidar_z:=0.14` on the Orin). Measure the lidar's position on the body with a ruler (x forward, z up from body centre) and set it in the Orin's `lidar-scan.service`. | `env/lidar-scan.service`, log | You, and a yes to change the Orin |
| 3 | **Done 2026-10-09, and it changed the plan**: the walls map and the low `/scan` do not match (fit 0.40; furniture). A scan cut 0.9 to 1.7 m above `base_link` fits 0.87 to 0.99 (log section 17). So localization gets a second, high scan (`/scan_walls`) from the Orin, through the relay; AMCL and `locate.py` use it, the costmaps keep `/scan`. Built and measured live the same day: fit 1.00 against 0.37 for the low scan. The map is `lab`, 0.1 m cells. `get_map.sh` written 2026-10-09; the map is to be recorded anew, so the fetch and the overlay wait for it. `get_map.sh`; fetch the map of the floor the robot is on. Then `check_map.py`: draw one live scan on that map at a hand-given pose. If walls do not line up (wrong height slice, wrong scale), stop here and fix the map side first. | `env/get_map.sh`, `demos/check_map.py` | Robot |
| 4 | **Done 2026-10-09** (50 Hz, a Python tf2 listener costs 18.5% of a core: decision 5 stands, and `locate.py` reads odometry at a low fixed rate). Measure the `/tf` rate and what a Python tf2 listener costs on the Jetson, to settle decision 5. Tried 2026-10-09 lying down: `/tf` is empty without Nav2 (an idle listener costs 2.8% of a core), so this is measured in the same run as task 1. | log | Robot, with task 1 |

### Phase 1b: map mode (depends on 1a)

| # | Task | Files | Needs |
|---|---|---|---|
| 5 | **Done 2026-10-09** (on the robot's installed file it changes exactly the nine intended paths). `nav2_map_params.py`, self-check first: the output has the `map` frames, the static layer first in the global costmap, AMCL and map server blocks, and every other key identical to the input. | `env/nav2_map_params.py` | laptop |
| 6 | **Done 2026-10-09, by hand** (log section 17): map mode starts, AMCL localizes on `/scan_walls`, unmapped cells are walls to the planner (`unknown_cost_value`, set on the costmap node). Nav2 only finishes starting once AMCL has a pose, so `load_floor` and `set_pose` are two steps. Decide here, with the real map in hand, what unmapped cells mean to the planner (today `track_unknown_space` is off and NavFn has `allow_unknown: true`, so anything not a wall is walkable). `nav2_map.launch.py` and `start_nav2_map.sh`. Start by hand on the robot, send the initial pose with `ros2 topic pub`, drive round with the handheld, watch `map -> odom`. | `env/nav2_map.launch.py`, `env/start_nav2_map.sh` | Robot |
| 7 | **Laptop part done 2026-10-09**; the node itself has not run against AMCL. One scan fit costs 0.54 ms on the Jetson. `locate.py`, self-check first for the pure parts: pose pairing, spread from a covariance, scan fit against a small made-up map, the localized rule (pose given since start, fresh, spread under limit, fit over limit). | `robot/locate.py` | laptop, then robot |
| 8 | **Done 2026-10-09** (on the robot, standing: load 17 s, pose 2 s). `Nav`: `floors()`, `nav_start(floor)` / `load_floor`, `set_pose`, `localized`, `map_pose`; `Lite3` subscribes to `/localized`. `set_pose` stamps with the odometry's clock. | `robot/nav.py`, `robot/lite3.py` | laptop, then robot |
| 9 | During a 2 m route and back (log section 17): idle 29 to 38%, no dropped camera messages once under way, but a burst of about 50 in the seconds after the first goal starts (AMCL's first update, probably). Not measured with the panel open. First numbers 2026-10-09, standing, no goal, no panel: idle 41% against 53% mapless, 0 dropped camera messages, 0 loop misses, so it passes so far. Still to measure: during a route, and with the panel open. **Load gate.** Repeat task 1 with map mode running. Pass = no dropped camera messages once Nav2 has been up 20 s, which is what the baseline showed. Fail = move map server and AMCL to the Orin and carry `map -> odom` across in the relay (replans tasks 6 and 7, nothing else). | log, `README.md` | Robot |
| 10 | Known from the paper room in `locate.py`'s check: with `FIT_MIN` 0.7 a pose 0.2 m or 3 degrees off still passes, a different room sharing a corner scores 0.62, and a slide along one pair of walls scores 0.58. Tune on the floor: spread and fit limits from real numbers (localized, carried 5 m, wrong floor loaded); AMCL noise if the corridor test slides. | `robot/locate.py`, `env/nav2_map_params.py`, log | Robot |

### Phase 2: places from code (depends on 8)

| # | Task | Files | Needs |
|---|---|---|---|
| 11 | **Done 2026-10-09.** `places.py`, self-check first: save, rename, delete, duplicate name in another case, empty name, a damaged file (kept aside, not overwritten), UTC timestamps, write to a temp file then rename. | `robot/places.py` | laptop |
| 12 | **Done 2026-10-09** (checks). Goals in the map frame: `goal_send` and `cost_at` use the frame Nav2 was started in; `goto(forward)` keeps working in both modes. Checks added to `check_motion.py` first. | `robot/nav.py`, `demos/check_motion.py` | laptop |
| 13 | **Done 2026-10-09** (checks; no route walked on the robot yet). `go_to_point` and `go_to(name)`: refuse when not localized, when the point is off the map or in a lethal cell, when the name is unknown; cancel the goal and halt if localization is lost on the way; same return value as `goto()`. Checks first. | `robot/nav.py`, `demos/check_motion.py` | laptop |
| 14 | **Done 2026-10-09.** `places()`, `save_place`, `delete_place` on `Lite3`, and the CLI words. | `robot/nav.py`, `robot/lite3.py` | laptop |
| 15 | Begun 2026-10-09: two routes of 2 m out and back, stopping 0.11 to 0.18 m from the target; he now faces along the route himself before Nav2 gets the goal. On the floor: save three places, go between them, close a door, carry it mid-route. README section 7 gets map mode. | `README.md`, log | Robot |

### Phase 3: the panel (depends on 14)

| # | Task | Files |
|---|---|---|
| 16 | `hmi.py`: map bytes route; floor list, map scale, pose and localized state in the status message; the planned path sent in the map frame. | `robot/hmi.py` |
| 17 | Map view, read-only: map, robot, places, route; pan and pinch-zoom; floor picker; the "not localized" banner. All the states the spec lists. | `robot/hmi_static/index.html` |
| 18 | Commands: go to place, go to point with "Go here?", "I am here" tap-and-drag. Last goal wins and both panels show it. | `robot/hmi.py`, `robot/hmi_static/index.html` |
| 19 | Places editing: save this spot, drop a pin, rename, delete, export, import. | `robot/hmi.py`, `robot/hmi_static/index.html` |
| 20 | Browser check with a fake connection at 375 px and laptop width, then on the robot from a phone. | none expected |

### Phase 4: voice (depends on 14)

| # | Task | Files |
|---|---|---|
| 21 | A "go to place" tool for the talker whose only allowed values are the saved names; it says the place back, then calls `go_to`. | `robot/talk.py` |
| 22 | On the robot, with a misheard name and a name that does not exist. | log |

### Phase 5: tags (depends on 15; planned again when we get there)

| # | Task | Files |
|---|---|---|
| 23 | Check which tag library runs on the Jetson (its OpenCV may already have ArUco) and what it costs per frame. | log |
| 24 | Detect tags in the colour image; a place can carry a `tag_id`; `go_to` a tag place stops in front of it. | `robot/tags.py`, `robot/nav.py` |
| 25 | A tag with a known map position corrects the pose, within a distance, an angle and a maximum step. | `robot/tags.py`, `robot/locate.py` |

### Order

| Parallel group | Tasks | Why |
|---|---|---|
| A | 1, 2, 4 | independent measurements |
| B | 5, 7, 11 | laptop only, separate files |
| C | 19, 21 | panel editing and voice do not touch each other |

| Sequential | Depends on | Why |
|---|---|---|
| 3 | 2 | the overlay is only meaningful with the lidar where it really is |
| 6 | 3, 5 | needs a map that fits and the generated params |
| 8 | 6, 7 | wraps both |
| 9 | 8 | measures the whole thing |
| 10 | 9 | no point tuning before knowing which computer it runs on |
| 12 to 15 | 8 | need the map frame and the localized reading |
| 16 to 20 | 14 | the panel calls `Lite3` |

---

## Testing plan

**Places file (task 11, laptop)**: save, rename, delete; duplicate names
without regard to case; empty name; damaged file; timestamps end in `Z`.

**Navigation logic (tasks 5, 7, 12, 13, laptop)**

- Generated params: only the intended keys differ from the input.
- Localized rule: false before a pose is given, false when the estimate is
  stale, false on wide spread, false on poor fit, true otherwise.
- `go_to`: arrives, cancelled, timed out, as `goto()` is checked today.
- Refusals with a reason: not localized, unknown name, off the map, lethal
  cell (spec edge cases 1 and 7).
- Localization lost mid-route: goal cancelled and the robot halted (edge
  case 2).
- `goto(forward)` in mapless mode behaves as before: the existing
  `check_motion.py` must pass unchanged.

**On the robot (tasks 6, 9, 10, 15)**

- Drawn position follows it when driven by hand.
- Load: dropped camera messages per minute against the baseline.
- Restart Nav2 mid-floor: refuses until told where it is.
- Carried 5 m: reports lost or corrects itself, does not walk on.
- Wrong floor loaded: never localized (edge case 8).
- Door closed on the route: re-plans or stops with "blocked" (edge case 4).
- Relay service stopped mid-route: stops, as today (edge case 9).
- Longest corridor and back: drawn position within 0.5 m.
- Glass section: neither lost nor into the glass (edge case 3).
- The spec's 10 starts, 30 m, 0.3 m, 9 of 10.

**Panel (task 20)**: the page driven in a browser with a fake connection:
every state in the spec, the messages it sends for each command, 375 px
and laptop width. Then two panels sending goals (edge case 11).

**Voice (task 22)**: says the place back first; refuses a name not in the
list (edge case 12).

---

## Gate 2

- [x] Follows the repo's patterns (mixin, `env/` scripts, self-checks)
- [x] Layers: panel and voice call `Lite3`; `Lite3` calls Nav2 and reads `/localized`
- [x] New and changed files listed with locations
- [x] Tasks are small, dependencies and parallel groups marked
- [x] Data, logic, integration and panel tests planned
- [x] Spec edge cases mapped to tests (13 waits for the phase 5 replan)

## Open questions

1. A new map is to be recorded (user, 2026-10-09); task 3 waits for it.
2. The 9-of-10, 0.3 m and 0.5 m targets stay "to be confirmed" until task
   10 gives real numbers.
