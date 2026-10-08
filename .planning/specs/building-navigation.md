# Spec: Building navigation

**Created**: 2026-10-08
**Status**: draft
**Author**: Haavard Karlsen Solheim
**Epic**: none

Builds on what was done on 2026-10-08: fused odometry, the lidar as a flat
`/scan`, and that scan in Nav2's costmaps. See `LIDAR_ORIN_LOG.md` and
`README.md` sections 7 and 10.7.

---

## Problem

The Lite3 can only be told "go 1.5 m ahead". It has no memory of the
building: it cannot be sent to a place it cannot see from where it stands,
and after every restart its coordinates start from zero. At UiA, where it
is a university robot shown to whoever is around, nobody can say "go to the
lab door" and have it find its own way.

## Goal

On one floor of a UiA building, anyone with access to the web panel can
send the robot to a named place from anywhere on that floor, and it gets
there by itself.

Measured as (targets, to be confirmed on the first real floor):

- From 10 different starting spots on the floor, it reaches a saved place
  and stops within 0.3 m of it in at least 9 of 10 runs, on routes of 30 m
  or more.
- After a restart or after being carried, it knows where it is again
  within 30 s of being told roughly where it stands (one tap on the map).
- It never starts a route while it does not know where it is.
- The perception computer drops no more camera frames with this running
  than it does today with mapless Nav2.

## User stories

- As anyone with the panel open, I tap "Lab door" in a list of places and
  the robot walks there, so I do not have to drive it.
- As anyone with the panel open, I tap a spot on the floor map and the
  robot goes there.
- As the person setting it up, I drive the robot to a spot and press
  "save this place", or drop a pin on the map, and give it a name.
- As someone writing a demo script, I call `bot.go_to('lab door')` and get
  back whether it arrived, the way `goto()` works today.
- As someone standing next to it, I say "go to the lab door" and it goes,
  after telling me where it understood it should go.
- As a visitor, I hear it talk while it walks (nice to have).
- As the person setting it up, I stick a printed tag on a door, and that
  tag is a place the robot can be sent to (later phase).

## Requirements

Ordered in phases. Each phase is usable without the ones after it.

### Must have

**Phase 1: a map and knowing where it is**

- Load one floor map at a time, chosen by name. Maps come from the Orin's
  mapping run (`~/maps/<run>/slam_output/latest/walls/floorN.{pgm,yaml}`),
  copied to the perception computer.
- Localization on the perception computer: a map server and AMCL matching
  `/scan` against the map, publishing `map -> odom` on top of
  `/odom_fused`'s `odom -> base_link`.
- Nav2's global costmap uses the floor map plus today's obstacle layers.
  The local costmap stays as it is.
- A way to tell the robot roughly where it is (an initial pose), and a
  "do I know where I am" reading that code and the panel can use.
- It refuses to start a route until it is localized, and stops a route if
  it stops being localized.
- The mapless mode of today keeps working, unchanged, when no map is
  loaded.
- **Load gate:** before anything else is built on it, measure the
  perception computer with localization running. If it does not fit, the
  fallback is to run the map server and AMCL on the Orin and carry
  `map -> odom` across with the relay. Moving all of Nav2 to the Orin is
  not part of this spec.

**Phase 2: places, from code**

- Places are stored per floor: a name, a position and a heading on the
  map.
- `Lite3` gets calls next to `goto()`: go to a place by name, go to a map
  point, list places, save the current spot as a place, delete a place,
  load a floor, set the initial pose, and read whether it is localized.
- Going to a place returns the same kind of result `goto()` does, and can
  be cancelled the same way (`goto_cancel()`, `estop()`).
- An unreachable place, or one inside an obstacle, is refused or given up
  on cleanly, with a reason.

**Phase 3: the web panel**

- The floor map with the robot's position and heading on it, live.
- The list of places; tap one to go there. Tap the map to go to that spot.
- Save the current spot as a place; drop a pin on the map as a place;
  rename and delete places.
- Tap-and-drag on the map to tell the robot where it is.
- Choose the floor. Show clearly when it is not localized.
- The planned route drawn on the map, and a stop button that always works.
- Export and import of a floor's places as a file.

**Phase 4: voice**

- The Gemini talker can be asked to go to a saved place by name.
- It says back the place it understood and only then starts.
- It never goes to a place that is not in the saved list.

### Nice to have

- Talking while it walks, without stopping.
- No-go areas drawn on the map in the panel (for glass and for places it
  should stay out of).
- A route of several places in order (the base for a tour).
- Finding its position by itself after a restart, with no tap.

### Later phase, in scope

**Phase 5: physical tags**

- Detect printed tags (AprilTag or ArUco family, to be chosen) with the
  depth camera's colour image.
- A place can be "at this tag": the robot goes to stand in front of it.
- Optionally, a tag whose position on the map is known corrects the
  robot's position when it is seen. This is aimed at the long corridors.

### Out of scope

- Stairs and lifts, and changing floors by itself. One floor at a time;
  a person moves it between floors and picks the floor.
- Building the map. This spec uses a finished map from the Orin.
- Updating the map automatically when the building changes.
- Opening doors. A closed door is a wall.
- Outdoors.
- Moving all of Nav2 to the Orin.

## Data model

No database. Files on the perception computer, outside git (they are
specific to the building):

```
~/lite3_maps/
  <floor name>/
    map.pgm            the floor map, as the Orin made it
    map.yaml           resolution and origin, Nav2's format
    places.json        the places on this floor
```

`places.json`:

```json
{
  "floor": "uia-floor-2",
  "places": [
    {"name": "lab door", "x": 12.40, "y": -3.10, "yaw_deg": 90.0,
     "tag_id": null, "created_at": "2026-10-20T09:15:00Z"}
  ]
}
```

- Names are unique per floor, compared without regard to case.
- `x`, `y` in metres and `yaw_deg` in degrees, in the map's frame.
- `tag_id` is null except for a tag place (phase 5).
- Timestamps are UTC, ISO 8601 with `Z`.
- Deleting a place removes it from the file. The export button is the
  backup; there is no soft delete.

## API changes

No HTTP API of the backend kind. Two interfaces change.

**`Lite3` (Python), next to `goto()`** (names to be settled in the plan):

| Call | Does |
|---|---|
| `load_floor(name)` | start localization on that floor's map |
| `floors()` | the floors that have a map |
| `set_pose(x, y, yaw_deg)` | tell it where it is on the map |
| `localized` | whether it knows where it is |
| `map_pose` | `(x, y, yaw)` on the map, or `None` |
| `places()` | the places on the loaded floor |
| `save_place(name, x=None, y=None, yaw_deg=None)` | here, or at a map point |
| `delete_place(name)` | |
| `go_to(name)` | go to a place; returns the action status like `goto()` |
| `go_to_point(x, y, yaw_deg=None)` | go to a map point |

**Web panel connection (`robot/hmi.py`)**: new messages on the existing
websocket, in the style of the ones there now, for: the list of floors and
the loaded one, the map picture and its scale, the robot's map pose and
localized state, the places, the planned route, and the commands load
floor / set pose / go to place / go to point / save, rename and delete
place / export and import.

## Frontend changes

The panel is one page, `robot/hmi_static/index.html`, served by
`robot/hmi.py`. There is no build step and no TypeScript. The pendant look
the panel has now is kept.

**Files to change**

- `robot/hmi_static/index.html`: a new Map view.
- `robot/hmi.py`: the new messages, the map picture, places.

**Map view**

- The floor map drawn to scale, pan and pinch-zoom on a phone.
- On it: the robot (position and heading), saved places as labelled pins,
  the planned route, the current goal.
- A floor picker and a "not localized" banner that blocks the go actions.
- A places list beside or below the map; tap a place to go.

**Behaviour**

- Tap a place, or tap the map: a "Go here?" confirmation, then it goes.
- "Save this spot": asks for a name; rejects an empty name and a name
  already used on this floor, with a message saying which.
- "I am here": tap where the robot stands and drag towards where it faces.
- Stop is always visible and stops at once; a route under way is
  cancelled, as Cancel does today.
- If two people send goals, the last one wins and both panels show the
  goal now in force.
- States designed: no map loaded, loading, not localized, localized and
  idle, going, arrived, could not get there (with the reason), lost.
- Works at phone width (375 px) and on a laptop.

## Edge cases

1. **It does not know where it is** (after a restart, after being
   carried): refuses to navigate, the panel says so, and asks for a tap.
2. **It gets lost on the way.** In the long featureless corridors of the
   building the position can slide along the corridor. It must notice
   (the position estimate's spread passes a limit, or the scan stops
   fitting the map), stop, and say so. Tags (phase 5) are the planned help.
3. **Glass.** The lidar sees through glass, so glass walls are missing
   from the map and the scan will not match there. The depth camera is
   also poor on glass; the sonars do see it. Glass areas are marked by
   hand on the map (or as no-go areas) and listed per floor.
4. **A door on the route is closed.** All doors are opaque, so the robot
   sees a closed one as a wall. It re-plans another way if there is one,
   and otherwise stops and reports that the way is blocked. It does not
   push.
5. **A door that was closed when the map was made is open now**, or the
   other way round. The map shows a wall that is not there, or a gap that
   is. The first costs a detour; the second is case 4.
6. **The map no longer matches** (furniture moved): handled as obstacles
   on the day; if the scan fits the map too badly, case 2.
7. **A place is inside an obstacle or cannot be reached**: refused before
   it starts, or given up on with a reason.
8. **The wrong floor is loaded**: the scan will not fit; it never becomes
   localized and says so.
9. **FAST-LIO2 or the relay drops out mid-route**: odometry falls back to
   the legs as today; localization carries on from the scan.
10. **The perception computer cannot keep up**: the load gate of phase 1;
    the Orin fallback.
11. **Two people send it to different places**: last one wins, shown to
    both.
12. **Voice mishears a place name**: it says the place back first, and
    only goes to names in the list.
13. **A tag is moved, damaged, or seen from far off at an angle**: a tag
    reading is used only within a set distance and angle, and never moves
    the position by more than a set amount in one go.
14. **A person stands in the way**: as today, Nav2 goes round or waits.

## Testing criteria

**Happy path**

- Load a floor, tap "I am here", and the robot's drawn position follows
  it as it is driven round by hand.
- From 10 starting spots, `go_to` a saved place 30 m or more away: within
  0.3 m in at least 9 of 10.
- The same from the panel by tapping a place, and by tapping the map.
- Save a place by standing there, and by dropping a pin; go to both.
- By voice: it says the place back, then goes.

**Edge cases**

- Restart Nav2 mid-floor: it refuses to go until told where it is.
- Carry it 5 m and set it down: it reports lost, or corrects itself; it
  does not drive on a wrong position.
- Close a door on the route: it re-plans or stops with "blocked".
- Load the wrong floor on purpose: it never reports localized.
- Stop the relay service mid-route: it stops, as it does today.
- Send goals from two panels: the last one wins.
- A pin dropped inside a wall: refused with a reason.
- Drive the length of the longest corridor and back: the drawn position
  ends within 0.5 m of where the robot really is.
- Past a glass section: it neither gets lost nor walks into the glass.
- Load: camera frames dropped per minute with localization on, against
  today's figure.

**Without the robot**

- The places file: save, rename, delete, duplicate names, a damaged file.
- `go_to` and the refusals, against the pretend Nav2 used by
  `demos/check_motion.py`.
- The panel's Map view driven in a browser with a fake connection.

## Dependencies

- Step 1 of 2026-10-08, working: `/odom_fused`, `/scan`, the scan layer
  (`lio_relay.service`, the Orin's `lidar-scan.service`).
- A floor map from the Orin's mapping run for the first test floor, and a
  way to copy it across (the Orin has `robot_send.sh`; to be checked).
- Nav2's map server and AMCL in the robot's Foxy Nav2 build. The vendor
  tree has a map-based config (`dr_nav2`); whether it builds and runs is
  to be checked first.
- The lidar's position on the body, still a guess (0.20, 0, 0.10 m). It
  matters more for localization than it did for obstacles, so it is
  measured before phase 1 is judged.
- Open from step 1: whether the floor leaks into the scan while trotting,
  and why FAST-LIO2's poses arrive at about 8 a second.
- For phase 5: a tag library that runs on the perception computer or the
  Orin, and printed tags.
- For phase 4: the Gemini talker in `robot/talk.py`.
