# lite3-robot

Python API and behaviours for a **DeepRobotics Lite3 Pro** running **ROS2 Foxy**,
on the robot's perception computer (Jetson Xavier NX, Ubuntu 20.04, Python 3.8).

## Layout

    robot/      the library - import this
      lite3.py      the API: motion, state, depth, Nav2, e-stop, heartbeat
      voice.py      audio out: Piper TTS, clip playback, YouTube, the speaker
      rocky.py      the talking persona, on Gemini Live with a REST fallback
      person.py     person detection and following
      run_robot.py  Mission: a scripted run with per-step logging
    bin/        things you run
      rocky_walk.py   walk or navigate a route, stop, look, talk
      robot_cmd.py    raw EthCommand sender - the escape hatch for unwrapped codes
    demos/      teaching and regression scripts
      demo.py        a guided tour of the API, lesson by lesson
      shake.py       end-to-end regression test          (MOVES THE ROBOT)
      estop_nav.py   e-stop behaviour under Nav2         (MOVES THE ROBOT)
    env/        environment and launchers
      lite3_env.sh          source this first - five ROS2 workspaces, CycloneDDS, PYTHONPATH
      start_nav2_mapless.sh mapless Nav2 stack (lite3.py starts this for you)
      start_realsense_slow.sh  RealSense at depth_fps 6, for diagnosing USB faults
      cyclone_dds.xml
    archive/    pre-git history: the old .bak files, and superseded scripts

## Running things

    source ~/robot/env/lite3_env.sh
    python3 ~/robot/bin/rocky_walk.py "walk 1.5, turn 90"

Sourcing `lite3_env.sh` also puts the repo root on `PYTHONPATH`, so this works
from anywhere afterwards:

    from robot.lite3 import Lite3
    with Lite3() as bot:
        bot.stand()

The scripts in `bin/` and `demos/` add the repo root to `sys.path` themselves,
so they run even without sourcing - but they still need the ROS2 workspaces
that `lite3_env.sh` sets up, so source it anyway.

Paths are resolved relative to this directory (`_repo()` in `lite3.py`,
`$BASH_SOURCE` in the shell scripts), so the repo can be moved or cloned
anywhere without editing anything.

## Things that will bite you

- **The handheld is the only physical e-stop.** `heartbeat_start()` makes this
  process the controller, so the robot obeys with no handheld powered on at
  all - and then the terminal is your stop, not the handheld. `Lite3` installs
  a Ctrl-C e-stop by default (`estop_on_sigint=True`).
- **Stand before starting Nav2.** Standing jumps leg odometry by ~1.3 m in one
  step. A costmap built across that jump shows a clear path into a real wall.
- **`realsense_ros2.service` does not come back after a reboot.** Start it
  manually or anything depending on depth raises "no point cloud".
- **`shake.py` and `estop_nav.py` move the robot.** `demos/demo.py` needs
  `--move` for lessons 5 and up; without it they are skipped.
- Dependencies come from the ROS2 Foxy workspaces, not PyPI. `pyproject.toml`
  is metadata only - do not expect `pip install` to work here.
