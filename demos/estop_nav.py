import os, signal, sys, threading, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot.lite3 import Lite3
from robot.nav import dist, goal_ahead

with Lite3() as bot:
    print('battery %s%%  basic=%s' % (bot.state.get('battery'), bot.state.get('basic')))
    bot.stand()
    time.sleep(1.5)
    clear = bot.clearance()
    print('clearance straight ahead: %.2f m' % clear)
    if clear < 1.2:
        print('ABORT: less than 1.2 m ahead, not running a walking test')
        raise SystemExit(0)

    bot.nav_start()
    prof = bot.cost_ahead(out_to=3.0, settle=10)
    print('cost ahead:', [(round(d, 2), c) for d, c in prof])
    goal = goal_ahead(prof)
    if goal is None:
        print('ABORT: under 0.75 m of free costmap ahead')
        raise SystemExit(0)
    goal = min(1.75, goal)
    print('goal: %.2f m straight ahead' % goal)

    t0_pose = bot.pose
    # Fires a REAL SIGINT at this process - identical to you pressing Ctrl-C.
    threading.Timer(6.0, lambda: os.kill(os.getpid(), signal.SIGINT)).start()

    hit = None
    try:
        st = bot.goto(goal, timeout=60)
        print('goal finished on its own, status %s (no interrupt tested)' % st)
    except KeyboardInterrupt:
        hit = bot.pose
        print('INTERRUPTED after walking %.3f m' % dist(t0_pose, hit))

    if hit:
        time.sleep(3.0)
        print('COAST after e-stop: %.3f m' % dist(hit, bot.pose))
    print('nav2 still running: %s   (False = stack killed)' % bot.nav_running)
    print('final:', bot.status())
