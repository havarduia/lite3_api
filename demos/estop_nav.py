import math, os, signal, threading, time
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot.lite3 import Lite3

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
    prof = bot.cost_ahead(out_to=3.0)
    print('cost ahead:', [(round(d, 2), c) for d, c in prof])
    free = [d for d, c in prof if c is not None and c < 50 and d >= 0.75]
    if not free:
        print('ABORT: no free costmap cell beyond 0.75 m')
        raise SystemExit(0)
    goal = min(1.75, max(free) - 0.25)
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
        moved = math.hypot(hit[0] - t0_pose[0], hit[1] - t0_pose[1])
        print('INTERRUPTED after walking %.3f m' % moved)

    if hit:
        time.sleep(3.0)
        end = bot.pose
        print('COAST after e-stop: %.3f m' % math.hypot(end[0]-hit[0], end[1]-hit[1]))
    print('nav2 still running: %s   (False = stack killed)' % bot.nav_running)
    print('final:', bot.status())
