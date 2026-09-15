import traceback
from lite3 import Lite3, Lite3Error

results = []
def step(name, fn):
    try:
        r = fn()
        results.append((name, 'OK', r))
        print('[OK]   %-16s %s' % (name, r))
        return r
    except Exception as e:
        results.append((name, 'FAIL', str(e)))
        print('[FAIL] %-16s %s' % (name, e))
        return None

with Lite3() as bot:
    print('start:', bot.status())
    try:
        step('stand', lambda: bot.stand())
        print('       status:', bot.status())

        clear = bot.clearance()
        print('       clearance ahead: %.2f m' % clear)
        dist = 0.5 if clear > 1.1 else max(0.0, clear - 0.6)
        if dist >= 0.2:
            r = step('walk %.2fm' % dist, lambda: bot.walk(dist))
            if r: print('       moved %.3f m (%s)' % (r['moved'], r['reason']))
        else:
            print('[SKIP] walk - only %.2f m of clearance' % clear)

        r = step('turn +90', lambda: bot.turn_deg(90))
        if r: print('       turned %.1f deg' % r['turned_deg'])
        r = step('turn -90', lambda: bot.turn_deg(-90))
        if r: print('       turned %.1f deg' % r['turned_deg'])

        step('nav_start', lambda: bot.nav_start())
        if bot.nav_running:
            profile = bot.cost_ahead(out_to=3.0)
            print('       cost ahead:', [(round(d,2), c) for d, c in profile])
            free = [d for d, c in profile if c is not None and c < 50 and d >= 0.75]
            goal = max(free) - 0.25 if free else 0
            if goal >= 0.5:
                st = step('goto %.2fm' % goal, lambda: bot.goto(goal, timeout=90))
                print('       action status %s (4=SUCCEEDED, 6=ABORTED)' % st)
            else:
                print('[SKIP] goto - no free cell beyond 0.75 m')
            step('nav_stop', lambda: bot.nav_stop())
    finally:
        step('sit', lambda: bot.sit())
        print('end:', bot.status())

print('\n==== SUMMARY ====')
for n, s, _ in results:
    print('  %-14s %s' % (n, s))
