# Archived 2026-09-14 - superseded by ~/lite3.py

Nothing here is called by any systemd unit, cron job or vendor code; it was
checked before archiving. Restore any of it with `mv <file> ~/`.

Replaced by the API:
  walk_forward.py    -> bot.walk() / bot.strafe() / bot.turn()
  scan.py            -> bot.scan() / bot.scan_text()
  clearance.py       -> bot.clearance()
  costmap_probe.py   -> bot.cost_ahead() / bot.cost_at()
  send_goal.py       -> bot.goto()
  check_state.py,
  motion_query.py    -> bot.state / bot.status()
  check_camera.py    -> bot.wait_cloud()
  check_costmap.py,
  check_costmap_region.py -> bot.cost_at()

One-off experiments, kept for their findings (all recorded in the project notes):
  failsafe_test.py        velocity failsafe: robot coasts ~0.083 m and stops
                          ~1.5 s after the publisher dies. Its built-in 0.05 m
                          pass threshold is too tight - a legged robot needs ~0.1 m.
  obstacle_test.py,
  voa_test.py             VOA characterisation (steering avoider, right-bias,
                          blind below 10 cm - NOT an emergency stop)
  try_stand.py,
  test_cmdvel*.py,
  watch_state_and_move.py, test_plan.py
                          early cmd_vel / stand experiments, before the
                          auto-mode requirement was understood
  nav_test_logs/          logs from the first mapless nav runs

Still live in ~:
  lite3.py                the API
  lite3_env.sh            sources all five workspaces + CycloneDDS
  start_nav2_mapless.sh   launched by bot.nav_start()
  shake.py                end-to-end regression test for the API
  robot_cmd.py            raw EthCommand sender - the escape hatch for codes
                          lite3.py does not wrap (e.g. VOA 0x21012109)
