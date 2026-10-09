#!/usr/bin/env python3
"""nav2_map_params - Nav2's parameters for map mode, made from the mapless ones.

    python3 env/nav2_map_params.py <mapless.yaml> <map.yaml> <out.yaml>
    python3 env/nav2_map_params.py check

Map mode is the mapless stack with a floor map under it: the global costmap
moves to the `map` frame and gets the map as its first layer, and a map
server and AMCL are added. Everything else is the mapless file as it stands
on the robot, so its tuning (env/nav2-mapless-lite3.patch) is done once.
"""
import copy
import sys

import yaml

# AMCL, from the block the Orin's ~/robot_pc/nav2_params.yaml had for this robot.
AMCL = {
    'use_sim_time': False,
    'base_frame_id': 'base_link',
    'odom_frame_id': 'odom',
    'global_frame_id': 'map',
    'scan_topic': 'scan',
    'robot_model_type': 'omnidirectional',      # a legged robot can step sideways
    'laser_model_type': 'likelihood_field',
    'laser_min_range': 0.4,                     # nearer is the robot itself (the Orin's scan drops it)
    'laser_max_range': 30.0,
    'max_beams': 120,
    'min_particles': 500,
    'max_particles': 3000,
    # leg odometry slips more than wheels; not tuned on the floor yet
    'alpha1': 0.3, 'alpha2': 0.3, 'alpha3': 0.3, 'alpha4': 0.3, 'alpha5': 0.3,
    'update_min_d': 0.15,
    'update_min_a': 0.15,
    'resample_interval': 1,
    'transform_tolerance': 0.5,
    'recovery_alpha_fast': 0.0,
    'recovery_alpha_slow': 0.0,
    'z_hit': 0.5, 'z_rand': 0.5, 'z_max': 0.05, 'z_short': 0.05,
    'sigma_hit': 0.2,
    'lambda_short': 0.1,
    'laser_likelihood_max_dist': 2.0,
    'pf_err': 0.05, 'pf_z': 0.99,
    'do_beamskip': False,
    'save_pose_rate': 0.5,
    'tf_broadcast': True,
    'set_initial_pose': False,                  # it waits to be told where it is
}

STATIC_LAYER = {
    'plugin': 'nav2_costmap_2d::StaticLayer',
    'map_subscribe_transient_local': True,
}
# The global costmap takes the map's size and cell (0.05 m), about four
# times the cells of the mapless one for a 35 m floor, so it is redone and
# sent less often. The local costmap, which steers, is untouched.
GLOBAL_UPDATE_HZ = 1.0
GLOBAL_PUBLISH_HZ = 0.5


def map_params(mapless, map_yaml):
    """The mapless parameters (as a dict) -> map mode's."""
    p = copy.deepcopy(mapless)
    p['bt_navigator']['ros__parameters']['global_frame'] = 'map'
    g = p['global_costmap']['global_costmap']['ros__parameters']
    g['global_frame'] = 'map'
    g['rolling_window'] = False             # the whole floor, not a window round the robot
    g['update_frequency'] = GLOBAL_UPDATE_HZ
    g['publish_frequency'] = GLOBAL_PUBLISH_HZ
    g['plugins'] = ['static_layer'] + g['plugins']
    g['static_layer'] = dict(STATIC_LAYER)
    p['map_server'] = {'ros__parameters': {'use_sim_time': False, 'yaml_filename': map_yaml}}
    p['amcl'] = {'ros__parameters': dict(AMCL)}
    return p


def changed(a, b, at=''):
    """The paths at which two nested dicts differ."""
    if isinstance(a, dict) and isinstance(b, dict):
        return sorted(sum((changed(a.get(k), b.get(k), at + '/' + str(k))
                           for k in set(a) | set(b)), []))
    return [] if a == b and type(a) is type(b) else [at]


def demo():
    mapless = yaml.safe_load('''
bt_navigator:
  ros__parameters: {global_frame: odom, robot_base_frame: base_link}
local_costmap:
  local_costmap:
    ros__parameters: {global_frame: odom, rolling_window: True, plugins: ["scan_layer", "inflation_layer"], update_frequency: 5.0}
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: odom
      rolling_window: True
      width: 30
      footprint: "[[0.3, 0.2], [0.3, -0.2]]"
      update_frequency: 5.0
      publish_frequency: 2.0
      plugins: ["scan_layer", "inflation_layer"]
      scan_layer: {plugin: "nav2_costmap_2d::ObstacleLayer", scan: {max_obstacle_height: 2.0}}
recoveries_server:
  ros__parameters: {global_frame: odom}
''')
    before = copy.deepcopy(mapless)
    out = map_params(mapless, '/maps/f/map.yaml')
    assert mapless == before                                    # the input is not touched
    g = '/global_costmap/global_costmap/ros__parameters/'
    assert changed(mapless, out) == sorted([
        '/amcl', '/map_server', '/bt_navigator/ros__parameters/global_frame',
        g + 'global_frame', g + 'rolling_window', g + 'plugins', g + 'static_layer',
        g + 'update_frequency', g + 'publish_frequency']), changed(mapless, out)
    gp = out['global_costmap']['global_costmap']['ros__parameters']
    assert gp['plugins'] == ['static_layer', 'scan_layer', 'inflation_layer']   # the map first, obstacles over it
    assert gp['global_frame'] == 'map' and gp['rolling_window'] is False
    assert out['local_costmap'] == mapless['local_costmap']     # steering stays in odom
    assert out['recoveries_server']['ros__parameters']['global_frame'] == 'odom'
    assert out['map_server']['ros__parameters']['yaml_filename'] == '/maps/f/map.yaml'
    assert out['amcl']['ros__parameters']['base_frame_id'] == 'base_link'
    # written and read back it is the same, types included (a float must not come back a string)
    assert changed(out, yaml.safe_load(yaml.safe_dump(out))) == []
    print('nav2_map_params ok')


if __name__ == '__main__':
    args = sys.argv[1:]
    if args == ['check']:
        demo()
    elif len(args) == 3:
        with open(args[0]) as f:
            made = map_params(yaml.safe_load(f), args[1])
        with open(args[2], 'w') as f:
            f.write('# Made by env/nav2_map_params.py from %s. Do not edit: change that file.\n' % args[0])
            yaml.safe_dump(made, f)
    else:
        sys.exit(__doc__)
