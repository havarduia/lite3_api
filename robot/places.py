"""places - named places on a floor map, kept in a file beside the map.

    ~/lite3_maps/<floor>/map.pgm, map.yaml     the floor map (env/get_map.sh)
    ~/lite3_maps/<floor>/places.json           this module's

    python3 -m robot.places check    self-check, anywhere

No ROS in here. Every call reads the file again: the panel and a script may
both be changing it.
"""
import datetime
import json
import math
import os
import sys
import tempfile

from .protocol import Lite3Error

ROOT = os.path.expanduser('~/lite3_maps')
NAME_MAX = 60


def floors(root=None):
    """The floors that have a map, by name."""
    root = root or ROOT
    if not os.path.isdir(root):
        return []
    return sorted(f for f in os.listdir(root)
                  if os.path.isfile(os.path.join(root, f, 'map.yaml')))


def floor_dir(floor, root=None):
    """Where a floor's files are. Only a floor that is there: the name comes
    from whoever has the panel open, and must not become a path."""
    if floor not in floors(root):
        raise Lite3Error('no floor %r; there is %s' % (floor, ', '.join(floors(root)) or 'none'))
    return os.path.join(root or ROOT, floor)


def _file(floor, root):
    return os.path.join(floor_dir(floor, root), 'places.json')


def _key(name):
    return name.strip().casefold()


def load(floor, root=None):
    """The places on a floor: [{name, x, y, yaw_deg, tag_id, created_at}, ...].
    A file that cannot be read is moved aside, never written over."""
    path = _file(floor, root)
    if not os.path.exists(path):
        return []
    try:
        with open(path) as f:
            places = json.load(f)['places']
        for p in places:
            _check(p['name'], p['x'], p['y'], p['yaw_deg'])
    except (ValueError, KeyError, TypeError, Lite3Error) as e:
        aside = '%s.damaged-%s' % (path, _now().replace(':', ''))
        os.replace(path, aside)
        raise Lite3Error('places.json of %r could not be read (%s); it is kept as %s '
                         'and the floor now has no places' % (floor, e, os.path.basename(aside)))
    return places


def find(floor, name, root=None):
    """The place of that name (case does not matter)."""
    for p in load(floor, root):
        if _key(p['name']) == _key(name):
            return p
    raise Lite3Error('no place %r on %r' % (name, floor))


def save(floor, name, x, y, yaw_deg, tag_id=None, root=None):
    """Add a place. Refuses an empty name and one already used on this floor."""
    name = _check(name, x, y, yaw_deg)
    places = load(floor, root)
    _free(places, name)
    place = {'name': name, 'x': round(float(x), 3), 'y': round(float(y), 3),
             'yaw_deg': round(float(yaw_deg), 1), 'tag_id': tag_id, 'created_at': _now()}
    _write(floor, places + [place], root)
    return place


def rename(floor, old, new, root=None):
    places = load(floor, root)
    place = _one(places, old, floor)
    new = _check(new, 0.0, 0.0, 0.0)
    if _key(new) != _key(old):
        _free(places, new)
    place['name'] = new
    _write(floor, places, root)
    return place


def delete(floor, name, root=None):
    """Remove a place for good; the panel's export is the backup."""
    places = load(floor, root)
    places.remove(_one(places, name, floor))
    _write(floor, places, root)


def _one(places, name, floor):
    for p in places:
        if _key(p['name']) == _key(name):
            return p
    raise Lite3Error('no place %r on %r' % (name, floor))


def _free(places, name):
    for p in places:
        if _key(p['name']) == _key(name):
            raise Lite3Error('there is already a place called %r on this floor' % p['name'])


def _check(name, x, y, yaw_deg):
    if not isinstance(name, str) or not name.strip():
        raise Lite3Error('a place needs a name')
    name = name.strip()
    if len(name) > NAME_MAX or not name.isprintable():
        raise Lite3Error('a place name is at most %d ordinary characters' % NAME_MAX)
    for v in (x, y, yaw_deg):
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise Lite3Error('a place needs a position and a heading in numbers, not %r' % (v,))
    return name


def _now():
    return datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')


def _write(floor, places, root):
    """Whole file to a temporary one, then swapped in: a crash half way leaves
    the old file, not half a new one."""
    path = _file(floor, root)
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path), prefix='.places-')
    try:
        with os.fdopen(fd, 'w') as f:
            json.dump({'floor': floor, 'places': places}, f, indent=1)
            f.write('\n')
        os.replace(tmp, path)
    except BaseException:
        os.unlink(tmp)
        raise


def demo():
    def refused(fn, *a, **kw):
        try:
            fn(*a, **kw)
        except Lite3Error as e:
            return str(e)
        raise AssertionError('%s%r was not refused' % (fn.__name__, a))

    with tempfile.TemporaryDirectory() as root:
        assert floors(root) == [] and floors(root + '/nowhere') == []
        for floor in ('uia-2', 'uia-3'):
            os.makedirs(os.path.join(root, floor))
            open(os.path.join(root, floor, 'map.yaml'), 'w').close()
        os.makedirs(os.path.join(root, 'no-map-here'))
        assert floors(root) == ['uia-2', 'uia-3']
        kw = {'root': root}
        # a floor name cannot reach outside the folder
        for bad in ('../uia-2', 'uia-2/..', 'no-map-here', '', '.'):
            assert 'no floor' in refused(load, bad, **kw)

        assert load('uia-2', **kw) == []
        p = save('uia-2', '  Lab door ', 12.4004, -3.1, 90, **kw)
        assert p['name'] == 'Lab door' and p['x'] == 12.4 and p['tag_id'] is None
        assert p['created_at'].endswith('Z') and len(p['created_at']) == 20      # UTC, ISO 8601
        assert find('uia-2', 'LAB DOOR', **kw) == p and load('uia-3', **kw) == []
        on_disk = json.load(open(os.path.join(root, 'uia-2', 'places.json')))
        assert on_disk == {'floor': 'uia-2', 'places': [p]}

        # names: unique per floor whatever the case, not empty, not on another floor's account
        assert 'already' in refused(save, 'uia-2', 'lab DOOR', 0, 0, 0, **kw)
        assert 'needs a name' in refused(save, 'uia-2', '   ', 0, 0, 0, **kw)
        assert 'at most' in refused(save, 'uia-2', 'x' * 61, 0, 0, 0, **kw)
        assert 'numbers' in refused(save, 'uia-2', 'nan', float('nan'), 0, 0, **kw)
        assert 'numbers' in refused(save, 'uia-2', 'text', '1', 0, 0, **kw)
        save('uia-3', 'Lab door', 1, 2, 3, **kw)
        save('uia-2', 'Canteen', 1, 2, 3, **kw)

        assert 'already' in refused(rename, 'uia-2', 'canteen', 'LAB DOOR', **kw)
        assert rename('uia-2', 'canteen', 'Canteen ', **kw)['name'] == 'Canteen'    # its own name, respelt
        assert rename('uia-2', 'canteen', 'Kantina', **kw)['name'] == 'Kantina'
        assert 'no place' in refused(find, 'uia-2', 'canteen', **kw)
        assert 'no place' in refused(delete, 'uia-2', 'canteen', **kw)
        delete('uia-2', 'KANTINA', **kw)
        assert [q['name'] for q in load('uia-2', **kw)] == ['Lab door']
        assert not [f for f in os.listdir(os.path.join(root, 'uia-2')) if f.startswith('.places-')]

        # a damaged file is moved aside and said so, once; it is never written over
        path = os.path.join(root, 'uia-2', 'places.json')
        for damage in ('{"places": [{"name": "half', '{"places": [{"name": "no position"}]}', '[]'):
            with open(path, 'w') as f:
                f.write(damage)
            assert 'kept as places.json.damaged-' in refused(save, 'uia-2', 'New', 0, 0, 0, **kw)
            kept = [f for f in os.listdir(os.path.dirname(path)) if '.damaged-' in f]
            assert len(kept) == 1 and open(os.path.join(os.path.dirname(path), kept[0])).read() == damage
            assert load('uia-2', **kw) == []
            os.unlink(os.path.join(os.path.dirname(path), kept[0]))
    print('places ok')


if __name__ == '__main__':
    if sys.argv[1:] == ['check']:
        demo()
    else:
        print(__doc__)
