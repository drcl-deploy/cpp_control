#!/usr/bin/env python3
"""Calibration bag -> the replay rig's read_XXXX.npz, labels included."""

#     repose_export_observe_bag.py <bag> <out_dir>
#     repose_planner_selftest --replay <out_dir> --config <yaml> --summary
#
# Reads the sqlite3 bag directly: the recorded types are the planner's own, and a
# bag recorded before a rename cannot be deserialized by the installed messages
# at all. The five topics are paired on the exact header stamp the observe node
# stamps them all with.

import argparse
import os
import sqlite3
import struct
import sys

import numpy as np

# The observe node published under /vibe/sys1/... before the level-3 rename.
PREFIXES = ('/vibe/planner/calibration', '/vibe/sys1/calibration')


class Cdr:
    """Just enough CDR to read the calibration topics, alignment included."""

    def __init__(self, buf):
        self.b, self.p, self.o = buf, 4, 4
        self.le = buf[1] == 1

    def _align(self, n):
        m = (self.p - self.o) % n
        if m:
            self.p += n - m

    def _get(self, fmt, n):
        self._align(n)
        v = struct.unpack_from(('<' if self.le else '>') + fmt, self.b, self.p)[0]
        self.p += n
        return v

    def i32(self):
        return self._get('i', 4)

    def u32(self):
        return self._get('I', 4)

    def u64(self):
        return self._get('Q', 8)

    def f32(self):
        return self._get('f', 4)

    def f64(self):
        return self._get('d', 8)

    def b8(self):
        v = self.b[self.p]
        self.p += 1
        return bool(v)

    def string(self):
        n = self.u32()
        v = self.b[self.p:self.p + n - 1].decode('utf8', 'replace')
        self.p += n
        return v

    def stamp(self):
        sec, nsec = self.i32(), self.u32()
        self.string()  # frame_id
        return sec * 10 ** 9 + nsec


def read_state(buf):
    r = Cdr(buf)
    return {'stamp': r.stamp(), 'source_sequence': r.u64(), 'expected': r.i32()}


def read_observation(buf):
    r = Cdr(buf)
    d = {'stamp': r.stamp(), 'sequence': r.u64(), 'state_ready': r.b8(),
         'stand_locked': r.b8(), 'color_ok': r.b8(), 'pose_ok': r.b8(),
         'color': r.i32(), 'reason': r.string()}
    d['position'] = [r.f32() for _ in range(3)]
    d['phi'] = r.f32()
    d['n_px'] = r.i32()
    d['range_m'] = r.f32()
    d['bearing'] = r.f32()
    d['R'] = [r.f32() for _ in range(9)]
    d['t'] = [r.f32() for _ in range(3)]
    return d


def read_image(buf):
    r = Cdr(buf)
    stamp = r.stamp()
    h, w = r.u32(), r.u32()
    enc = r.string()
    r.b8()
    r.u32()  # is_bigendian, step
    n = r.u32()
    return {'stamp': stamp, 'h': h, 'w': w, 'enc': enc,
            'data': r.b[r.p:r.p + n]}


def read_compressed(buf):
    r = Cdr(buf)
    stamp = r.stamp()
    r.string()  # format
    n = r.u32()
    return {'stamp': stamp, 'data': r.b[r.p:r.p + n]}


def read_camera_info(buf):
    r = Cdr(buf)
    stamp = r.stamp()
    r.u32(), r.u32()
    r.string()  # height, width, distortion_model
    nd = r.u32()
    for _ in range(nd):
        r.f64()
    k = [r.f64() for _ in range(9)]
    return {'stamp': stamp, 'fx': k[0], 'fy': k[4], 'cx': k[2], 'cy': k[5]}


def load(db_path):
    con = sqlite3.connect(db_path)
    topics = {name: tid for tid, name, _ in
              con.execute('select id, name, type from topics')}

    def grab(suffix, parse):
        for prefix in PREFIXES:
            tid = topics.get(prefix + suffix)
            if tid is None:
                continue
            rows = con.execute(
                'select data from messages where topic_id = ? order by timestamp',
                (tid,))
            return [parse(bytes(d[0])) for d in rows]
        raise SystemExit(f'export: no topic {{{",".join(PREFIXES)}}}{suffix} in the bag')

    return (grab('/state', read_state), grab('/observation', read_observation),
            grab('/color/compressed', read_compressed), grab('/depth', read_image),
            grab('/camera_info', read_camera_info))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='bag directory, or its .db3')
    ap.add_argument('out_dir')
    args = ap.parse_args()

    try:
        import cv2
    except ImportError:
        raise SystemExit('export: needs opencv-python to decode the colour plane')

    db = args.bag
    if os.path.isdir(db):
        found = [f for f in sorted(os.listdir(db)) if f.endswith('.db3')]
        if not found:
            raise SystemExit(f'export: no .db3 under {db}')
        db = os.path.join(db, found[0])

    states, obs, colors, depths, infos = load(db)
    by_stamp = {}
    for group, key in ((states, 'state'), (obs, 'obs'), (colors, 'color'),
                       (depths, 'depth'), (infos, 'info')):
        for m in group:
            by_stamp.setdefault(m['stamp'], {})[key] = m

    os.makedirs(args.out_dir, exist_ok=True)
    kept = skipped = 0
    for stamp in sorted(by_stamp):
        rec = by_stamp[stamp]
        if len(rec) != 5 or not rec['obs']['state_ready']:
            skipped += 1
            continue
        depth = rec['depth']
        if depth['enc'] != '16UC1':
            raise SystemExit(f"export: depth encoding {depth['enc']} is not 16UC1")
        z = (np.frombuffer(depth['data'], np.uint16)
             .reshape(depth['h'], depth['w']).astype(np.float32) / 1000.0)
        bgr = cv2.imdecode(np.frombuffer(rec['color']['data'], np.uint8),
                           cv2.IMREAD_COLOR)
        if bgr is None:
            skipped += 1
            continue
        info = rec['info']
        np.savez(os.path.join(args.out_dir, f'read_{kept:04d}.npz'),
                 bgr=np.ascontiguousarray(bgr),
                 depth=np.ascontiguousarray(z),
                 # Intrinsics are the DEPTH plane's own pixels, which is the
                 # plane CubeSight unprojects — the colour plane is resized onto
                 # it, never the other way round.
                 intrinsics=np.array([info['fx'], info['fy'], info['cx'], info['cy']],
                                     np.float32),
                 R_bc=np.array(rec['obs']['R'], np.float32),
                 t_bc=np.array(rec['obs']['t'], np.float32),
                 label=np.int32(rec['state']['expected']))
        kept += 1

    print(f'export: {kept} reads -> {args.out_dir} ({skipped} skipped)')
    if not kept:
        sys.exit(1)


if __name__ == '__main__':
    main()
