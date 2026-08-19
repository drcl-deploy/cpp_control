#!/usr/bin/env python3
"""Fit the observe block to a labelled bag: Python fits, C++ scores."""

#     repose_export_observe_bag.py <bag> /tmp/reads
#     repose_tune_observe.py --reads /tmp/reads --write <g1_real.yaml>
#
# The palette is measured here, offline. Every score printed comes from running
# the PRODUCTION CubeSight over the same reads (`repose_planner_selftest
# --replay --config`), so a sweep can never tune against a second implementation
# of the read that has drifted from the deployed one.
#
# The cube-top mask used for MEASURING is geometry-only — a horizontal plane at
# the top of the depth blob — so the palette it produces carries no prior from
# the palette being replaced.

import argparse
import glob
import itertools
import os
import re
import shutil
import subprocess
import sys
import tempfile

import numpy as np

COLORS = ('red', 'orange', 'green', 'yellow', 'blue', 'pink')
SCORE_RE = re.compile(r'SCORE accuracy ([\d.]+) \((\d+)/(\d+)\)\s+false_positives (\d+)/(\d+)')


# ── measuring ────────────────────────────────────────────────────

def cube_top(bgr, depth, intr, R, t, band_m, half_extent):
    """Geometry-only top face: no colour prior of any kind. None if not found."""
    import cv2
    h, w = depth.shape
    if bgr.shape[:2] != depth.shape:
        bgr = cv2.resize(bgr, (w, h), interpolation=cv2.INTER_AREA)
    fx, fy, cx, cy = intr
    u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    rays = np.stack([(u - cx) / fx, -(v - cy) / fy, -np.ones_like(u)], -1)
    pts = (rays * depth[..., None]) @ R.reshape(3, 3).T + t
    ok = np.isfinite(depth) & (depth > 0.3) & (depth < 3.0)
    if ok.sum() < 400:
        return None
    zb = pts[..., 2]
    z_top = np.percentile(zb[ok], 99)
    m = ok & (np.abs(zb - z_top) <= band_m)
    if m.sum() < 150:
        return None
    n, cc, stats, _ = cv2.connectedComponentsWithStats(m.astype(np.uint8), 8)
    if n < 2:
        return None
    m = cc == (1 + np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    m = cv2.erode(m.astype(np.uint8), np.ones((3, 3), np.uint8), iterations=1).astype(bool)
    if m.sum() < 60:
        return None
    q = pts[m] - pts[m].mean(0)
    _, vecs = np.linalg.eigh(q.T @ q)
    if abs(vecs[:, 0][2]) < 0.9:          # not a horizontal face: do not measure it
        return None
    return bgr[m][:, ::-1].astype(np.float32)   # RGB


def chrom(rgb):
    s = np.maximum(rgb.sum(-1, keepdims=True), 1e-6)
    return (rgb / s)[..., :2]


def to_rgb255(rg):
    """Chromaticity back to an 8-bit triple. Only the ratio is ever read."""
    r, g = rg
    v = np.array([r, g, max(1.0 - r - g, 0.0)])
    return [int(round(x)) for x in v / max(v.max(), 1e-6) * 255]


def fit_palette(reads, band_m, half_extent, min_frames=5):
    """Per colour, a lit and a shaded reference off the frame pool."""
    # Split at the pool's own median value. A colour with too few clean frames
    # keeps the base palette rather than being fitted from noise.
    pooled = {c: [] for c in range(6)}
    used = {c: 0 for c in range(6)}
    for r in reads:
        if r['label'] < 0:
            continue
        px = cube_top(r['bgr'], r['depth'], r['intrinsics'], r['R_bc'], r['t_bc'],
                      band_m, half_extent)
        if px is None:
            continue
        pooled[r['label']].append(px)
        used[r['label']] += 1
    out = {}
    for c in range(6):
        if used[c] < min_frames:
            continue
        px = np.concatenate(pooled[c])
        val = px.max(1)
        cut = np.median(val)
        lit, shaded = px[val >= cut], px[val < cut]
        if len(lit) < 50 or len(shaded) < 50:
            lit = shaded = px
        out[COLORS[c]] = {'lit': to_rgb255(np.median(chrom(lit), 0)),
                          'shaded': to_rgb255(np.median(chrom(shaded), 0)),
                          'frames': used[c], 'pixels': int(len(px))}
    return out


# ── the config under test ────────────────────────────────────────

def observe_block(palette, knobs, provenance=()):
    L = list(provenance) + [
        'observe:',
        f"  read: {knobs['read']}",
        '  proc_width: 0',
        '',
        '  # MEASURED, not designed: medians over the geometry-only cube-top',
        '  # masks of the labelled bag this file was tuned from. lit/shaded is',
        '  # that pool split at its own median value.',
        '  palette:']
    for name in COLORS:
        p = palette.get(name)
        if p is None:
            L.append(f'    # {name}: too few clean frames to measure — keeps the preset row')
            continue
        L.append(f"    {name+':':<8}{{lit: {p['lit']}, shaded: {p['shaded']}}}"
                 f"   # {p['frames']} frames, {p['pixels']} px")
    L += [
        '',
        '  gates:',
        f"    min_value: {knobs['min_value']}",
        f"    min_rel_sat: {knobs['min_rel_sat']}",
        '    min_area_frac: 0.002',
        '    min_px_floor: 8',
        f"    chroma_reject: {knobs['chroma_reject']}   # 0 = every lit pixel votes",
        '',
        '  geometry:',
        '    up_dot_min: 0.90',
        '    min_visible: 0.60',
        '    min_visible_color: 0.0',
        '    big_max: 1.4',
        '    slab_frac: 0.25',
        '    z_min_m: 0.05',
        '',
        '  mask:',
        '    top_pct: 97.0',
        f"    band_m: {knobs['band_m']}",
        '    erode: 1',
        '']
    return '\n'.join(L) + '\n'


def splice(base_text, block):
    """Replace the whole top-level `observe:` block."""
    # Top-level keys are the only unindented lines, so the end is unambiguous.
    lines = base_text.splitlines(keepends=True)
    start = next((i for i, l in enumerate(lines) if l.startswith('observe:')), None)
    if start is None:
        raise SystemExit('tune: the base yaml has no top-level `observe:` block')
    end = start + 1
    while end < len(lines) and (lines[end].strip() == '' or lines[end][0] in ' \t#'):
        end += 1
    # keep trailing blank lines outside the block
    while end > start + 1 and lines[end - 1].strip() == '':
        end -= 1
    return ''.join(lines[:start]) + block + ''.join(lines[end:])


def score(cfg_text, args):
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as f:
        f.write(cfg_text)
        path = f.name
    try:
        out = subprocess.run(
            [args.selftest, '--replay', args.reads, '--config', path,
             '--table', args.table, '--summary'],
            capture_output=True, text=True)
        m = SCORE_RE.search(out.stdout)
        if not m:
            sys.stderr.write(out.stdout[-2000:] + out.stderr[-2000:])
            raise SystemExit('tune: the scorer printed no SCORE line')
        return float(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4)), out.stdout
    finally:
        os.unlink(path)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--reads', required=True, help='dir of read_XXXX.npz from the exporter')
    ap.add_argument('--base', default='config/repose_planner/g1.yaml')
    ap.add_argument('--selftest', default='repose_planner_selftest')
    ap.add_argument('--table', required=True, help='sys1_clips.npz, for half_extent')
    ap.add_argument('--write', help='yaml to splice the winning observe block into')
    ap.add_argument('--measure-band-m', type=float, default=0.06,
                    help='half-thickness of the MEASURING slab (not the runtime knob)')
    ap.add_argument('--max-fp', type=int, default=0,
                    help='most no-cube frames the winner may answer a colour on. '
                         'Default 0: seeing a cube that is not there makes the planner '
                         'ACT, which is not the same mistake as standing still')
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.reads, 'read_*.npz')))
    if not files:
        raise SystemExit(f'tune: no read_*.npz under {args.reads}')
    reads = [dict(np.load(f)) for f in files]
    for r in reads:
        r['label'] = int(r['label'])
    n_pos = sum(1 for r in reads if r['label'] >= 0)
    half_extent = float(np.load(args.table, allow_pickle=True)['half_extent'])
    print(f'tune: {len(files)} reads ({n_pos} labelled, {len(files)-n_pos} negative), '
          f'cube half-extent {half_extent:.4f} m')

    palette = fit_palette(reads, args.measure_band_m, half_extent)
    print(f'\nmeasured palette ({len(palette)}/6 colours):')
    for name in COLORS:
        p = palette.get(name)
        print(f'  {name:>7}  lit {str(p["lit"]):>18}  shaded {str(p["shaded"]):>18}'
              f'  ({p["frames"]} frames)' if p else f'  {name:>7}  -- too few clean frames')

    base = open(args.base).read()
    grid = {'read': ['blobs', 'mask'],
            'chroma_reject': [0.0, 0.045, 0.06, 0.08],
            'min_rel_sat': [0.10, 0.15, 0.22],
            'band_m': [0.05, 0.08, 0.12],
            'min_value': [30]}
    keys = list(grid)
    rows = []
    print(f'\nsweep ({int(np.prod([len(grid[k]) for k in keys]))} configs, production read):')
    print(f'  {"read":<6}{"reject":>8}{"rel_sat":>9}{"band_m":>8}{"acc":>9}{"FP":>7}')
    for combo in itertools.product(*(grid[k] for k in keys)):
        knobs = dict(zip(keys, combo))
        text = splice(base, observe_block(palette, knobs))
        acc, right, n, fp, _ = score(text, args)
        rows.append((acc, fp, knobs, text))
        print(f'  {knobs["read"]:<6}{knobs["chroma_reject"]:>8.3f}'
              f'{knobs["min_rel_sat"]:>9.2f}{knobs["band_m"]:>8.2f}'
              f'{acc*100:>8.1f}%{fp:>4}/{n_pos and len(files)-n_pos}')

    # False positives are a CONSTRAINT, not a tie-break. A wrong colour makes the
    # planner turn the cube the wrong way and the ladder recovers; a colour on an
    # empty floor makes it commit a clip at a cube that is not there.
    clean = [r for r in rows if r[1] <= args.max_fp]
    if not clean:
        best_fp_seen = min(r[1] for r in rows)
        print(f'\ntune: NOTHING in the sweep held to --max-fp {args.max_fp} '
              f'(best was {best_fp_seen}). Widen the grid or raise the bound '
              f'deliberately — do not let this pass silently.')
        sys.exit(1)
    clean.sort(key=lambda r: (-r[0], r[1]))
    best_acc, best_fp, best_knobs, best_text = clean[0]
    ceiling = max(rows, key=lambda r: r[0])
    if ceiling[0] > best_acc:
        print(f'\nnote: {ceiling[0]*100:.1f}% is reachable at {ceiling[1]} false '
              f'positives ({ceiling[2]["read"]}, reject {ceiling[2]["chroma_reject"]}, '
              f'rel_sat {ceiling[2]["min_rel_sat"]}) — not taken, --max-fp is '
              f'{args.max_fp}')
    base_acc = next((a for a, _, k, _ in rows if k['read'] == 'blobs'
                     and k['chroma_reject'] == 0.0 and k['min_rel_sat'] == 0.22), None)
    print(f'\nbest: {best_knobs}  ->  {best_acc*100:.1f}%, {best_fp} false positives')
    if base_acc is not None:
        print(f'shipping blobs/sim-gates on the same reads: {base_acc*100:.1f}%')

    if args.write:
        # Re-render the winner WITH its provenance. A tuned config that does not
        # say what it was tuned on, and against what, is a set of magic numbers.
        banner = [
            '# ── TUNED, not designed ──────────────────────────────────────────',
            '#',
            f'# repose_tune_observe.py, {len(files)} reads ({n_pos} labelled, '
            f'{len(files) - n_pos} negative)',
            f'# reads: {os.path.abspath(args.reads)}',
            f'# score: {best_acc * 100:.1f}% colour accuracy, {best_fp} false positives '
            f'on the no-cube frames,',
            f'#        against {base_acc * 100:.1f}% for the shipping blobs read on the '
            'same frames.',
            '#        Measured by the PRODUCTION CubeSight — repose_planner_selftest',
            '#        --replay <reads> --config <this file> reproduces it.',
            '#',
            '# The palette is bound to the camera state that recorded those frames.',
            "# The D435i's auto exposure and auto white balance were both ON, and the",
            '# blue face reads cyan because of it. Lock either one, or relight the',
            '# room, and these twelve numbers are stale — re-record and re-run.',
            '',
        ]
        best_text = splice(base, observe_block(palette, best_knobs, banner))
        if os.path.exists(args.write):
            shutil.copy(args.write, args.write + '.bak')
        open(args.write, 'w').write(best_text)
        print(f'wrote {args.write}')
        print('re-check it: repose_planner_selftest --replay <reads> '
              f'--config {args.write} --table <clips.npz>')


if __name__ == '__main__':
    main()
