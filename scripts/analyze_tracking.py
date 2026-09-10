#!/usr/bin/env python3
"""Tracking error and transfer metrics from difftrack run logs.

    python3 scripts/analyze_tracking.py recordings/runs/*.npz
    python3 scripts/analyze_tracking.py --compare <sim>.npz <robot>.npz
    python3 scripts/analyze_tracking.py -o out/ recordings/runs/*.npz   # + figures
    python3 scripts/analyze_tracking.py --json recordings/runs/*.npz    # machine-readable

The runs come from the controller itself (`record:=true`, on by default) --
see docs/run_logs.md for the file format and common/run_recorder.hpp for why
they are written in there rather than recovered from a bag.

WHAT IS MEASURED, and why these and not others

A transfer result is ONE policy measured the same way in two plants, so every
number here is computed from columns that mean the same thing in both. The node
writes the same file from the same code in sim2sim and on a robot; nothing below
knows which it is reading.

  tracking      Where the robot was against where the reference said to be.
                E_root is the same quantity the node's SUMMARY line reports, so
                the two are cross-checkable; the rest splits it into the parts
                that fail differently. E_z and E_rot are observable to an
                onboard estimator and E_xy is not, so a run on /odom that looks
                bad in E_xy and fine in the others is an estimator result, not a
                policy one.

  joints        E_jpos is the reference joint angles against the measured ones,
                which is the tracking error the POLICY is actually paid for --
                the root error is partly the plant's. MPJPE is the same thing in
                the world, through the export's own kinematic table: the metric
                motion-tracking papers report, and the one that notices an error
                that stays small in every joint and adds up along a limb.

  execution     What the two plants did NOT share. Torque saturation, control
                period jitter, dropped state messages, motor temperature and the
                PD's own following error are where a sim2real gap actually comes
                from, and they are invisible in any tracking number.

`--compare` pairs two runs by CLIP STEP -- not by time, which drifts, and not by
row, which differs the moment one run misses a control step -- and reports both
runs and their difference on the steps they have in common.
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

PHASE_TRACK = 1


# ══════════════════════════════════════════════════════════════
#  Loading
# ══════════════════════════════════════════════════════════════

class Run:
    """One recorded run: its columns, its metadata, and the tracked window."""

    def __init__(self, path):
        self.path = Path(path)
        z = np.load(self.path)
        self.arrays = {k: z[k] for k in z.files}
        if "meta_json" not in self.arrays:
            raise ValueError(f"{path}: no meta_json — not a difftrack run log")
        self.meta = json.loads(bytes(self.arrays["meta_json"]).decode())
        self.export = None
        if "difftrack_config_json" in self.arrays:
            self.export = json.loads(bytes(self.arrays["difftrack_config_json"]).decode())

        self.name = self.path.stem
        self.tag = self.meta["run"].get("tag", "?")
        self.motion = self.meta["export"].get("motion", "?")
        self.variant = self.meta["export"].get("variant", "")
        self.dt = float(self.meta["run"].get("control_dt", 0.02))
        self.world = self.meta["world"].get("source", "?")
        self.summary = self.meta.get("summary", {})

        self.policy_to_motor = np.asarray(self.meta["joints"]["policy_to_motor"], dtype=int)
        self.policy_joints = self.meta["joints"]["policy_joint_names"]

        # The tracked window: the steps the policy was driving against a clip.
        # Everything else in the file -- the entry ramp, the exit, the rest
        # state, the tail -- is deliberately in there and deliberately not in a
        # tracking mean.
        clip = self["clip_step"]
        phase = self["phase"]
        self.track = (self["policy_tick"] > 0.5) & np.isfinite(clip) & (clip >= 0) \
            & (phase == PHASE_TRACK)
        self.clip_step = clip

    def __getitem__(self, key):
        return self.arrays[key]

    def has(self, key):
        return key in self.arrays

    @property
    def n(self):
        return len(self["t"])

    def policy_order(self, key):
        """A motor-order column re-indexed into POLICY order, so it lines up
        with ref_dof_pos and action. Never assumed to be the identity: it is on
        the G1 and the node resolves it by name anyway, because a permutation is
        the one error here that produces no error at all."""
        return self[key][:, self.policy_to_motor]

    def label(self):
        """Short and unique in a directory of runs: the tag says which plant,
        the wall time says which run. Two files of one session differ only in
        the timestamp, so a label built from the run index alone collides."""
        stamp = self.name.split("_")[0]
        return f"{self.tag}/{stamp.split('-')[-1]}"

    def fingerprint(self):
        """What has to match before two runs can be compared at all."""
        e = self.meta["export"]
        return (e.get("source_run"), e.get("train_run"), e.get("variant"),
                e.get("checkpoint_iter"), e.get("motion"))


# ══════════════════════════════════════════════════════════════
#  Quaternions and forward kinematics (wxyz, mirroring the C++)
# ══════════════════════════════════════════════════════════════

def qmul(a, b):
    aw, ax, ay, az = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bw, bx, by, bz = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([aw * bw - ax * bx - ay * by - az * bz,
                     aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw], axis=-1)


def qrot(q, v):
    """Rotate v by q. Both broadcast on the leading axes."""
    u = q[..., 1:]
    w = q[..., :1]
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def qnorm(q):
    return q / np.clip(np.linalg.norm(q, axis=-1, keepdims=True), 1e-12, None)


def axis_angle(axis, angle):
    """(3,) axis and (T,) angle -> (T, 4) quaternion, wxyz."""
    half = 0.5 * angle
    s = np.sin(half)
    return np.stack([np.cos(half), axis[0] * s, axis[1] * s, axis[2] * s], axis=-1)


def heading_yaw(q):
    """Yaw about +Z, the Z-up convention the clips use — the same formula the
    node's heading_yaw() applies."""
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def geodesic_angle(qa, qb):
    """Angle of the rotation taking qa to qb, radians."""
    d = np.abs(np.sum(qnorm(qa) * qnorm(qb), axis=-1))
    return 2.0 * np.arccos(np.clip(d, -1.0, 1.0))


def forward_kinematics(fk, root_pos, root_quat, dof):
    """World body positions for every step.

    A transcription of DiffTrackObsBuilder::forwardKinematics — one forward
    sweep over a tree of body-origin hinges, parents before children — against
    the SAME table the controller used, which travels inside the run file.

    root_pos (T,3), root_quat (T,4) wxyz, dof (T,nA)  ->  (T, nb, 3)
    """
    parent = fk["parent"]
    body_pos = np.asarray(fk["body_pos"], dtype=float)
    body_quat = np.asarray(fk["body_quat"], dtype=float)
    joint_axis = np.asarray(fk["joint_axis"], dtype=float)
    joint_dof = fk["joint_dof"]
    fixed_offset = np.asarray(fk["fixed_offset"], dtype=float)
    nb = len(parent)
    T = root_pos.shape[0]

    pos = np.zeros((nb, T, 3))
    quat = np.zeros((nb, T, 4))
    pos[0] = root_pos
    quat[0] = qnorm(root_quat)
    for i in range(1, nb):
        p = parent[i]
        pq = quat[p]
        if joint_dof[i] < 0:
            quat[i] = pq
            pos[i] = pos[p] + qrot(pq, np.broadcast_to(fixed_offset[i], (T, 3)))
            continue
        pos[i] = pos[p] + qrot(pq, np.broadcast_to(body_pos[i], (T, 3)))
        local = qmul(np.broadcast_to(body_quat[i], (T, 4)),
                     axis_angle(joint_axis[i], dof[:, joint_dof[i]]))
        quat[i] = qmul(pq, local)
    return np.transpose(pos, (1, 0, 2))


# ══════════════════════════════════════════════════════════════
#  Metrics
# ══════════════════════════════════════════════════════════════

def _mean(x):
    return float(np.nanmean(x)) if np.size(x) else float("nan")


def _max(x):
    return float(np.nanmax(x)) if np.size(x) else float("nan")


def tracking_metrics(run, mask=None):
    """Reference against measured, over the tracked window."""
    m = run.track if mask is None else (run.track & mask)
    out = {"steps": int(m.sum()), "seconds": float(m.sum() * run.dt)}
    if out["steps"] == 0:
        return out

    p = run["root_pos"][m]
    pr = run["ref_root_pos"][m]
    q = run["root_quat"][m]
    qr = run["ref_root_quat"][m]

    err = np.linalg.norm(p - pr, axis=1)
    out["E_root"] = _mean(err)
    out["E_root_max"] = _max(err)
    # Split, because the two halves fail for different reasons and an onboard
    # estimator can only see one of them.
    out["E_xy"] = _mean(np.linalg.norm(p[:, :2] - pr[:, :2], axis=1))
    out["E_z"] = _mean(np.abs(p[:, 2] - pr[:, 2]))
    out["E_rot"] = math.degrees(_mean(geodesic_angle(q, qr)))
    yaw = np.arctan2(np.sin(heading_yaw(q) - heading_yaw(qr)),
                     np.cos(heading_yaw(q) - heading_yaw(qr)))
    out["E_yaw"] = math.degrees(_mean(np.abs(yaw)))
    out["min_height"] = float(np.min(p[:, 2]))
    out["speed"] = _mean(np.linalg.norm(run["root_lin_vel"][m][:, :2], axis=1))

    # Joint space, in POLICY order so it lines up with the reference.
    qm = run.policy_order("q")[m]
    qref = run["ref_dof_pos"][m]
    jerr = np.abs(qm - qref)
    out["E_jpos"] = _mean(jerr)
    out["E_jpos_max"] = _max(jerr)
    per_joint = np.nanmean(jerr, axis=0)
    order = np.argsort(-per_joint)[:3]
    out["worst_joints"] = [(run.policy_joints[i], float(per_joint[i])) for i in order]
    out["per_joint"] = per_joint.tolist()
    for group, needles in (("leg", ("hip", "knee", "ankle")),
                           ("arm", ("shoulder", "elbow", "wrist")),
                           ("waist", ("waist", "torso"))):
        idx = [i for i, n in enumerate(run.policy_joints) if any(k in n for k in needles)]
        if idx:
            out[f"E_jpos_{group}"] = _mean(jerr[:, idx])

    # Body space, through the export's own kinematic table. This is the metric
    # tracking papers report, and the one that catches a per-joint error that is
    # small everywhere and adds up along a limb.
    if run.export is not None:
        fk = run.export["fk"]
        bp = forward_kinematics(fk, p, q, qm)
        br = forward_kinematics(fk, pr, qr, qref)
        d = np.linalg.norm(bp - br, axis=2)
        out["mpjpe"] = _mean(d)
        out["mpjpe_max"] = _max(d)
        # Root-relative: the pose error with the root error taken out, i.e. how
        # wrong the SHAPE was rather than where it was.
        d_local = np.linalg.norm((bp - bp[:, :1]) - (br - br[:, :1]), axis=2)
        out["mpjpe_local"] = _mean(d_local)
        key_ids = run.export["obs"].get("key_body_ids")
        if key_ids:
            out["E_keybody"] = _mean(d[:, key_ids])
    return out


def execution_metrics(run, mask=None):
    """What the plant did about the commands — the half of a transfer gap that
    no tracking number contains."""
    m = run.track if mask is None else (run.track & mask)
    out = {}
    if m.sum() < 2:
        return out

    # Control period, off the steady clock. A wall timer that slipped, a state
    # stream that stalled and a machine that was too slow all land here, and all
    # three change the physics per control step.
    tw = run["t_wall"]
    d = np.diff(tw)
    d = d[np.isfinite(d) & (d > 0)]
    if d.size:
        out["dt_mean"] = _mean(d)
        out["dt_max"] = _max(d)
        out["dt_jitter"] = float(np.std(d))
        out["late_steps"] = int(np.sum(d > 1.5 * run.dt))

    # The plant's own tick. A gap here is a state message that never arrived,
    # which no timing number can distinguish from a slow loop.
    tick = run["state_tick"]
    dtick = np.diff(tick)
    dtick = dtick[np.isfinite(dtick) & (dtick > 0)]
    if dtick.size:
        out["state_tick_step"] = float(np.median(dtick))
        out["state_tick_max"] = float(np.max(dtick))

    # Torque, against the export's own limits. Saturation is where a policy
    # that works in simulation stops working on a robot.
    tau = np.abs(run["tau"][m])
    out["tau_mean"] = _mean(tau)
    out["tau_max"] = _max(tau)
    limit = np.asarray(run.meta["gains"]["torque_limit"], dtype=float)
    if limit.size == len(run.policy_to_motor):
        tau_p = np.abs(run.policy_order("tau")[m])
        sat = tau_p >= 0.95 * limit[None, :]
        out["tau_sat_frac"] = float(np.mean(sat))
        per = np.mean(sat, axis=0)
        j = int(np.argmax(per))
        out["tau_sat_worst"] = (run.policy_joints[j], float(per[j]))

    # The PD loop's own following error: how far the joint was from what it was
    # told, which is a property of the actuator and the gains rather than of the
    # policy. It is routinely much larger on a robot than in a simulator.
    out["E_cmd"] = _mean(np.abs(run["q"][m] - run["cmd_q"][m]))

    # Action smoothness. The reversal rate is the share of steps on which the
    # action changes sign of its own increment: a flip every step is a 25 Hz
    # square wave on the joint target. scripts/compare_action_smoothness.sh is
    # the full treatment; this is the one number, for free.
    a = run["action"][m]
    if a.shape[0] > 2:
        da = np.diff(a, axis=0)
        out["action_rate"] = _mean(np.abs(da)) / run.dt
        rev = np.sign(da[1:]) * np.sign(da[:-1]) < 0
        out["action_reversal"] = float(np.mean(rev))

    # Hardware only, and zero everywhere else. Motors that have climbed 30 C
    # over a session are not the plant the first run of it measured.
    temp = run["temp"]
    if np.nanmax(temp) > 0:
        out["temp_max"] = _max(temp)
        out["temp_rise"] = float(np.nanmax(temp[-1]) - np.nanmax(temp[0]))

    # World-pose health. Only ever non-zero when the pose came from outside the
    # state stream — mocap or an onboard estimator, i.e. hardware.
    age = run["world_age"]
    if np.nanmax(age) > 0:
        out["world_age_max"] = _max(age)
        timeout = float(run.meta["world"].get("timeout", 0.0))
        if timeout > 0:
            out["world_stale_steps"] = int(np.sum(age > timeout))
    out["world_invalid_steps"] = int(np.sum(run["world_valid"][run.track] < 0.5))
    return out


def run_report(run):
    r = {
        "file": run.path.name,
        "tag": run.tag,
        "motion": run.motion,
        "variant": run.variant,
        "world": run.world,
        "host": run.meta["run"].get("host", ""),
        "rows": int(run.n),
        "truncated": bool(run.meta["run"].get("truncated", False)),
        "closed_by": run.meta["run"].get("closed_by", ""),
        "fell": bool(run.summary.get("fell", False)),
        "fell_at": int(run.summary.get("fell_at", -1)),
        "reason": run.summary.get("reason", ""),
        "summary_mean_err": run.summary.get("mean_err"),
    }
    r.update(tracking_metrics(run))
    r.update(execution_metrics(run))
    return r


# ══════════════════════════════════════════════════════════════
#  Printing
# ══════════════════════════════════════════════════════════════

def fmt(v, spec="7.3f"):
    if v is None:
        return "-".rjust(int(spec.split(".")[0]))
    if isinstance(v, float) and not math.isfinite(v):
        return "-".rjust(int(spec.split(".")[0]))
    try:
        return format(v, spec)
    except (TypeError, ValueError):
        return str(v)


def print_table(reports):
    print()
    print("tracking — reference against measured, over the tracked window "
          "(metres, degrees)")
    head = (f"{'run':<38} {'motion':<14} {'steps':>6} {'fell':>5} "
            f"{'E_root':>7} {'max':>7} {'E_xy':>7} {'E_z':>7} {'E_rot':>7} "
            f"{'E_jpos':>7} {'mpjpe':>7} {'local':>7}")
    print(head)
    print("-" * len(head))
    for r in reports:
        fell = str(r["fell_at"]) if r["fell"] else "none"
        print(f"{r['file'][:38]:<38} {r['motion'][:14]:<14} {r.get('steps', 0):>6} "
              f"{fell:>5} {fmt(r.get('E_root'))} {fmt(r.get('E_root_max'))} "
              f"{fmt(r.get('E_xy'))} {fmt(r.get('E_z'))} {fmt(r.get('E_rot'))} "
              f"{fmt(r.get('E_jpos'))} {fmt(r.get('mpjpe'))} {fmt(r.get('mpjpe_local'))}")

    print()
    print("execution — what the plant did about the commands")
    head = (f"{'run':<38} {'world':<10} {'dt_mean':>8} {'dt_max':>8} {'late':>5} "
            f"{'|tau|':>7} {'sat%':>6} {'E_cmd':>7} {'rev%':>6} {'Tmax':>6} {'stale':>6}")
    print(head)
    print("-" * len(head))
    for r in reports:
        sat = r.get("tau_sat_frac")
        rev = r.get("action_reversal")
        print(f"{r['file'][:38]:<38} {r['world'][:10]:<10} "
              f"{fmt(r.get('dt_mean'), '8.4f')} {fmt(r.get('dt_max'), '8.4f')} "
              f"{r.get('late_steps', 0):>5} {fmt(r.get('tau_mean'))} "
              f"{fmt(None if sat is None else 100 * sat, '6.1f')} "
              f"{fmt(r.get('E_cmd'))} "
              f"{fmt(None if rev is None else 100 * rev, '6.1f')} "
              f"{fmt(r.get('temp_max'), '6.1f')} "
              f"{r.get('world_stale_steps', r.get('world_invalid_steps', 0)):>6}")

    truncated = [r["file"] for r in reports if r["truncated"]]
    if truncated:
        print()
        print(f"  {len(truncated)} run(s) hit the buffer bound and are INCOMPLETE — "
              "raise record_max_seconds:")
        for f in truncated:
            print(f"    {f}")


COMPARE_ROWS = [
    ("steps tracked", "steps", "{:.0f}", 1.0),
    ("E_root  (m)", "E_root", "{:.4f}", 1.0),
    ("E_root max (m)", "E_root_max", "{:.4f}", 1.0),
    ("E_xy    (m)", "E_xy", "{:.4f}", 1.0),
    ("E_z     (m)", "E_z", "{:.4f}", 1.0),
    ("E_rot   (deg)", "E_rot", "{:.2f}", 1.0),
    ("E_yaw   (deg)", "E_yaw", "{:.2f}", 1.0),
    ("E_jpos  (rad)", "E_jpos", "{:.4f}", 1.0),
    ("E_jpos legs", "E_jpos_leg", "{:.4f}", 1.0),
    ("E_jpos arms", "E_jpos_arm", "{:.4f}", 1.0),
    ("mpjpe   (m)", "mpjpe", "{:.4f}", 1.0),
    ("mpjpe local (m)", "mpjpe_local", "{:.4f}", 1.0),
    ("min height (m)", "min_height", "{:.3f}", 1.0),
    ("speed   (m/s)", "speed", "{:.3f}", 1.0),
    ("|tau|   (Nm)", "tau_mean", "{:.2f}", 1.0),
    ("tau saturated (%)", "tau_sat_frac", "{:.2f}", 100.0),
    ("E_cmd   (rad)", "E_cmd", "{:.4f}", 1.0),
    ("action rate (1/s)", "action_rate", "{:.3f}", 1.0),
    ("action reversal (%)", "action_reversal", "{:.1f}", 100.0),
    ("dt mean (s)", "dt_mean", "{:.4f}", 1.0),
    ("dt max  (s)", "dt_max", "{:.4f}", 1.0),
    ("late steps", "late_steps", "{:.0f}", 1.0),
]


def print_compare(a, b, ra, rb, common):
    print()
    print(f"compare — {len(common)} clip steps in common "
          f"({len(common) * a.dt:.1f}s), paired by clip step")
    print(f"  A  {a.path.name}   tag={a.tag} world={a.world} host={a.meta['run'].get('host','')}")
    print(f"  B  {b.path.name}   tag={b.tag} world={b.world} host={b.meta['run'].get('host','')}")
    print()
    head = f"{'metric':<22} {'A':>12} {'B':>12} {'B - A':>12} {'B / A':>8}"
    print(head)
    print("-" * len(head))
    for label, key, spec, scale in COMPARE_ROWS:
        va, vb = ra.get(key), rb.get(key)
        if va is None or vb is None:
            continue
        va, vb = float(va) * scale, float(vb) * scale
        if not (math.isfinite(va) and math.isfinite(vb)):
            continue
        ratio = vb / va if abs(va) > 1e-12 else float("nan")
        print(f"{label:<22} {spec.format(va):>12} {spec.format(vb):>12} "
              f"{spec.format(vb - va):>12} "
              f"{('%8.2f' % ratio) if math.isfinite(ratio) else '       -'}")


# ══════════════════════════════════════════════════════════════
#  Pairing two runs
# ══════════════════════════════════════════════════════════════

def pair_on_clip_step(a, b):
    """Masks selecting the clip steps both runs tracked.

    By clip step, not by time and not by row: two runs of one clip agree on
    what step 137 is, and on nothing else. Time drifts between the plants and a
    row index shifts the moment either run misses a control step.
    """
    sa = a.clip_step.copy()
    sb = b.clip_step.copy()
    sa[~a.track] = np.nan
    sb[~b.track] = np.nan
    common = np.intersect1d(sa[np.isfinite(sa)].astype(int),
                            sb[np.isfinite(sb)].astype(int))
    ma = a.track & np.isin(np.nan_to_num(sa, nan=-1).astype(int), common)
    mb = b.track & np.isin(np.nan_to_num(sb, nan=-1).astype(int), common)
    return ma, mb, common


def obs_shift(a, b, ma, mb):
    """How far apart the policy's own INPUT was between the two runs, per
    channel, on the steps they share. The observation is the whole of what the
    policy sees, so this is the transfer gap in the only space that matters to
    it -- and it is the character block, not the reference block, that a robot
    changes."""
    if not (a.has("obs") and b.has("obs")):
        return None
    oa, ob = a["obs"][ma], b["obs"][mb]
    n = min(oa.shape[0], ob.shape[0])
    if n == 0 or oa.shape[1] != ob.shape[1]:
        return None
    d = np.abs(oa[:n] - ob[:n])
    char = int(a.meta["export"].get("char_obs_dim", 0))
    out = {"obs_mean_abs_diff": _mean(d)}
    if 0 < char <= d.shape[1]:
        out["obs_char_diff"] = _mean(d[:, :char])
        out["obs_tar_diff"] = _mean(d[:, char:])
    return out


# ══════════════════════════════════════════════════════════════
#  Figures
# ══════════════════════════════════════════════════════════════

def figures(runs, outdir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("  matplotlib not available — skipping figures", file=sys.stderr)
        return []

    outdir.mkdir(parents=True, exist_ok=True)
    written = []
    fig, ax = plt.subplots(2, 3, figsize=(16, 8))

    for run in runs:
        m = run.track
        if m.sum() == 0:
            continue
        label = run.label()
        t = run.clip_step[m] * run.dt
        p, pr = run["root_pos"][m], run["ref_root_pos"][m]

        ax[0][0].plot(t, np.linalg.norm(p - pr, axis=1), lw=1, label=label)
        ax[0][1].plot(t, p[:, 2], lw=1, label=label)
        ax[0][1].plot(t, pr[:, 2], lw=0.8, ls="--", color="k", alpha=0.4)
        ax[0][2].plot(p[:, 0], p[:, 1], lw=1, label=label)
        ax[0][2].plot(pr[:, 0], pr[:, 1], lw=0.8, ls="--", color="k", alpha=0.4)

        jerr = np.abs(run.policy_order("q")[m] - run["ref_dof_pos"][m])
        ax[1][0].plot(np.nanmean(jerr, axis=0), lw=1, marker=".", ms=3, label=label)
        ax[1][1].plot(t, np.max(np.abs(run["tau"][m]), axis=1), lw=1, label=label)
        d = np.diff(run["t_wall"])
        d = d[np.isfinite(d) & (d > 0)]
        if d.size:
            ax[1][2].hist(d * 1000, bins=60, alpha=0.5, label=label)

    ax[0][0].set(title="root tracking error", xlabel="clip time (s)", ylabel="m")
    ax[0][1].set(title="root height (dashed = reference)", xlabel="clip time (s)", ylabel="m")
    ax[0][2].set(title="path in the world (dashed = reference)", xlabel="x (m)", ylabel="y (m)")
    ax[0][2].axis("equal")
    ax[1][0].set(title="mean |joint error| per joint (policy order)",
                 xlabel="policy joint index", ylabel="rad")
    ax[1][1].set(title="peak |torque| across joints", xlabel="clip time (s)", ylabel="Nm")
    ax[1][2].set(title="control period", xlabel="ms", ylabel="steps")
    for row in ax:
        for a in row:
            a.grid(alpha=0.3)
            a.legend(fontsize=7)
    fig.tight_layout()
    path = outdir / "tracking.png"
    fig.savefig(path, dpi=130)
    plt.close(fig)
    written.append(path)
    return written


# ══════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(
        description="tracking error and transfer metrics from difftrack run logs",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("runs", nargs="+", help="run npz files (recordings/runs/*.npz)")
    ap.add_argument("--compare", action="store_true",
                    help="pair the FIRST TWO runs by clip step and report the "
                         "difference — the transfer result")
    ap.add_argument("-o", "--out", type=Path, help="write figures here")
    ap.add_argument("--json", action="store_true", help="dump every metric as JSON")
    ap.add_argument("--csv", type=Path, help="write the per-run table as csv")
    args = ap.parse_args()

    runs = []
    for path in args.runs:
        try:
            runs.append(Run(path))
        except Exception as exc:
            print(f"  skipping {path}: {exc}", file=sys.stderr)
    if not runs:
        print("no readable run logs", file=sys.stderr)
        return 1
    runs.sort(key=lambda r: r.name)

    reports = [run_report(r) for r in runs]

    if args.json:
        print(json.dumps(reports, indent=2, default=float))
    else:
        print_table(reports)

    if args.csv:
        keys = sorted({k for r in reports for k in r
                       if not isinstance(r[k], (list, tuple))})
        with args.csv.open("w") as f:
            f.write(",".join(keys) + "\n")
            for r in reports:
                f.write(",".join(str(r.get(k, "")) for k in keys) + "\n")
        print(f"\ncsv: {args.csv}")

    if args.compare:
        if len(runs) < 2:
            print("--compare needs two runs", file=sys.stderr)
            return 2
        a, b = runs[0], runs[1]
        if a.fingerprint() != b.fingerprint():
            # Not fatal — comparing two policies on one clip is a real question
            # — but it is a different question from a transfer measurement, and
            # nothing downstream would say so.
            print("\n  WARNING: these runs are not the same export.")
            print(f"    A {a.fingerprint()}")
            print(f"    B {b.fingerprint()}")
            print("    A difference below is a policy difference, not a transfer one.")
        ma, mb, common = pair_on_clip_step(a, b)
        if len(common) == 0:
            print("\n  no clip steps in common — nothing to pair", file=sys.stderr)
            return 2
        ra = {**tracking_metrics(a, ma), **execution_metrics(a, ma)}
        rb = {**tracking_metrics(b, mb), **execution_metrics(b, mb)}
        print_compare(a, b, ra, rb, common)
        shift = obs_shift(a, b, ma, mb)
        if shift:
            print()
            print("observation — mean |A - B| per channel on the shared steps")
            for k, v in shift.items():
                print(f"  {k:<22} {v:.4f}")
        elif not (a.has("obs") and b.has("obs")):
            print("\n  (one of these runs has no observation logged — "
                  "record_obs:=true to compare the policy's own input)")

    if args.out:
        for path in figures(runs, args.out):
            print(f"figure: {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
