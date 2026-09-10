#!/usr/bin/env python3
"""Compare how SMOOTH the actions of one or more difftrack policies are.

    python3 scripts/analyze_action_smoothness.py run_a.npz run_b.npz --out DIR

Reads what scripts/record_policy_actions.py captured off /lowcmd during a
sim2sim run, recovers the policy's actions, and draws the five figures that
answer "is this policy jittery, where, and by how much".

FIRST-ORDER SMOOTHNESS, DEFINED

The policy emits one action vector a[t] every control step (20 ms). Everything
here is about the SEQUENCE:

    rate      d[t]  = a[t] - a[t-1]            the first difference
    curvature dd[t] = a[t] - 2a[t-1] + a[t-2]  the second

`rate` is the quantity an action-rate regulariser penalises, and it is the one
that decides whether a robot buzzes: the joint target moves by
action_scale * d[t] every 20 ms whatever the reference asks for, and the motor
answers that with kp * action_scale * d[t] newton-metres of torque step.

A LARGE RATE IS NOT THE SAME AS JITTER, and the two are separated here rather
than conflated:

    a fast clip     large |d|, consistently SIGNED, energy at the gait
                    frequency. The motion asked for it.
    jitter          large |d| that REVERSES every step, energy near the 25 Hz
                    Nyquist. Nothing asked for it.

So every rate number is reported three ways: against the other policy, against
the frequency it lives at, and against the same number computed on the policy's
OWN reference clip (`ref_dof` in the recording, straight out of motion.bin).
The clip's rate is the floor -- the motion the policy was told to produce, at
the same 50 Hz -- and the ratio to it is what "excess" means below.

THE NUMBERS

    rate_rms        RMS of d over all steps and joints, in ACTION units
    target_rate     the same in rad/s at the joint target
                    (rate_rms * action_scale / dt) -- the readable one
    reversal        fraction of steps on which d changes sign. 0 is a ramp;
                    0.5 is a random walk; 1.0 is pure step-to-step chatter,
                    i.e. a 25 Hz square wave the plant has to absorb
    hf_fraction     share of the action's AC power above --hf-cut Hz
    excess          rate_rms / rate_rms(reference clip)
    rate_penalty    mean over steps of ||d||^2 -- literally the action-rate
                    reward term, and therefore the number to divide when you
                    change its weight

TUNING FROM THIS

rate_penalty is the term; its WEIGHT is what you set. If two checkpoints differ
by a factor of k in rate_penalty and you want the jittery one to land where the
smooth one is, k is the first thing to try on its weight -- a starting point,
not a law, because the penalty is not linear in the weight and the clips differ.
Read it next to `excess` and `reversal`: a policy with a high rate_penalty and a
LOW reversal rate is tracking a busy clip and does not want more regularisation.
"""

import argparse
import json
import os
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.gridspec import GridSpec, GridSpecFromSubplotSpec
from matplotlib.lines import Line2D


# ── palette ───────────────────────────────────────────────────────────────────
# Categorical slots in their fixed order -- never cycled, never reassigned by
# rank, so a policy keeps its hue across every figure in the set.
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100",
          "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#8a8984"
GRID = "#e6e5e0"
RED = "#e34948"
# Sequential, one hue light -> dark. Magnitude only.
SEQ = LinearSegmentedColormap.from_list(
    "blues", ["#f4f8fe", "#cde2fb", "#9ec5f4", "#6da7ec",
              "#3987e5", "#256abf", "#184f95", "#0d366b"])

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "font.family": "DejaVu Sans", "font.size": 9,
    "text.color": INK, "axes.labelcolor": INK2, "axes.titlecolor": INK,
    "xtick.color": INK2, "ytick.color": INK2,
    "axes.edgecolor": GRID, "axes.linewidth": 0.8,
    "grid.color": GRID, "grid.linewidth": 0.7, "grid.linestyle": "-",
    "xtick.major.size": 3, "ytick.major.size": 3,
    "legend.frameon": False, "axes.spines.top": False, "axes.spines.right": False,
})


# ── joint bookkeeping ─────────────────────────────────────────────────────────
GROUPS = ["legs", "waist", "arms"]


def short_name(n):
    n = n.replace("_joint", "").replace("left_", "L ").replace("right_", "R ")
    return n.replace("_", " ")


def group_of(n):
    if "waist" in n:
        return "waist"
    if any(k in n for k in ("hip", "knee", "ankle")):
        return "legs"
    return "arms"


# ── loading ───────────────────────────────────────────────────────────────────
def longest_true(mask):
    """[lo, hi) of the longest run of True.

    Contiguity matters. An engage, a fall back to the hold and a second engage
    are two episodes, and differencing across the seam invents a step that
    nothing ever commanded."""
    best, i = (0, 0), 0
    while i < len(mask):
        if mask[i]:
            j = i
            while j < len(mask) and mask[j]:
                j += 1
            if j - i > best[1] - best[0]:
                best = (i, j)
            i = j
        else:
            i += 1
    return best


def read_runlog(npz_path):
    """The sim2sim result line for this run, if the driver left one beside the
    npz. A policy that FELL is not a smoothness result, and the figures say so
    rather than quietly plotting a heap on the floor beside a healthy run."""
    log = os.path.splitext(npz_path)[0] + ".runlog"
    if not os.path.exists(log):
        return {}
    out = {}
    for line in open(log, errors="ignore"):
        f = line.split()
        if len(f) >= 7 and f[1].isdigit() and "/" in f[2]:
            out = dict(steps=f[2], mean_err=f[3], max_err=f[4],
                       min_h=f[5], fell_at=f[6])
    return out


class Run:
    """One recorded sim2sim run, reduced to the policy's own actions."""

    def __init__(self, path, color):
        d = np.load(path, allow_pickle=True)
        m = json.loads(str(d["meta"]))
        self.path, self.color, self.meta = path, color, m
        self.label = m.get("label") or m.get("motion") or os.path.basename(path)
        self.dt = float(m["control_dt"])
        self.scale = float(m["action_scale"])
        self.names = list(m["policy_joint_names"])
        self.groups = [group_of(n) for n in self.names]
        self.short = [short_name(n) for n in self.names]
        self.kp = np.asarray(m["joint_stiffness"], dtype=np.float64)
        p2m = np.asarray(m["policy_to_motor"], dtype=int)
        default = np.asarray(m["default_angles"], dtype=np.float64)

        q = d["q_cmd"][:, p2m].astype(np.float64)
        kp = d["kp"][:, p2m].astype(np.float64)
        t = d["t_cmd"].astype(np.float64)

        # THE POLICY'S OWN SAMPLES, and nothing else. Every hold mode publishes
        # a LowCmd whose q is a joint target too; only the policy drives through
        # the checkpoint's trained stiffness. The test is exact rather than
        # thresholded because both sides are the same float32s out of the
        # export's difftrack_config.json.
        drives = np.all(np.isclose(kp, self.kp[None, :], rtol=1e-4, atol=1e-4), axis=1)
        lo, hi = longest_true(drives)
        if hi - lo < 8:
            raise SystemExit(f"{path}: only {hi - lo} samples carried the policy's gains "
                             "— the run never really tracked anything.")
        self.a = (q[lo:hi] - default[None, :]) / self.scale
        self.t = t[lo:hi] - t[lo]
        self.n = self.a.shape[0]

        # A DROPPED command would double the next first difference and read as a
        # jitter spike that never happened. The timestamps say where one
        # happened; those differences are excluded rather than smoothed over.
        self.ok = np.diff(self.t) <= 1.5 * self.dt
        self.gaps = int((~self.ok).sum())

        # The clip's own joint angles, expressed as the action that would
        # produce them: the floor every rate number is read against.
        self.a_ref = None
        if "ref_dof" in d:
            ref = d["ref_dof"].astype(np.float64)
            K = int(m["clip_steps"])
            step = np.arange(self.n)
            idx = step % K if m.get("loop_wrap") else np.clip(step, 0, K - 1)
            self.a_ref = (ref[idx] - default[None, :]) / self.scale

        self.summary = read_runlog(path)
        self.rate = np.diff(self.a, axis=0)
        self.curv = np.diff(self.a, n=2, axis=0)


# ── metrics ───────────────────────────────────────────────────────────────────
def rms(x, axis=None):
    return np.sqrt(np.mean(np.square(x), axis=axis))


def reversal_rate(rate, ok):
    """Fraction of consecutive steps on which the action rate changes sign.

    The jitter measure a fast clip cannot fake: moving a joint quickly holds one
    sign for many steps, while chattering flips every step -- and a flip every
    step is a 25 Hz square wave on the joint target."""
    s = np.sign(rate)
    pair = ok[:-1] & ok[1:]
    if pair.sum() == 0:
        return np.zeros(rate.shape[1])
    flip = (s[:-1] * s[1:]) < 0
    return (flip & pair[:, None]).sum(axis=0) / pair.sum()


def welch_psd(x, dt, nperseg=256):
    """One-sided power spectral density, Hann windows, 50% overlap.

    Written out rather than imported so this script needs numpy and matplotlib
    and nothing else."""
    nperseg = min(nperseg, len(x))
    step = max(1, nperseg // 2)
    win = np.hanning(nperseg)
    norm = (win ** 2).sum() / dt
    segs = []
    for s in range(0, len(x) - nperseg + 1, step):
        seg = x[s:s + nperseg]
        segs.append(np.abs(np.fft.rfft((seg - seg.mean()) * win)) ** 2 / norm)
    if not segs:
        return np.array([0.0]), np.array([0.0])
    p = np.mean(segs, axis=0)
    p[1:-1] *= 2.0
    return np.fft.rfftfreq(nperseg, dt), p


def hf_fraction(a, dt, cut):
    """Share of the action's AC power above `cut` Hz, averaged over joints."""
    out = []
    for j in range(a.shape[1]):
        f, p = welch_psd(a[:, j], dt)
        tot = p[1:].sum()
        out.append(p[1:][f[1:] > cut].sum() / tot if tot > 0 else 0.0)
    return float(np.mean(out))


def measure(run, hf_cut):
    """Everything the figures and the tables need, per joint and aggregate."""
    r, ok, c = run.rate, run.ok, run.curv
    ok2 = ok[:-1] & ok[1:]
    m = {}
    m["rate_rms_j"] = rms(r[ok], axis=0)
    m["curv_rms_j"] = rms(c[ok2], axis=0)
    m["rev_j"] = reversal_rate(r, ok)
    m["a_rms_j"] = rms(run.a, axis=0)
    # rad/s at the joint target, and N·m per control step at the motor: the two
    # units this actually costs something in.
    m["target_rate_j"] = m["rate_rms_j"] * run.scale / run.dt
    m["torque_step_j"] = m["rate_rms_j"] * run.scale * run.kp

    m["rate_rms"] = float(rms(r[ok]))
    m["curv_rms"] = float(rms(c[ok2]))
    m["rev"] = float(np.mean(m["rev_j"]))
    m["target_rate"] = float(m["rate_rms"] * run.scale / run.dt)
    m["hf_fraction"] = hf_fraction(run.a, run.dt, hf_cut)
    # The reward terms themselves: mean over steps of a squared norm.
    m["rate_penalty"] = float(np.mean(np.sum(r[ok] ** 2, axis=1)))
    m["curv_penalty"] = float(np.mean(np.sum(c[ok2] ** 2, axis=1)))
    m["action_penalty"] = float(np.mean(np.sum(run.a ** 2, axis=1)))
    m["excess"] = float("nan")

    if run.a_ref is not None:
        rr = np.diff(run.a_ref, axis=0)
        good = np.ones(len(rr), bool)
        m["ref_rate_rms_j"] = rms(rr, axis=0)
        m["ref_rate_rms"] = float(rms(rr))
        m["ref_rev_j"] = reversal_rate(rr, good)
        m["ref_hf"] = hf_fraction(run.a_ref, run.dt, hf_cut)
        with np.errstate(divide="ignore", invalid="ignore"):
            m["excess_j"] = np.where(m["ref_rate_rms_j"] > 1e-9,
                                     m["rate_rms_j"] / m["ref_rate_rms_j"], np.nan)
        if m["ref_rate_rms"] > 0:
            m["excess"] = m["rate_rms"] / m["ref_rate_rms"]
    return m


# ── figures ───────────────────────────────────────────────────────────────────
def tidy(ax, grid="y"):
    ax.grid(True, axis=grid, alpha=0.9, zorder=0)
    ax.set_axisbelow(True)
    ax.tick_params(length=3, width=0.8)


def group_blocks(run):
    """Joint indices ordered legs -> waist -> arms, and where each block sits."""
    idx, blocks, at = [], [], 0
    for g in GROUPS:
        members = [j for j in range(len(run.names)) if run.groups[j] == g]
        if not members:
            continue
        idx += members
        blocks.append((g, at, at + len(members)))
        at += len(members)
    return np.asarray(idx, dtype=int), blocks


def fig_scorecard(runs, mets, out, hf_cut):
    """The headline: what each policy costs, and on which joints."""
    idx, blocks = group_blocks(runs[0])
    labels = [runs[0].short[j] for j in idx]
    nj, nr = len(idx), len(runs)
    h = 0.78 / nr
    band = 0.75 + 0.55 * nr
    height = band + 0.30 * nj

    fig = plt.figure(figsize=(13.2, height))
    gs = GridSpec(2, 2, figure=fig, height_ratios=[band, 0.30 * nj],
                  hspace=0.055, wspace=0.24,
                  left=0.128, right=0.985, top=1 - 0.42 / height, bottom=0.055)

    # ---- the numbers, as a band rather than as more bars ----
    head = fig.add_subplot(gs[0, :])
    head.axis("off")
    cols = [("target rate\nrad/s RMS", 0.245), ("reversals\n% of steps", 0.385),
            ("power > %g Hz\n%% of AC power" % hf_cut, 0.520),
            ("vs its own clip\nrate ratio", 0.655),
            ("rate penalty\nmean ‖Δa‖²", 0.790)]
    for c, x in cols:
        head.text(x, 0.93, c, ha="center", va="top", fontsize=7.6, color=MUTED,
                  linespacing=1.4)
    for k, (run, m) in enumerate(zip(runs, mets)):
        y = 0.42 - k * 0.42
        head.plot([0.012], [y], marker="s", ms=7, color=run.color, clip_on=False)
        head.text(0.030, y + 0.055, run.label, fontsize=10.5, color=INK,
                  va="center", weight="bold")
        fell = run.summary.get("fell_at")
        if fell and fell != "none":
            head.text(0.030, y - 0.10, f"FELL at step {fell} — not a smoothness result",
                      fontsize=7.4, color=RED, va="center")
        else:
            err = run.summary.get("mean_err")
            head.text(0.030, y - 0.10,
                      f"{run.n} steps" + (f" · root err {err} m · no fall" if err else ""),
                      fontsize=7.4, color=MUTED, va="center")
        vals = [f"{m['target_rate']:.2f}", f"{100 * m['rev']:.0f}",
                f"{100 * m['hf_fraction']:.0f}", f"{m['excess']:.1f}×",
                f"{m['rate_penalty']:.2f}"]
        for (_, x), v in zip(cols, vals):
            head.text(x, y, v, ha="center", va="center", fontsize=15,
                      color=run.color, weight="bold")
    # The swatch beside each name IS the legend; the only thing left to explain
    # is the tick mark, so it is explained once here instead of twice in the
    # panels, where it collided with the bars.
    head.plot([0.012], [0.42 - nr * 0.42 + 0.06], marker="|", ms=8, mew=1.5,
              color=INK2, clip_on=False)
    head.text(0.030, 0.42 - nr * 0.42 + 0.06,
              "the same quantity computed on that policy's own reference clip — "
              "the floor a perfect tracker would sit at",
              fontsize=7.8, color=MUTED, va="center")
    head.set_xlim(0, 1)
    head.set_ylim(0.42 - nr * 0.42 - 0.18, 1.0)

    # ---- per joint ----
    for c, (title, key) in enumerate([
            ("target rate  ·  rad/s RMS of the commanded joint target", "target_rate_j"),
            ("reversal rate  ·  share of steps where the action rate flips sign"
             "  ·  the rule at 0.5 is a random walk", "rev_j")]):
        ax = fig.add_subplot(gs[1, c])
        y = np.arange(nj)
        for k, (run, m) in enumerate(zip(runs, mets)):
            off = (k - (nr - 1) / 2) * h
            ax.barh(y + off, m[key][idx], height=h * 0.90, color=run.color,
                    edgecolor=SURFACE, linewidth=1.2, zorder=3)
        # The clip's own value as a tick per joint: a floor, not a series.
        refkey = "ref_rate_rms_j" if key == "target_rate_j" else "ref_rev_j"
        for k, (run, m) in enumerate(zip(runs, mets)):
            if refkey not in m:
                continue
            off = (k - (nr - 1) / 2) * h
            v = m[refkey][idx]
            if key == "target_rate_j":
                v = v * run.scale / run.dt
            ax.plot(v, y + off, "|", ms=8, mew=1.5, color=INK2, zorder=4)
        ax.set_yticks(y)
        ax.set_yticklabels(labels, fontsize=7.4)
        ax.set_ylim(nj - 0.5, -0.5)
        ax.set_title(title, fontsize=9.5, pad=8, loc="left")
        tidy(ax, grid="x")
        for _, _, end in blocks[:-1]:
            ax.axhline(end - 0.5, color=GRID, lw=1.6)
        # Group names live in the margin, rotated, spanning their block: a
        # label sitting on the first tick of each block covers a joint name.
        for g, start, end in blocks:
            ax.text(-0.215, (start + end - 1) / 2, g, transform=ax.get_yaxis_transform(),
                    fontsize=8, color=MUTED, ha="center", va="center", rotation=90,
                    style="italic")
        if key == "rev_j":
            ax.set_xlim(0, max(0.62, ax.get_xlim()[1]))
            ax.axvline(0.5, color=MUTED, lw=0.9, zorder=2)

    fig.suptitle("Action smoothness, per joint", x=0.012, y=1 - 0.10 / height,
                 ha="left", fontsize=13.5, weight="bold")
    fig.savefig(out, dpi=170)
    plt.close(fig)


def fig_traces(runs, mets, out, joints, zoom):
    """The eyeball plot: the action itself, and its first difference under it."""
    nr, nj = len(runs), len(joints)
    height = 2.55 * nj + 1.05
    # A floor on the width, and margins stated in INCHES: with one run the
    # fractional margins that suit a two-column figure clip the y labels off.
    width = max(9.0, 6.3 * nr)
    fig = plt.figure(figsize=(width, height))
    outer = GridSpec(nj, 1, figure=fig, hspace=0.44, left=0.95 / width,
                     right=1 - 0.16 / width, top=1 - 0.78 / height,
                     bottom=0.62 / height)

    for r, j in enumerate(joints):
        inner = GridSpecFromSubplotSpec(2, nr, subplot_spec=outer[r],
                                        height_ratios=[2.1, 1.0], hspace=0.10,
                                        wspace=0.17)
        for c, (run, m) in enumerate(zip(runs, mets)):
            lo = max(0, int(round(zoom[0] / run.dt)))
            hi = min(run.n, int(round(zoom[1] / run.dt)))
            top = fig.add_subplot(inner[0, c])
            bot = fig.add_subplot(inner[1, c], sharex=top)
            t = run.t[lo:hi]

            if run.a_ref is not None:
                top.plot(t, run.a_ref[lo:hi, j], color=MUTED, lw=1.1, zorder=2,
                         label="clip reference")
            top.plot(t, run.a[lo:hi, j], color=run.color, lw=1.6, zorder=3,
                     label="policy action")
            top.set_title(f"{run.label}  ·  {run.short[j]}", fontsize=9.5,
                          loc="left", pad=6)
            tidy(top)
            top.tick_params(labelbottom=False)
            if c == 0:
                top.set_ylabel("action", fontsize=8.5)

            dhi = max(lo, hi - 1)
            d, td = run.rate[lo:dhi, j], run.t[lo:dhi]
            bot.axhline(0, color=GRID, lw=1.0, zorder=1)
            bot.fill_between(td, 0, d, color=run.color, alpha=0.22, lw=0, zorder=2)
            bot.plot(td, d, color=run.color, lw=1.1, zorder=3)
            # Headroom at the top for the per-panel numbers: a jittery joint
            # fills its band edge to edge and the label lands on the trace.
            lim = float(np.max(np.abs(d))) if len(d) else 1.0
            bot.set_ylim(-1.08 * lim, 1.52 * lim)
            tidy(bot)
            if c == 0:
                bot.set_ylabel("Δ action\nper step", fontsize=8.5)
            bot.set_xlabel("seconds into the clip", fontsize=8.5)
            bot.annotate(f"RMS {m['rate_rms_j'][j]:.3f}  ·  reversals "
                         f"{100 * m['rev_j'][j]:.0f}%",
                         xy=(0.99, 0.93), xycoords="axes fraction", ha="right",
                         va="top", fontsize=7.6, color=INK2)

    # One legend for the whole figure, in the header: inside an axes it lands
    # on the trace, and the trace is the thing this figure is for.
    handles = [Line2D([], [], color=MUTED, lw=1.4, label="clip reference")]
    handles += [Line2D([], [], color=r.color, lw=1.8, label=f"{r.label} action")
                for r in runs]
    fig.legend(handles=handles, loc="upper right",
               bbox_to_anchor=(1 - 0.16 / width, 1 - 0.10 / height),
               fontsize=8.5, ncol=len(handles))
    fig.suptitle(f"What the action does, step by step   ·   "
                 f"{zoom[0]:.1f}–{zoom[1]:.1f} s of each run",
                 x=0.012, y=1 - 0.10 / height, ha="left", fontsize=13.5, weight="bold")
    fig.text(0.012, 1 - 0.40 / height,
             "A smooth policy's Δ-action panel is a slow wave. A jittery one fills its "
             "panel with a band that changes sign every step.",
             ha="left", va="top", fontsize=8.6, color=MUTED)
    fig.savefig(out, dpi=170)
    plt.close(fig)


def fig_spectrum(runs, mets, out, hf_cut):
    """Where the action's energy actually lives. Jitter is the tail."""
    fig, axes = plt.subplots(2, len(GROUPS), figsize=(13.2, 6.8),
                             gridspec_kw=dict(hspace=0.32, wspace=0.23,
                                              left=0.068, right=0.988,
                                              top=0.815, bottom=0.085))
    for c, g in enumerate(GROUPS):
        ax, cum = axes[0, c], axes[1, c]
        for run in runs:
            members = [j for j in range(len(run.names)) if run.groups[j] == g]
            if not members:
                continue
            f, _ = welch_psd(run.a[:, members[0]], run.dt)
            p = np.mean([welch_psd(run.a[:, j], run.dt)[1] for j in members], axis=0)
            ax.semilogy(f[1:], p[1:], color=run.color, lw=1.6, label=run.label, zorder=3)
            if run.a_ref is not None:
                pr = np.mean([welch_psd(run.a_ref[:, j], run.dt)[1] for j in members],
                             axis=0)
                ax.semilogy(f[1:], pr[1:], color=run.color, lw=1.0, alpha=0.42, zorder=2,
                            label=f"{run.label} — its clip")
            cum.plot(f[1:], 100 * np.cumsum(p[1:]) / p[1:].sum(), color=run.color,
                     lw=1.6, zorder=3)
        for a in (ax, cum):
            a.axvline(hf_cut, color=MUTED, lw=0.9, zorder=1)
            tidy(a, grid="both")
            a.set_xlim(0, 25)
        ax.set_title(g, fontsize=10, loc="left", pad=6)
        cum.set_xlabel("Hz", fontsize=8.5)
        cum.set_ylim(0, 100)
        cum.annotate(f"{hf_cut:g} Hz", xy=(hf_cut, 0.06), xycoords=("data", "axes fraction"),
                     xytext=(4, 0), textcoords="offset points", fontsize=7.4, color=MUTED)
        if c == 0:
            ax.set_ylabel("action power\n(per Hz, log)", fontsize=8.5)
            cum.set_ylabel("cumulative share\nof AC power (%)", fontsize=8.5)
            ax.legend(loc="lower left", fontsize=7.8)

    fig.suptitle("Action spectrum, averaged over the joints of each group",
                 x=0.012, y=0.985, ha="left", fontsize=13.5, weight="bold")
    fig.text(0.012, 0.925,
             "Thick line: the policy. Thin line of the same colour: the clip it was tracking, "
             "at the same 50 Hz. The clip is the floor — energy the thick\nline has and the "
             "thin one does not is the policy's own. Past roughly 10 Hz nothing in the motion "
             "asks for it; 25 Hz is one sign flip per control step.",
             ha="left", va="top", fontsize=8.6, color=MUTED, linespacing=1.5)
    fig.savefig(out, dpi=170)
    plt.close(fig)


def fig_heatmap(runs, out):
    """When and where. A constant chatter and a per-footstep burst are both
    'high RMS' and want different fixes."""
    nr = len(runs)
    height = 1.05 + 3.6 * nr
    left, right, top, bottom = 0.108, 0.895, 1 - 0.78 / height, 0.62 / height
    fig, axes = plt.subplots(nr, 1, figsize=(13.2, height), squeeze=False,
                             gridspec_kw=dict(hspace=0.32, left=left, right=right,
                                              top=top, bottom=bottom))
    vmax = max(float(np.percentile(np.abs(r.rate), 99)) for r in runs)
    for k, run in enumerate(runs):
        idx, blocks = group_blocks(run)
        ax = axes[k, 0]
        im = ax.imshow(np.abs(run.rate[:, idx]).T, aspect="auto", origin="upper",
                       cmap=SEQ, vmin=0, vmax=vmax, interpolation="nearest",
                       extent=[run.t[0], run.t[-1], len(idx) - 0.5, -0.5])
        ax.set_yticks(np.arange(len(idx)))
        ax.set_yticklabels([run.short[j] for j in idx], fontsize=6.6)
        ax.set_title(run.label, fontsize=10.5, loc="left", pad=6, color=run.color,
                     weight="bold")
        ax.set_xlabel("seconds into the clip", fontsize=8.5)
        ax.grid(False)
        for _, _, end in blocks[:-1]:
            ax.axhline(end - 0.5, color=SURFACE, lw=1.8)
    # ONE bar for one scale. A colourbar per panel invites the reading that each
    # panel has its own, which is the opposite of the point.
    cax = fig.add_axes([right + 0.012, bottom, 0.011, top - bottom])
    cb = fig.colorbar(im, cax=cax)
    cb.set_label("|Δ action| per step", fontsize=8)
    cb.outline.set_visible(False)
    cb.ax.tick_params(length=2, labelsize=7.5)

    fig.suptitle("Where the jitter is, and when", x=0.012, y=1 - 0.10 / height,
                 ha="left", fontsize=13.5, weight="bold")
    fig.text(0.012, 1 - 0.40 / height,
             "One shared colour scale, so the panels are directly comparable. Vertical "
             "stripes are contact events; a solid band is a joint that never settles.",
             ha="left", va="top", fontsize=8.6, color=MUTED)
    fig.savefig(out, dpi=170)
    plt.close(fig)


def fig_penalty(runs, mets, out):
    """The reward term itself, over the run and in aggregate."""
    fig = plt.figure(figsize=(13.2, 5.6))
    gs = GridSpec(1, 2, figure=fig, width_ratios=[1.8, 1], wspace=0.22,
                  left=0.068, right=0.982, top=0.775, bottom=0.105)

    ax = fig.add_subplot(gs[0, 0])
    for run, m in zip(runs, mets):
        ax.plot(run.t[1:], np.sum(run.rate ** 2, axis=1), color=run.color, lw=1.0,
                alpha=0.75, zorder=3, label=run.label)
        ax.axhline(m["rate_penalty"], color=run.color, lw=1.4, zorder=4)
        # Inside the axes, not past its right edge: outside, it lands on the
        # neighbouring panel's tick labels.
        ax.annotate(f"mean {m['rate_penalty']:.2f} ", xy=(0.997, m["rate_penalty"]),
                    xycoords=("axes fraction", "data"), color=run.color,
                    fontsize=8.4, ha="right", va="bottom")
    ax.set_yscale("log")
    ax.set_xlabel("seconds into the clip", fontsize=8.5)
    ax.set_ylabel("‖a[t] − a[t−1]‖²   over all 29 joints", fontsize=8.5)
    ax.set_title("the action-rate penalty, step by step", fontsize=10, loc="left", pad=6)
    ax.legend(loc="upper right", fontsize=8.4)
    tidy(ax, grid="both")

    bx = fig.add_subplot(gs[0, 1])
    terms = [("action rate\n‖Δa‖²", "rate_penalty"),
             ("curvature\n‖Δ²a‖²", "curv_penalty"),
             ("magnitude\n‖a‖²", "action_penalty")]
    y = np.arange(len(terms))
    h = 0.78 / len(runs)
    for k, (run, m) in enumerate(zip(runs, mets)):
        off = (k - (len(runs) - 1) / 2) * h
        vals = [m[key] for _, key in terms]
        bx.barh(y + off, vals, height=h * 0.9, color=run.color, edgecolor=SURFACE,
                linewidth=1.2, zorder=3, label=run.label)
        for v, yy in zip(vals, y + off):
            bx.annotate(f" {v:.2f}", xy=(v, yy), va="center", fontsize=8.2, color=INK2)
    bx.set_yticks(y)
    bx.set_yticklabels([t for t, _ in terms], fontsize=8.5)
    bx.set_ylim(len(terms) - 0.5, -0.5)
    # LINEAR, and anchored at zero. A bar on a log axis has a length that is not
    # proportional to its value and a left edge that is an arbitrary choice.
    bx.set_xlim(0, 1.22 * max(m[key] for m in mets for _, key in terms))
    bx.set_title("mean over the run", fontsize=10, loc="left", pad=6)
    tidy(bx, grid="x")

    fig.suptitle("The regulariser's own number", x=0.012, y=0.985, ha="left",
                 fontsize=13.5, weight="bold")
    lines = ["This is the term an action-rate penalty multiplies. Changing its weight by the "
             "ratio between two policies moves one toward the other — a starting point, not "
             "a law, since the penalty is not linear in the weight."]
    if len(runs) == 2 and min(m["rate_penalty"] for m in mets) > 0:
        hi, lo = (0, 1) if mets[0]["rate_penalty"] >= mets[1]["rate_penalty"] else (1, 0)
        k = mets[hi]["rate_penalty"] / mets[lo]["rate_penalty"]
        lines.append(f"Here {runs[hi].label} pays {k:.1f}× what {runs[lo].label} pays. Read "
                     f"that beside the reversal rates before acting on it: a busy clip earns "
                     f"a large ‖Δa‖² honestly, and no weight should take it away.")
    fig.text(0.012, 0.905, "\n".join(lines), ha="left", va="top", fontsize=8.6,
             color=MUTED, linespacing=1.5)
    fig.savefig(out, dpi=170)
    plt.close(fig)


# ── reporting ─────────────────────────────────────────────────────────────────
def write_tables(runs, mets, outdir, hf_cut):
    hdr = ["label", "motion", "steps", "target_rate_rad_s", "rate_rms", "curv_rms",
           "reversal", "hf_frac_above_%gHz" % hf_cut, "excess_vs_clip", "rate_penalty",
           "curv_penalty", "action_penalty", "gaps", "fell_at", "mean_err", "train_run"]
    with open(os.path.join(outdir, "metrics.csv"), "w") as f:
        f.write(",".join(hdr) + "\n")
        for run, m in zip(runs, mets):
            f.write(",".join(str(x) for x in [
                run.label, run.meta.get("motion", ""), run.n,
                f"{m['target_rate']:.4f}", f"{m['rate_rms']:.4f}", f"{m['curv_rms']:.4f}",
                f"{m['rev']:.4f}", f"{m['hf_fraction']:.4f}", f"{m['excess']:.4f}",
                f"{m['rate_penalty']:.4f}", f"{m['curv_penalty']:.4f}",
                f"{m['action_penalty']:.4f}", run.gaps,
                run.summary.get("fell_at", ""), run.summary.get("mean_err", ""),
                run.meta.get("train_run", "")]) + "\n")

    # Per joint, because the weight you change is global but the problem is not.
    with open(os.path.join(outdir, "metrics_per_joint.csv"), "w") as f:
        f.write("label,joint,group,target_rate_rad_s,rate_rms,ref_rate_rms,excess,"
                "reversal,torque_step_Nm\n")
        for run, m in zip(runs, mets):
            for j, name in enumerate(run.names):
                ref = m["ref_rate_rms_j"][j] if "ref_rate_rms_j" in m else float("nan")
                exc = m["excess_j"][j] if "excess_j" in m else float("nan")
                f.write(f"{run.label},{name},{run.groups[j]},{m['target_rate_j'][j]:.4f},"
                        f"{m['rate_rms_j'][j]:.4f},{ref:.4f},{exc:.4f},"
                        f"{m['rev_j'][j]:.4f},{m['torque_step_j'][j]:.4f}\n")


def print_table(runs, mets, hf_cut):
    w = max(len(r.label) for r in runs) + 2
    hf = ">%gHz" % hf_cut
    print()
    print(f"{'policy':<{w}}{'steps':>7}{'rad/s':>8}{'revers':>8}{hf:>8}"
          f"{'excess':>8}{'rate pen':>10}{'fell':>7}")
    for run, m in zip(runs, mets):
        print(f"{run.label:<{w}}{run.n:>7}{m['target_rate']:>8.2f}{m['rev']:>8.2f}"
              f"{m['hf_fraction']:>8.2f}{m['excess']:>8.1f}{m['rate_penalty']:>10.2f}"
              f"{run.summary.get('fell_at', '?'):>7}")
    print()
    print("  rad/s     RMS rate of the commanded joint target")
    print("  revers    share of control steps on which the action rate flips sign")
    print(f"  {hf:<9} share of the action's AC power above {hf_cut:g} Hz")
    print("  excess    that RMS rate divided by the SAME number on its own reference clip")
    print("  rate pen  mean ‖a[t] − a[t−1]‖², the action-rate reward term")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("npz", nargs="+", help="recordings from record_policy_actions.py")
    ap.add_argument("--out", default=".", help="directory for the figures")
    ap.add_argument("--joints", default="",
                    help="comma-separated joint names for the trace figure "
                         "(default: the jitteriest ones)")
    ap.add_argument("--n-trace-joints", type=int, default=3)
    ap.add_argument("--zoom", default="4:7",
                    help="seconds A:B of each run to show in the trace figure")
    ap.add_argument("--hf-cut", type=float, default=10.0,
                    help="the frequency above which action power is called jitter")
    args = ap.parse_args()

    runs = [Run(p, SERIES[i % len(SERIES)]) for i, p in enumerate(sorted(args.npz))]
    mets = [measure(r, args.hf_cut) for r in runs]
    os.makedirs(args.out, exist_ok=True)

    lo, _, hi = args.zoom.partition(":")
    zoom = (float(lo), float(hi))
    span = min(r.t[-1] for r in runs)
    if zoom[1] > span:
        zoom = (max(0.0, span - (zoom[1] - zoom[0])), span)
        print(f"  --zoom runs past the end of the shortest run; showing "
              f"{zoom[0]:.1f}:{zoom[1]:.1f}s instead")

    if args.joints:
        joints = []
        for want in [j.strip() for j in args.joints.split(",")]:
            hit = [j for j, n in enumerate(runs[0].names) if want in n]
            if hit:
                joints.append(hit[0])
            else:
                print(f"  no joint matching '{want}' — the names are in "
                      "metrics_per_joint.csv")
    else:
        # The jitteriest joint of EACH body group, in turn, rather than the top
        # three overall: on a policy whose arms chatter, the top three overall
        # are three wrists, and the figure then says nothing about the legs.
        # Ranked on the worst rate ANY policy shows, so a joint only one of them
        # struggles with is still picked.
        worst = np.max([m["rate_rms_j"] for m in mets], axis=0)
        ranked = {g: sorted([j for j in range(len(worst)) if runs[0].groups[j] == g],
                            key=lambda j: -worst[j]) for g in GROUPS}
        joints, rank = [], 0
        while len(joints) < args.n_trace_joints and rank < len(worst):
            for g in GROUPS:
                if rank < len(ranked[g]) and len(joints) < args.n_trace_joints:
                    joints.append(ranked[g][rank])
            rank += 1
    joints = joints or [0]

    for r in runs:
        if r.gaps:
            print(f"  {r.label}: {r.gaps} gap(s) in /lowcmd — those differences are excluded")

    fig_scorecard(runs, mets, os.path.join(args.out, "01_smoothness_scorecard.png"), args.hf_cut)
    fig_traces(runs, mets, os.path.join(args.out, "02_action_traces.png"), joints, zoom)
    fig_spectrum(runs, mets, os.path.join(args.out, "03_action_spectrum.png"), args.hf_cut)
    fig_heatmap(runs, os.path.join(args.out, "04_rate_heatmap.png"))
    fig_penalty(runs, mets, os.path.join(args.out, "05_rate_penalty.png"))
    write_tables(runs, mets, args.out, args.hf_cut)
    print_table(runs, mets, args.hf_cut)
    print(f"  figures and csv: {os.path.abspath(args.out)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
