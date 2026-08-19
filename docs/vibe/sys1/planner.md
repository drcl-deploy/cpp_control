# sys1 — the retrieval planner above SONIC

Given a target colour, sys1 looks at the cube, picks a recorded quarter-turn clip
that reaches it from where the robot actually stands, and writes it into sys0's
reference. Two modes, one state integer, one ranking scalar.

Reference implementation: vibe `src/vibe/repose/planner/clips.py` (algorithm) and
`command.py` (the sim seam). Behaviour spec: vibe `docs/sys1_v5.md`. Migration
spec: vibe `docs/sys1_cpp.md` — **read §1 below before that one**, four of its
structural requirements do not apply to this stack. This port has since diverged
from the Python by design: the planner is split OBSERVE / PLAN / ACT (§3), and
the versions it carried are down to one ablation (§0). Build record for that
split, with the parity matrix: [v7_simplified.md](v7_simplified.md).

**Off by default.** `sys1:=false` is bit-for-bit the open-loop rig.

---

## 0. One shipping config, one ablation

v5 and its knobs are gone: the A/B is finished, the numbers are below, and a
binary carrying two dead configurations is not a minimal planner. What survives
is `nominal_stand`, because that ablation is the result.

| knob | v6 | v7 | what it does |
|---|---|---|---|
| `nominal_stand` | false | **true** | hold the robot's own nominal stance in a still mode, not the library's stand frame |

Everything the versions used to split — `pattern: RRF`, `horizon_gain: 0.3`,
`min_visible_color: 0.0` — is now simply the default. `stall_limit` and
`scan_peek_every` were built, measured, shipped OFF, and are now deleted;
`clips.py`'s docstrings remain the post-mortems.

`nominal_stand` is one boolean and it is the largest win in the algorithm's
history: sim solve 62.5% → **95%** (n=120, z=6.15), usable reads 35% → 82–89%,
`side_face` 44–47% → **5–7%**. Not a perception fix — a **gaze** fix. The
library's quietest frame was chosen for quietness and carries a waist that aims
the head at the near ground. Measured on this deployment's own mount quat and
its own baked stand row, and asserted every run by `sys1_selftest`:

| still pose | waist yaw/roll/pitch | camera | cube-top band |
|---|---|---|---|
| library frame 12953 | −14.0° / −4.5° / **+26.7°** | **71.2° down, −27.2° off-axis** | ends at 0.57 m — *before* the median clip's own exit, so the cube left view on its own every other commit |
| **nominal stance** | 0 / 0 / 0 | **45.0° down, 0.0°** | 0.26 – 1.35 m, 100% of clip exits in view |

45.0° is the camera's **mount angle**, recovered exactly because a nominal pose
has waist = 0 and the torso is therefore vertical. No search, no tuning. It is
also why a CLIP is never read from: a clip ends in an arbitrary exit pose whose
waist aims the head wherever the recording left it, which is the v6 gaze again.

---

## 0.5 sys1 is deliberately stupid

sys1 exists to make sys0's *implicit* competence measurable — object
localisation, contact correction, whole-body recovery — none of which sys0 was
told to have. A planner that compensated for any of it would hide the result.
So three things that look like omissions are load-bearing:

| looks like a gap | why it stays |
|---|---|
| the ladder is a blind roll pattern, ~3 rolls to a target where a map-aware solver needs 1.2 | a solver would be doing the demo's work for it |
| retrieval has **no cost ceiling** — a clip commits at any stance residual | that residual is the experiment. `cost` is published so a bag can price it: **P(tip \| cost)** is sys0's implicit-localisation curve |
| there is no approach primitive — 78 quarter-turns and nothing that walks | whatever closes the gap is sys0's |

Instrument, never guard.

## 1. Four things this stack does not need

The migration spec was written against the sim's data model. cpp_control's SONIC
seam is narrower, and that deletes most of it.

| # | in the sim | here |
|---|---|---|
| F1 | cube-exact SE(2) warp of 19852 frames | **one scalar.** `fill_tokenizer` reads `jp`, `jv` and root ORIENTATION only; `motion_cmd` reads `jp`/`jv`; the adapter's twist is body-frame. Root POSITION reaches no observation, so the translation half of the warp is unobservable and `_pristine` disappears |
| F2 | `robot_yaw`, `root_xy`, odometry drift budget | **nothing.** The tokenizer's 6D is `qinv(imu) · ref_quat` and sys1's yaw comes from the same IMU, so the IMU *is* the world frame and drift cancels in every difference |
| F3 | a 10-slot ring buffer, rewritten every step | **`MotionClock`.** `future_frame(k)` already yields `t + [0,5,…,45]` clamped to the clip end — same `FUTURE_STEPS`/`FRAME_SKIP`. A still mode is published as a clip whose frames happen to be identical apart from the yaw ramp |
| F4 | a 12-frame blend at commit | **a rate-limited lead-in**, on *every* commit — clips and stills alike. A reference that steps is a step input to a balancing policy. `lead_in_rate` walks there instead — the same mechanism the human-driven L1 prep provides, which is why the prep gate turns off under sys1. v7 makes this matter more, not less: the nominal stance is sys1's own vocabulary, so it sits further from a clip's exit pose than the library frame did, and the settle after every clip is the largest joint step in the loop |

What survives of the warp is `entry_yaw_offset`, and it is exactly the heading
term the retrieval minimises:

```
entry_yaw = wrap(qth[row] + sym·π/2 + φ)          # the plan
dth       = wrap(qth[row] + sym·π/2 − (−φ))       # _se2's heading residual
```

`sys1_selftest` asserts they are the same number for all four symmetries — rank
one thing, command another, and the planner is lying to itself.

## 2. Two rates, no clock

sys1 owns no clock. The ACT is timed off `Sys0Status`, so what the robot is
ACTUALLY playing is the only thing that advances the plan — and a human pressing
`B` mid-clip parks the planner instead of leaving it talking to itself. Only the
act waits: observing and planning run every tick (§3).

```
[50 Hz HARD RT]  /lowstate -> g1_vibe_sonic_node -> /lowcmd
                     |  reads MotionReference, plays it verbatim
                     +-> /vibe/sys0/status ---------------+
                                                          v
[20 Hz SOFT RT]  camera TCP -> g1_sys1_repose_node -> /tracker/reference
                     ^ /lowstate (FK + IMU), /vibe/sonic/goal_color
                     +-> /vibe/sys1/status -> scripts/sys1/sys1_console.py
```

Budget: `classify` is the only hot spot (12 squared distances per live pixel at
160×120, <1 ms), everything else <0.3 ms, the warp is ~50 µs. **~2% of one core.**
Clip spans resident: 3.5 MB.

## 3. The loop — OBSERVE / PLAN / ACT

The three used to be fused at the `finished` edge, and every special case in the
planner existed to patch that fusion. Split apart, they collapse into four lines
and one mechanism.

```
tick():                                                       # 20 Hz
    GUARD    sys0 accepting? · lowstate fresh? · nominal stance known?
    OBSERVE  not in a CLIP, new frame, |omega_cam| < omega_still
                 -> belief << look()
    PLAN     clips.observe(belief)        # ladder steps, done latches
             candidate = clips.decide(belief)      # PURE, ~50 us, every tick
    ACT      sys0.finished        -> commit(candidate); belief.clear()
             id never echoed      -> resend, once per 4 ticks

decide(belief):                           # a pure function, four branches
    done (LATCHED)                        -> STILL(0)
    belief.n_reads < min_votes            -> STILL(0)     # BLIND, do not guess
    !belief.valid or !belief.pose_ok      -> STILL(±psi)  # scan
    else                                  -> CLIP = argmin over pool × 4 syms
```

Structurally there are **two** modes, not three: a CLIP, and a STILL whose yaw is
either zero (SETTLE) or swept (SCAN). They share one builder and one writer
branch; the message keeps them apart only because a bag reads better for it.

### The belief

One vote over the last `belief_window` reads taken while the camera was quiet.
It replaces four rules outright:

| was | now |
|---|---|
| read only in the last `2·read_tail` frames | read whenever `omega_cam < omega_still` — the physical condition the window was a proxy for |
| the tip counts only on the read `decide()` consumed, or a flicker walks three rungs | the rung steps off the **voted** colour, which changes at most once per real tip *by construction* |
| a stale camera **parks** the planner | a stale camera **starves the belief**, and a starved belief plans the nominal stance. A camera lost mid-clip now puts the robot back on its feet instead of freezing it in a half-turned exit pose |
| `done` re-litigated every cycle, so one flickered read rolled a solved cube away | `done` LATCHES; the operator clears it by re-picking a colour (console `R`), which is also the trial loop |

`decide()` being pure is what lets it run at 20 Hz and commit at 1 Hz: the clip
is burned and the roll counted in `clips.commit()`, at the moment a reference is
actually published. `sys1_selftest` asserts twenty calls answer identically and
burn nothing.

**A CLIP is never read from.** It has quiet moments, but it spends them in an
arbitrary exit pose whose waist aims the head wherever the recording left it —
the v6 gaze, §0. So a clip ends with an empty belief, and an empty belief plans
the stance: the mandatory settle after every clip is now a *consequence* of the
mechanism rather than a branch in the loop.

### Sizing the evidence

Only the quiet part of a still is readable, so `hold_tail` is sized in **reads**,
not in camera timing. At 50 fps reference and a 20 Hz tick:

| still | frames | ψ | sweep | quiet | reads |
|---|---|---|---|---|---|
| SETTLE | 40 / 0.80 s | 0 | — | the whole hold | ~12 |
| SCAN | 110 / 2.20 s | ±90° | 1.60 s @ 0.98 rad/s | 0.60 s | ~12 |

The sweep rate is unchanged from the 90-frame SCAN it replaces; the extra 0.4 s
is all quiet time. Too few reads is self-correcting and never wanders — the
planner settles again, which is exactly the state that produces reads.

A deliberate departure from the Python survives: **a SCAN commands a turn rate.**
`robot_root_ang_vel_cmd` carries the sweep rate while the ramp runs; zero would
lie to the adapter for the whole turn.

### What to read off `/vibe/sys1/status`

Published **every tick**, not once per act — the belief and the candidate plan
both move between commits.

| field | for |
|---|---|
| `omega_cam`, `accept_rate` | placing `omega_still`: a held stance reads ~0.05, a sweep ~0.96 |
| `n_votes` / `n_reads` | how much evidence a decision stood on. 3/3 and 3/12 are not the same claim |
| `cand_*` | what the planner *would* commit if sys0 finished now — intent, ~a second early |
| `cost` + `tips` across a CLIP row | **P(tip \| cost)**, the implicit-localisation curve |
| `rolls`, `episode` | rolls-to-solve, per trial, without inferring episode boundaries |

## 4. Wire, and the frames it carries

Colour and depth arrive in ONE record on the existing camera socket, parsed by
`vision_encoders/frame_wire.hpp`. A producer without depth is byte-identical to
before.

| plane | format | why |
|---|---|---|
| colour | JPEG/PNG, 640×480 | unchanged |
| depth | **raw u16 mm, 160×120** | sys1 classifies at 160 px anyway, so this is 38 KB and 0 ms of encode — *smaller* than the JPEG. PNG-16 at full res costs 10–25 ms on the Orin |

Intrinsics ride **per-plane, in that plane's own pixels**, so sys1 has no camera
calibration in config and a downscaled depth plane stays exact.

```bash
python3 camera_streamer.py --depth            # Orin
camera_depth: 1                               # unitree_mujoco/simulate/config.yaml
```

## 5. Artifacts

| what | from | read by |
|---|---|---|
| `sys1_clips.npz` | vibe bench (offline; the bake stays Python) | the 78-row alphabet |
| `sys1_library.npz` | same | `motion_files` only, for `source: retargeted` |
| `retargeted_motions/` | the dataset | spans sliced at boot |
| a baked bundle | `sys1_selftest --bake` | the Orin: one file, no dataset |

sys1 brings its own npz reader: numpy writes ZIP64 local headers, which cnpy
reads as 4 GB lengths, and `motion_files` is an `object` dtype cnpy cannot parse
at all.

```bash
sys1_selftest --table sys1_clips.npz --library sys1_library.npz \
              --frames <retargeted_root> --bake sys1_bundle.npz
```

## 6. Running it

Full runbook: [experiments.md](experiments.md). The shape, and why it is that
shape — `g1_sys1_repose` is a **sibling** of `g1_vibe_repose`, not a layer on
it, so both get the bridge, the encoder and the controller from the one common
file and differ only by `sys1:=true` plus the planner process:

```
g1_vibe_sonic.launch.py            bridge + encoder + controller
  ├── g1_vibe_repose.launch.py        open-loop, untouched by sys1
  └── g1_sys1_repose.launch.py        + sys1:=true + g1_sys1_repose_node
```

```bash
ros2 launch cpp_control g1_sys1_repose.launch.py artifact_dir:=/path/to/export
bash scripts/sys1/run_sys1.sh      # 0-5 or r/o/g/y/b/p sets the target
```

The camera address comes from the launch, not from the yaml — one `camera_ip`
reaches the encoder and the planner, so the two halves of a run cannot end up
watching different cameras. `camera:` in the yaml is the standalone fallback.

`/vibe/sonic/goal_color` is read by BOTH the planner and the policy's
`object_goal_color` one-hot — one topic, so they cannot disagree about what "4"
means. The index order is `vibe::cube_color_name`: 0 red, 1 orange, 2 green,
3 yellow, 4 blue, 5 pink.

All three state topics ride the telemetry bridge in `full.yaml` and
`light.yaml`, silent unless a planner is up. So the standard viewers and bags
carry sys1 with no profile switch, and `replay_vibes.py` grows a planner pane
when a bag has one.

## 7. What `sys1:=true` changes in sys0

| | false (default) | true |
|---|---|---|
| references | staged, `A` commits | auto-commit on arrival |
| re-engage | resets observation histories | keeps them (a swap every ~2 s would starve every history term) |
| L1 prep gate | on | off — the lead-in replaces it |
| `Sys0Status` | not published | 20 Hz, its own timer |
| `entry_yaw_offset` | 0 from every publisher, so inert | applied at engage |

`Sys0Status.default_joint_pos` carries the manifest's nominal stance, and sys1's
v7 still pose is built from it rather than parsed here a second time — **one
source**, so the stance the planner commands and the stance the controller holds
in `NOMINAL_POSE` cannot drift. It also means the pose is the *deployed* robot's
by construction (`sys1_cpp.md` §8.6) and a baked bundle stays version-agnostic.
The planner refuses to arm until it has arrived, and logs the achieved gaze:

```
v7 nominal stand: waist 0.0/0.0/0.0 deg -> camera 45.0 deg down, +0.0 off-axis (mount is 45.0)
```

A robot whose nominal stance carries a stooped waist gets v6's band back, so
that line warns rather than fails — it is a config truth, not a code fault.

## 8. Bring-up order

| phase | what | gate |
|---|---|---|
| P0 | the wire + `sys1_selftest --replay` on recorded reads | labels match the Python oracle; `Sight.pos/.phi` ≤ 1e-4. Settles the palette and depth-realism risks with the robot standing still |
| P0.5 | the **gaze**, `sys1_selftest` unit checks | 45.0 ± 0.5° down, \|yaw\| < 0.5°. Free, offline, every run — and it is the whole v7 result, so nothing else is worth measuring until it passes |
| P1 | `retrieve()` picks the same `(row, sym)` as the oracle | offline |
| P2 | closed loop in sim | 40 trials against `sys1_eval.py` |
| P3 | onboard, **settle/scan only** | ψ, reference continuity, the 20 Hz budget under the live loop. Also where `omega_still` gets placed: watch `omega_cam` and `accept_rate` |
| P4 | clip commits on hardware | — |

**The palette is the most fragile piece.** The classifier has no "none" class:
every saturated pixel is forced into one of six chromaticities, and the twelve
refs were measured off the sim renderer. Re-measure under the deployment lights
(P0's replay rig dumps per-candidate stats for exactly this) and keep the floor
neutral, so it fails the saturation gate and never votes.
