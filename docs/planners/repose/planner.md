# The planner — the retrieval planner above SONIC

Given a target colour, the planner looks at the cube, picks a recorded quarter-turn clip
that reaches it from where the robot actually stands, and writes it into the controller's
reference. Two modes, one state integer, one ranking scalar.

Reference implementation: vibe `src/vibe/repose/planner/clips.py` (algorithm) and
`command.py` (the sim seam). Behaviour spec: vibe `docs/sys1_v5.md`. Migration
spec: vibe `docs/sys1_cpp.md` — **read §1 below before that one**, four of its
structural requirements do not apply to this stack. This port has since diverged
from the Python by design: the planner is split OBSERVE / PLAN / ACT (§3), and
the versions it carried are down to one ablation (§0). Build record for that
split, with the parity matrix: [v7_simplified.md](v7_simplified.md).

**Off by default.** `the planner:=false` is bit-for-bit the open-loop rig.

---

## 0. One shipping config, four ablations

v5 and its knobs are gone: the A/B is finished, the numbers are below, and a
binary carrying dead configurations is not a minimal planner. Each experimental
version is **one knob** over its declared base, so one binary can A/B it.

| version | knob | what it does |
|---|---|---|
| v6 | — | the baseline: `RRF`, `horizon_gain 0.3`, graded sighting |
| **v7** | `nominal_stand` | hold the robot's own nominal stance in a still mode, not the library's stand frame |
| v7.1 | `enter_yaw_rate_deg` | put the HEADING in the lead-in ramp, at a bounded rate (§7.1) |
| v7.2 | `kinematic_scan` | replace only the stationary SCAN sweep with a GEAR-SONIC stepping turn (§7.2) |
| v7.3 | `kinematic_scan_pure` + bounded native creep | fixed-angle Slow-Walk/Idle turns, searched directly with quiet reads inside scan-k (§7.3) |

Everything the versions used to split — `pattern: RRF`, `horizon_gain: 0.3`,
`min_visible_color: 0.0` — is now simply the default. `stall_limit` and
`scan_peek_every` were built, measured, shipped OFF, and are now deleted;
`clips.py`'s docstrings remain the post-mortems.

`nominal_stand` is one boolean and it is the largest win in the algorithm's
history: sim solve 62.5% → **95%** (n=120, z=6.15), usable reads 35% → 82–89%,
`side_face` 44–47% → **5–7%**. Not a perception fix — a **gaze** fix. The
library's quietest frame was chosen for quietness and carries a waist that aims
the head at the near ground. Measured on this deployment's own mount quat and
its own baked stand row, and asserted every run by `repose_planner_selftest`:

| still pose | waist yaw/roll/pitch | camera | cube-top band |
|---|---|---|---|
| library frame 12953 | −14.0° / −4.5° / **+26.7°** | **71.2° down, −27.2° off-axis** | ends at 0.57 m — *before* the median clip's own exit, so the cube left view on its own every other commit |
| **nominal stance** | 0 / 0 / 0 | **45.0° down, 0.0°** | 0.26 – 1.35 m, 100% of clip exits in view |

45.0° is the camera's **mount angle**, recovered exactly because a nominal pose
has waist = 0 and the torso is therefore vertical. No search, no tuning. It is
also why a CLIP is never read from: a clip ends in an arbitrary exit pose whose
waist aims the head wherever the recording left it, which is the v6 gaze again.

---

## 0.5 the planner is deliberately stupid

The planner exists to make the controller's *implicit* competence measurable — object
localisation, contact correction, whole-body recovery — none of which the controller was
told to have. A planner that compensated for any of it would hide the result.
So three things that look like omissions are load-bearing:

| looks like a gap | why it stays |
|---|---|
| the ladder is a blind roll pattern, ~3 rolls to a target where a map-aware solver needs 1.2 | a solver would be doing the demo's work for it |
| retrieval has **no cost ceiling** — a clip commits at any stance residual | that residual is the experiment. `cost` is published so a bag can price it: **P(tip \| cost)** is the controller's implicit-localisation curve |
| there is no approach primitive — 78 quarter-turns and nothing that walks | whatever closes the gap is the controller's |

Instrument, never guard.

## 1. Four things this stack does not need

The migration spec was written against the sim's data model. cpp_control's SONIC
seam is narrower, and that deletes most of it.

| # | in the sim | here |
|---|---|---|
| F1 | cube-exact SE(2) warp of 19852 frames | **one scalar.** `fill_tokenizer` reads `jp`, `jv` and root ORIENTATION only; `motion_cmd` reads `jp`/`jv`; the adapter's twist is body-frame. Root POSITION reaches no observation, so the translation half of the warp is unobservable and `_pristine` disappears |
| F2 | `robot_yaw`, `root_xy`, odometry drift budget | **nothing.** The tokenizer's 6D is `qinv(imu) · ref_quat` and the planner's yaw comes from the same IMU, so the IMU *is* the world frame and drift cancels in every difference |
| F3 | a 10-slot ring buffer, rewritten every step | **`MotionClock`.** `future_frame(k)` already yields `t + [0,5,…,45]` clamped to the clip end — same `FUTURE_STEPS`/`FRAME_SKIP`. A still mode is published as a clip whose frames happen to be identical apart from the yaw ramp |
| F4 | a 12-frame blend at commit | **a rate-limited lead-in**, on *every* commit — clips and stills alike. A reference that steps is a step input to a balancing policy. `lead_in_rate` walks there instead — the same mechanism the human-driven L1 prep provides, which is why the prep gate turns off under the planner. v7 makes this matter more, not less: the nominal stance is the planner's own vocabulary, so it sits further from a clip's exit pose than the library frame did, and the settle after every clip is the largest joint step in the loop |

What survives of the warp is `entry_yaw_offset`, and it is exactly the heading
term the retrieval minimises:

```
entry_yaw = wrap(qth[row] + sym·π/2 + φ)          # the plan
dth       = wrap(qth[row] + sym·π/2 − (−φ))       # _se2's heading residual
```

`repose_planner_selftest` asserts they are the same number for all four symmetries — rank
one thing, command another, and the planner is lying to itself.

## 2. Two rates, no clock

The planner owns no clock. The ACT is timed off `ControllerStatus`, so what the robot is
ACTUALLY playing is the only thing that advances the plan — and a human pressing
`B` mid-clip parks the planner instead of leaving it talking to itself. Only the
act waits: observing and planning run every tick (§3).

```
[50 Hz HARD RT]  /lowstate -> g1_vibe_sonic_node -> /lowcmd
                     |  reads MotionReference, plays it verbatim
                     +-> /vibe/controller/status ---------------+
                                                          v
[20 Hz SOFT RT]  camera TCP -> g1_repose_planner_node -> /tracker/reference
                     ^ /lowstate (FK + IMU), /vibe/sonic/goal_color
                     +-> /vibe/planner/status -> scripts/planners/repose/repose_console.py
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
    GUARD    the controller accepting? · lowstate fresh? · nominal stance known?
    OBSERVE  not in a CLIP, new frame, |omega_cam| < omega_still
                 -> belief << look()
    PLAN     clips.observe(belief)        # ladder steps, done latches
             candidate = clips.decide(belief)      # PURE, ~50 us, every tick
    ACT      the controller.finished        -> commit(candidate); belief.clear()
             id never echoed      -> resend, once per 4 ticks

decide(belief):                           # a pure function, four branches
    done (LATCHED)                        -> STILL(0)
    belief.n_reads < min_votes            -> STILL(0)     # BLIND, do not guess
    !belief.valid or !belief.pose_ok      -> STILL(±psi)  # scan
    else                                  -> CLIP = argmin over pool × 4 syms
```

Through v7.1 there are structurally **two** modes: a CLIP, and a STILL whose yaw
is either zero (SETTLE) or swept (SCAN). v7.2 adds one locomotion act,
`KINEMATIC_SCAN`; it is deliberately followed by the same nominal SETTLE that
makes a CLIP readable. v7.3 makes that act self-readable at its held tail and
may follow it directly with another `KINEMATIC_SCAN`.

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
actually published. `repose_planner_selftest` asserts twenty calls answer identically and
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

### What to read off `/vibe/planner/status`

Published **every tick**, not once per act — the belief and the candidate plan
both move between commits.

| field | for |
|---|---|
| `omega_cam`, `accept_rate` | placing `omega_still`: a held stance reads ~0.05, a sweep ~0.96 |
| `n_votes` / `n_reads` | how much evidence a decision stood on. 3/3 and 3/12 are not the same claim |
| `cand_*` | what the planner *would* commit if the controller finished now — intent, ~a second early |
| `cost` + `tips` across a CLIP row | **P(tip \| cost)**, the implicit-localisation curve |
| `rolls`, `episode` | rolls-to-solve, per trial, without inferring episode boundaries |

## 4. Wire, and the frames it carries

Colour and depth arrive in ONE record on the existing camera socket, parsed by
`vision_encoders/frame_wire.hpp`. A producer without depth is byte-identical to
before.

| plane | format | why |
|---|---|---|
| colour | JPEG/PNG, 640×480 | unchanged |
| depth | **raw u16 mm, 160×120** | the planner classifies at 160 px anyway, so this is 38 KB and 0 ms of encode — *smaller* than the JPEG. PNG-16 at full res costs 10–25 ms on the Orin |

Intrinsics ride **per-plane, in that plane's own pixels**, so the planner has no camera
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
| a baked bundle | `repose_planner_selftest --bake` | the Orin: one file, no dataset |

The yaml names all three relative to `$VIBE_ASSET_ROOT` — `data/sys1_clips.npz`,
not a home directory — so one file boots on the desktop and on the Orin
([assets.md](../../vibe/assets.md)). Absolute still wins where you want it.

The planner brings its own npz reader: numpy writes ZIP64 local headers, which cnpy
reads as 4 GB lengths, and `motion_files` is an `object` dtype cnpy cannot parse
at all.

```bash
repose_planner_selftest --table sys1_clips.npz --library sys1_library.npz \
              --frames <retargeted_root> --bake repose_bundle.npz
```

## 5.5 The config

`config/repose_planner/g1.yaml` is the only truth for every knob the observe and
plan stack has. `version:` names a preset in `cfg.hpp` — a measured point in the
ablation ledger — and supplies the defaults; every key overrides it.

| block | holds |
|---|---|
| `observe.read` | **`blobs`** or **`mask`** — which order the gates run in (§5.6) |
| `observe.palette` | the 12 chromaticity refs, `<colour>: {lit, shaded}` RGB. Partial is fine: an unlisted colour keeps the preset's row |
| `observe.gates` | which pixels are colour at all — `min_value`, `min_rel_sat`, `min_area_frac`, `min_px_floor`, `chroma_reject` |
| `observe.geometry` | which blobs are a top face — `up_dot_min`, `min_visible`, `min_visible_color`, `big_max`, `slab_frac`, `z_min_m`. All in cube edges or unit normals, so none of it is range- or resolution-dependent |
| `observe.mask` | `mask` read only — `top_pct`, `band_m`, `erode` |
| `belief` | `window`, `min_votes`, `omega_still` |
| `loop` | the ladder and the stills |
| `kinematic_scan` | v7.2+ planner mode/speed, deterministic seed, bounded turn, native creep/stop, displacement budget, quiet tail, and read wait |
| `seam` | lead-in, blend, the v7.1 ramp |

Two files ship, and the launch picks between them with `env:=sim|real`
(**sim by default**): `g1_sim.yaml` is the untuned base and currently selects
the v7.3 bring-up experiment; `g1_real.yaml` now selects the same experiment
with its measured palette and `mask` read for the requested hardware trial.
Set its one `version:` line back to v7 for the established stationary-SCAN
baseline. Apart from the measured perception block, both files share the
planner tuning.

Two rules the loader enforces, both by throwing:

1. **An unknown key is an error.** A silently ignored knob reads as tuned and
   behaves as stock, which is the worst outcome available.
2. **The version-owned knobs are absent from the shipped file** —
   `loop.nominal_stand`, `seam.enter_yaw_rate_deg`, `seam.lead_in_max_s`, and the
   internal `kinematic_scan` and pure-search enables plus native creep mode/speed. Pinning one in yaml would make `version:`
   stop switching the ablation it exists to switch. `ros2 run cpp_control
   repose_planner_selftest --config <file> --check-preset-parity` asserts all
   five presets still round-trip through the untuned sim file, so the day
   someone pins one, the test says so. The real file is deliberately tuned and
   is checked without this parity flag.

The pre-split flat `knobs:` block is refused by name, with the migration in the
message.

`clips`, `frames.path` and `frames.library` are the only paths in the file, and
all three go through `asset_path()` (§5).

## 5.6 Two reads, and why hardware needed the second

`observe.read` picks the ORDER of the same gates. Nothing else differs — same
thresholds, same meanings, same units. Full theory, with the diagrams:
[observe_calibration.md](observe_calibration.md).

| | `blobs` (shipping) | `mask` |
|---|---|---|
| first | colour: six independent blob extractions | geometry: one cube-top plane off every coloured pixel |
| then | gate each on geometry, **highest z wins** | modal colour over the one mask |
| fails when | one face straddles two chromaticity cells and its own fragment out-heights it | the cube is not the highest coloured thing in frame |

Measured on `bags/sys1_observe` — 180 labelled hardware frames, 60 no-cube —
scored by the production `CubeSight` through `repose_planner_selftest --replay`:

| config | accuracy | false positives |
|---|---|---|
| `blobs`, sim palette (**shipping**) | 53.9% | 0/60 |
| `blobs`, measured palette | 61.7% | 4/60 |
| `mask`, sim palette | 48.9% | 0/60 |
| **`mask`, measured palette, `chroma_reject` 0.06, `min_rel_sat` 0.15** | **83.3%** | **0/60** |

**Neither half works alone.** The palette alone moves `blobs` by 8 points because
54 of its 83 errors are the true blob passing every gate and then losing the
height contest; the read alone is worse than shipping because the sim palette
mislabels the pixels it is now voting over. The gain is the pair.

`chroma_reject` is what makes the pair safe: with a "none" class, `min_rel_sat`
can drop to 0.15 to catch a pale face — the deployment's green sits at 0.28
median relative saturation, right on the old 0.22 gate — without the floor
walking into the vote. All 60 no-cube frames still answer "none".

## 6. Running it

Full runbook: [experiments.md](experiments.md). The shape, and why it is that
shape — `g1_repose_planner` is a **sibling** of `g1_vibe_repose`, not a layer on
it, so both get the bridge, the encoder and the controller from the one common
file and differ only by `planner:=true` plus the planner process:

```
g1_vibe_sonic.launch.py            bridge + encoder + controller
  ├── g1_vibe_repose.launch.py        open-loop, untouched by the planner
  └── g1_repose_planner.launch.py        + the planner:=true + g1_repose_planner_node
```

```bash
ros2 launch cpp_control g1_repose_planner.launch.py artifact:=<run>/<export>
bash scripts/planners/repose/repose_run_console.sh      # 0-5 or r/o/g/y/b/p sets the target
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
carry the planner with no profile switch, and `replay_vibes.py` grows a planner pane
when a bag has one.

## 7. What `planner:=true` changes in the controller

| | false (default) | true |
|---|---|---|
| references | staged, `A` commits | `A` arms; auto-commit on arrival until `RB` disarms |
| re-engage | resets observation histories | keeps them (a swap every ~2 s would starve every history term) |
| L1 prep gate | on | off — the lead-in replaces it |
| `ControllerStatus` | not published | 20 Hz, its own timer |
| `entry_yaw_offset` | 0 from every publisher, so inert | applied at engage |

`ControllerStatus.default_joint_pos` carries the manifest's nominal stance, and the planner's
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

## 7.1 The ENTER ramp (v7.1)

`enter_yaw_rate_deg: 0` is v7 exactly, byte for byte, and the selftest asserts
it. Non-zero puts the heading residual into the lead-in the joints already ride.

**What was wrong.** `MotionClock::engage` pins reference frame 0 to
`robot_yaw + entry_yaw`, so the whole heading residual arrives in one frame
while the joints walk. Measured over the clean pools a commit asks med 9–22°,
p90 21–45° — 157–321 °/s through `blend_frames`, 3–6× the only rate this stack
has ever measured as followable (the SCAN sweep, 50 °/s).

**The fix, in one observation.** `engage` cancels the **frame-0** heading and
nothing else, so a rotation that VARIES across the lead-in survives it. The
lead-in rows are rotated by `−entry_yaw·(1−s)` and `entry_yaw_offset` goes to
**zero** on the wire: the residual is claimed exactly once, by whoever carried
it. This is the mechanism the SCAN sweep already used — after v7.1 the heading
always lives in the rows, one convention for both modes.

Sizing is a PEAK rate, not a frame count, because what is bounded is what the controller
can follow. `T = 1.5·max(Δθ/ω_max, Δq/q̇_max)` — the 1.5 is the smoothstep's
peak slope over its mean — clamped to `[lead_in_min_s, lead_in_max_s]`. The
ceiling moves to 1.2 s with the preset: 0.6 s truncates even the median ask.

### The split — and why prepending the ramp does not work

The first build prepended the ramp to the clip's own reference: one message, one
`engage`. It read worse in sim2sim than v7 did, and the reason is the one thing
this seam does NOT share with the sim.

**`MotionClock::engage` stamps the reference's world anchor from row 0 and never
re-measures it.** Everything after that frame is dead reckoning. Under v7 that
window is the ~0.5 s lead-in; a 1.2 s ramp is 5x, and drift over it lands
directly as a cube miss. The sim never sees this because its warp is
*cube-absolute* and re-solved at the clip's own commit (`_maybe_enter` →
`_warp_to_cube`), so the ramp costs it nothing in accuracy.

So the port takes the sim's mode boundary after all — **the ramp is its own act**:

    still --commit--> ENTER (own reference, own engage) --finished--> CLIP (re-engaged)

| | carries |
|---|---|
| ENTER reference | the ramp rows only. `entry_yaw_offset = 0` — the rows carry the heading. Burns no clip |
| at the ENTER commit | `enter_target_yaw = wrap(robot_yaw + entry_yaw)` — the ABSOLUTE heading, frozen while the robot is still quiescent. `entry_yaw` is base-frame and goes stale the instant it turns |
| CLIP reference | the raw span. No lead-in (nothing left to lead in from), no blend (it would re-open what the ramp closed). `entry_yaw = wrap(enter_target_yaw − robot_yaw)` — what the ramp *failed* to pay, measured |

Both halves are rebuilt from frozen inputs, so a resend is byte-identical.

Two smaller fixes rode along, both ramp-gated:

| | was | is |
|---|---|---|
| root twist across the ramp | ramped to the clip's entry twist | **zero**. The ramp's root position is pinned at the entry anchor, so a nonzero velocity is a lie the robot walks on |
| where the ramp starts | the LIVE pose — a reference step of one tracking error at row 0 | the last row of the reference the controller is actually playing (`LiveState::held_row`), Python's `_held_pose` |

### What the port still does not need

| sim needed | here |
|---|---|
| a previewed horizon (`_write_enter` at 10 phases) | the controller's future window reads the published rows |
| a frozen warp *anchor* (psi + cube xy) | no odometry, so no xy to freeze; the heading half is `enter_target_yaw` above |
| `_pick_still_slot` (3-frame headroom) | stills are synthesized, 40 rows |
| suppressing `_blend_entry` after a ramp | `Stage::CLIP` emits the bare span |

### The joint seam, measured — and why it stays open

The lead-in used to arrive at **zero** joint velocity and hand off to the clip's
entry frame. Those frames are spans sliced mid-motion, so they are already
moving: **med 4.8, p90 7.1, max 8.8 rad/s** over the 56 clean clips. v7.1's
Hermite matches the arrival velocity, but under a Fritsch–Carlson **monotone**
clamp — matched unclamped the curve detours ~0.8 rad backwards to wind up for
it, which is a worse thing to command than the step it removes.

| at the seam | v7 | v7.1 |
|---|---|---|
| position path | C0, monotone | C0, monotone (asserted, 0 overshoot) |
| joint-velocity step | 5.19 rad/s | 4.46 rad/s |
| peak commanded joint rate | 2.3 rad/s | **1.43** rad/s (the ramp is longer) |
| peak commanded yaw rate | one-frame step | **0.87 rad/s**, inside the 50 °/s ask |

**The remaining 4.5 rad/s is the alphabet's, not the seam's** — no ramp duration
removes it without overshooting. It is a discontinuity in a reference *channel*
the tokenizer reads, not in the commanded position, so it is a surprise to the
policy rather than a torque step. Closing it is a clip-slicing change in vibe
(cut spans at low-velocity frames), not a planner change.

## 7.2 The stepping SCAN (v7.2)

The stationary SCAN reference works in sim but asks a standing policy to turn
loaded feet on hardware. v7.2 changes that primitive only. The Repose node owns
the ROS-free GEAR-SONIC planner engine and requests:

```text
mode               = Walk
target_speed       = 0.10 m/s
movement_direction = [0, 0, 0]
facing_direction   = [cos(scan_yaw), sin(scan_yaw), 0]
```

The zero movement vector is intentional. Under a locomotion mode the released
model interprets it as a stepping turn toward the fixed facing target. A model
regression check at 45 degrees measures about 39 degrees of generated yaw and
2 cm of net reference translation.

This is a discrete act, not a new continuous planner:

```text
SETTLE -> KINEMATIC_SCAN -> SETTLE -> observe / decide
```

The moving act is never read from. Even when its final frames are quiet, its
waist is generated rather than v7's known nominal gaze. Commit clears the
belief; the empty belief therefore commands SETTLE next, and only that stance
may refill it. Every generation starts from measured joint positions with a
canonical root, so no synthetic root translation or odometry carries across
Repose acts. `MotionClock` remains the live heading authority.

v7.2 is built directly on v7, not v7.1. The scan primitive and clip-entry ramp
are independent hardware ablations; composing both before either is measured
would make the result uninterpretable. Full build notes and the smoke sequence
are in [v7_2_kinematic_scan.md](v7_2_kinematic_scan.md).

## 7.3 Bounded creep search (v7.3)

v7.2 made each generated turn unreadable, so its empty belief necessarily
selected nominal SETTLE before every next search turn. v7.3 keeps search in
scan-k, bounds every nonzero vision hint to a fixed 22.5 degree act, and uses a
short native Slow Walk creep before asking the same planner for Idle. The
generated walking and stop references are cross-faded before publication, then
a zero-velocity hold admits reads under the unchanged `omega_still` gate:

```text
scan-k CREEP -> generated IDLE -> scan-k QUIET READ
                 | no usable cube -> next scan-k
                 | cube + pose    -> CLIP
                 | target         -> SETTLE / DONE
                 | stale/timeout  -> SETTLE
```

Native Sonic exposes Slow Walk at 0.2–0.8 m/s, Walk at 0.8–1.5, and Run at
1.5–3.0; stick-neutral changes to Idle. v7.3 therefore uses Slow Walk at 0.20
m/s for 0.30 s rather than inventing a 0.05 m/s gait, then mirrors the native
Idle transition and eight-frame animation blend. A 0.14 m cumulative budget
admits roughly two generated 6.2 cm arcs; later turns in the same uninterrupted
search retain the fixed angle but use v7.2's in-place fallback.

This is pure at the state-machine level, not uninterrupted angular motion. A
moving frame still cannot become evidence: colour/depth blur and the lack of
capture-time lowstate alignment make that pose geometrically stale. The
controller naturally holds the last reference row while a bounded extra read
wait runs, so no recursion or controller change is involved.

Each next turn is generated from measured joints with a fresh canonical root.
The released-model regression measures about 20 degrees yaw and 6.2 cm
translation for the default creep/Idle act. Full design and bring-up gates are in
[v7_3_pure_scan.md](v7_3_pure_scan.md).

---

## 8. Bring-up order

| phase | what | gate |
|---|---|---|
| P0 | the wire + `repose_planner_selftest --replay` on recorded reads | labels match the Python oracle; `Sight.pos/.phi` ≤ 1e-4. Settles the palette and depth-realism risks with the robot standing still |
| P0.5 | the **gaze**, `repose_planner_selftest` unit checks | 45.0 ± 0.5° down, \|yaw\| < 0.5°. Free, offline, every run — and it is the whole v7 result, so nothing else is worth measuring until it passes |
| P1 | `retrieve()` picks the same `(row, sym)` as the oracle | offline |
| P2 | closed loop in sim | 40 trials against `sys1_eval.py` |
| P3 | onboard, **settle/scan only** | ψ, reference continuity, the 20 Hz budget under the live loop. Also where `omega_still` gets placed: watch `omega_cam` and `accept_rate` |
| P4 | clip commits on hardware | — |

**The palette is the most fragile piece.** The classifier has no "none" class:
every saturated pixel is forced into one of six chromaticities, and the twelve
refs were measured off the sim renderer. Re-measure under the deployment lights
(P0's replay rig dumps per-candidate stats for exactly this) and keep the floor
neutral, so it fails the saturation gate and never votes.
