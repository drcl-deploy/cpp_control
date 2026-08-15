# sys1 — the retrieval planner above SONIC

Given a target colour, sys1 looks at the cube, picks a recorded quarter-turn clip
that reaches it from where the robot actually stands, and writes it into sys0's
reference. Three modes, one state integer, one ranking scalar.

Reference implementation: vibe `src/vibe/repose/planner/clips.py` (algorithm) and
`command.py` (the sim seam). Behaviour spec: vibe `docs/sys1_v5.md`. Migration
spec: vibe `docs/sys1_cpp.md` — **read §1 below before that one**, four of its
structural requirements do not apply to this stack.

**Off by default.** `sys1:=false` is bit-for-bit the open-loop rig.

---

## 1. Four things this stack does not need

The migration spec was written against the sim's data model. cpp_control's SONIC
seam is narrower, and that deletes most of it.

| # | in the sim | here |
|---|---|---|
| F1 | cube-exact SE(2) warp of 19852 frames | **one scalar.** `fill_tokenizer` reads `jp`, `jv` and root ORIENTATION only; `motion_cmd` reads `jp`/`jv`; the adapter's twist is body-frame. Root POSITION reaches no observation, so the translation half of the warp is unobservable and `_pristine` disappears |
| F2 | `robot_yaw`, `root_xy`, odometry drift budget | **nothing.** The tokenizer's 6D is `qinv(imu) · ref_quat` and sys1's yaw comes from the same IMU, so the IMU *is* the world frame and drift cancels in every difference |
| F3 | a 10-slot ring buffer, rewritten every step | **`MotionClock`.** `future_frame(k)` already yields `t + [0,5,…,45]` clamped to the clip end — same `FUTURE_STEPS`/`FRAME_SKIP`. A still mode is published as a clip whose frames happen to be identical apart from the yaw ramp |
| F4 | a 12-frame blend at commit | **a rate-limited lead-in.** A reference that steps is a step input to a balancing policy. `lead_in_rate` walks there instead — the same mechanism the human-driven L1 prep provides, which is why the prep gate turns off under sys1 |

What survives of the warp is `entry_yaw_offset`, and it is exactly the heading
term the retrieval minimises:

```
entry_yaw = wrap(qth[row] + sym·π/2 + φ)          # the plan
dth       = wrap(qth[row] + sym·π/2 − (−φ))       # _se2's heading residual
```

`sys1_selftest` asserts they are the same number for all four symmetries — rank
one thing, command another, and the planner is lying to itself.

## 2. Two rates, no clock

sys1 owns no clock. Every mode is timed off `Sys0Status`, so what the robot is
ACTUALLY playing is the only thing that advances the plan — and a human pressing
`B` mid-clip parks the planner instead of leaving it talking to itself.

```
[50 Hz HARD RT]  /lowstate -> g1_vibe_sonic_node -> /lowcmd
                     |  reads MotionReference, plays it verbatim
                     +-> /vibe/sys0/status ---------------+
                                                          v
[20 Hz SOFT RT]  camera TCP -> g1_sys1_repose_node -> /tracker/reference
                     ^ /lowstate (FK + IMU), /vibe/sonic/goal_color
                     +-> /vibe/sys1/status -> scripts/sys1_console.py
```

Budget: `classify` is the only hot spot (12 squared distances per live pixel at
160×120, <1 ms), everything else <0.3 ms, the warp is ~50 µs. **~2% of one core.**
Clip spans resident: 3.5 MB.

## 3. The loop

```
tick():                                       # 20 Hz
    if !sys0.accepting:            park, disarm        # the human owns the robot
    if !armed:                     arm, reset, commit(SETTLE)
    if sys0.reference_id != mine:  resend after 3 ticks
    if mode == CLIP:               if sys0.finished -> commit(SETTLE)
    else:
        if frame >= frames - 2·read_tail:  look()      # camera stationary
        if sys0.finished:                  commit(decide())

decide():                                     # consumes the LATEST look
    tip:   colour changed since the last consumed read -> ladder advances
    done:  colour == target                   -> SETTLE
    !ok:   no pose                            -> SCAN, yaw = clamp(hint, ±sweep)
    else:                                     -> CLIP = argmin over pool × 4 syms
```

Two deliberate departures from the Python:

1. **The tip is counted on the read `decide()` consumes**, not on every read.
   The sim reads five times per still mode and each call could increment; a
   flickering read advanced three rungs from one still mode (`sys1_cpp.md` §8.5).
   Never fires in sim, will fire on hardware.
2. **A SCAN commands a turn rate.** `robot_root_ang_vel_cmd` carries the sweep
   rate while the ramp runs; zero would lie to the adapter for the whole turn.

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

```bash
# open-loop, unchanged
ros2 launch cpp_control g1_vibe_repose.launch.py artifact:=/path/model_58000.onnx

# closed-loop
ros2 launch cpp_control g1_sys1_repose.launch.py artifact:=/path/model_58000.onnx
ros2 run cpp_control sys1_console.py     # 0-5 or r/o/g/y/b/p sets the target
```

`/vibe/sonic/goal_color` is read by BOTH the planner and the policy's
`object_goal_color` one-hot — one topic, so they cannot disagree about what "4"
means. The index order is `vibe::cube_color_name`: 0 red, 1 orange, 2 green,
3 yellow, 4 blue, 5 pink.

## 7. What `sys1:=true` changes in sys0

| | false (default) | true |
|---|---|---|
| references | staged, `A` commits | auto-commit on arrival |
| re-engage | resets observation histories | keeps them (a swap every ~2 s would starve every history term) |
| L1 prep gate | on | off — the lead-in replaces it |
| `Sys0Status` | not published | 20 Hz, its own timer |
| `entry_yaw_offset` | 0 from every publisher, so inert | applied at engage |

## 8. Bring-up order

| phase | what | gate |
|---|---|---|
| P0 | the wire + `sys1_selftest --replay` on recorded reads | labels match the Python oracle; `Sight.pos/.phi` ≤ 1e-4. Settles the palette and depth-realism risks with the robot standing still |
| P1 | `retrieve()` picks the same `(row, sym)` as the oracle | offline |
| P2 | closed loop in sim | 40 trials against `sys1_eval.py` |
| P3 | onboard, **settle/scan only** | ψ, reference continuity, the 20 Hz budget under the live loop |
| P4 | clip commits on hardware | — |

**The palette is the most fragile piece.** The classifier has no "none" class:
every saturated pixel is forced into one of six chromaticities, and the twelve
refs were measured off the sim renderer. Re-measure under the deployment lights
(P0's replay rig dumps per-candidate stats for exactly this) and keep the floor
neutral, so it fails the saturation gate and never votes.
