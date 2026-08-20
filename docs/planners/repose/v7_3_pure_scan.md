# Repose v7.3 — bounded creep search

v7.3 keeps the pure scan-k state machine introduced over v7.2 and makes its
hardware turn a repeatable locomotion act. A missing cube no longer sends the
robot through nominal SETTLE, and perception can no longer jump the kinematic
planner between 5.625 and 90 degree requests. Every nonzero hint contributes
only its sign; the configured turn magnitude is fixed at 22.5 degrees.

## The loop

```text
scan-k CREEP -> generated IDLE -> scan-k QUIET READ
                 | no colour/pose -> next scan-k
                 | cube + pose    -> CLIP
                 | target colour  -> SETTLE / DONE
                 | stale camera   -> SETTLE
                 | no quiet reads -> SETTLE after bounded timeout
```

The first acts in an uninterrupted search briefly use Slow Walk at 0.20 m/s,
the lowest speed exposed by Sonic's native gamepad deployment. The movement
direction follows the target heading, as a forward stick does upstream. After
0.30 s the same ONNX planner is asked for Idle from the walking reference and
the two motions are joined with Sonic's native eight-frame cross-fade. This is
a learned stop; the node does not truncate a gait or freeze a mid-stride frame.

The cumulative generated translation is budgeted. With the released model the
default arc is about 6.2 cm, so a 0.14 m burst budget admits roughly two creep
acts. Further turns remain fixed at 22.5 degrees but use the v7.2 zero-vector
in-place fallback until scan-k ends. Any non-scan act resets the budget.

The quiet read is intentional. Frames above `belief.omega_still` remain blur,
not evidence, and the camera pose uses current lowstate rather than a
capture-time history. Truly uninterrupted rotation would therefore either
remain blind forever or require weakening the geometry contract.

The generated creep/Idle motion gets a `kinematic_scan.read_tail_s` final-pose hold. Its
joint positions, root pose, and contact remain the planner's final values;
joint velocity and root twist are zero. If the physical camera has not settled
by reference completion, the controller naturally keeps holding that row for
up to `read_timeout_s`. The mode and reference remain `scan-k` throughout.

Three accepted no-cube reads satisfy the evidence quota and directly request
the next generated turn. No new decision branch or recursive call is needed:
the existing `Clips::decide()` already maps sufficient but invalid evidence to
SCAN. Camera loss produces no reads, remains BLIND, and therefore retains the
existing nominal-SETTLE fail-safe.

## Configuration

The sim config selects v7.3:

```yaml
version: v7.3
kinematic_scan:
  random_seed: 1234
  turn_deg: 22.5
  creep_s: 0.30
  creep_budget_m: 0.14
  stop_blend_frames: 8
  read_tail_s: 0.60
  read_timeout_s: 1.00
```

Mode and speed are preset-owned so switching one `version:` line remains a
clean A/B: v7.2 retains Walk/0.10 with zero movement, while v7.3 uses Slow
Walk/0.20 for its native creep. `creep_s: 0` disables creep and is the direct
bounded in-place control. The upstream gamepad ranges are Slow Walk 0.2–0.8,
Walk 0.8–1.5, and Run 1.5–3.0 m/s; v7.3 validation refuses a creep outside the
selected mode's native range.

`v7.2` is the mandatory-SETTLE control and `v7` is the established stationary
SCAN baseline. Both sim and real configs select v7.3 for this hardware trial.
The model remains ignored by Git and is installed with:

```bash
scripts/planners/setup_sonic_kinematic.sh
```

Run the normal stack and console:

```bash
ros2 launch cpp_control g1_repose_planner.launch.py \
  artifact:=<run>/<export> env:=sim
ros2 run cpp_control repose_console.py
```

## Acceptance gates

1. With no cube visible, active mode stays `scan-k` while reference IDs advance;
   there is no intervening active SETTLE.
2. Every scan-k log reports `yaw +22.5` or `-22.5`; no 5.625/90 degree jump
   remains.
3. The first two default acts report `native creep -> Idle` and cumulative
   translation below 0.14 m; later acts report `in-place fallback`.
4. `n_reads` grows only in the quiet tail and `omega_cam` is below
   `omega_still` for every accepted read.
5. A placed cube transitions from `scan-k` directly to the selected clip.
6. The target colour transitions to nominal SETTLE and latches DONE.
7. Camera loss or a camera that never becomes quiet transitions to SETTLE.
8. RB still disarms immediately; stale lowstate prevents new acts.

The released-model regression for the native 0.30 s creep plus generated Idle
produces about 20 degrees of yaw and 6.2 cm of reference translation. These are
reference properties; physical policy tracking, foot clearance, and the
generated final camera gaze remain the deciding hardware tests.
