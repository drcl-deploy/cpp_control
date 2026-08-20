# Repose v7.2 — kinematic stepping SCAN

v7.2 is one focused hardware experiment over v7: replace the stationary
reference sweep used by SCAN with a GEAR-SONIC Walk-mode stepping turn. Clip
retrieval, belief, perception, ladder state, nominal SETTLE, and the downstream
Vibe-SONIC controller are unchanged. REACH is deliberately out of scope.

## Why this command

The released planner treats a locomotion mode with a positive speed and a zero
movement vector as an in-place stepping-turn request. The facing vector is a
fixed target, not a commanded angular velocity:

```text
mode 2 (Walk), speed 0.10 m/s
movement [0, 0, 0]
facing   [cos(yaw_offset), sin(yaw_offset), 0]
```

The standalone joystick wrapper does not exercise this condition: neutral
stick changes Walk to Idle. Direct model probes from the nominal context gave:

| target | generated yaw | net translation |
|---:|---:|---:|
| 22.5 deg | 20.4 deg | 2.1 cm |
| 45 deg | 40.3 deg | 1.8 cm |
| 90 deg | 80.5 deg | 1.1 cm |

Those are reference properties, not claims about tracking on the robot.

## State and safety

`KINEMATIC_SCAN` is an unreadable moving act, like CLIP. Its commit clears the
belief and `observe()` excludes it even if the camera-rate gate happens to
open. When it finishes, the empty belief produces nominal SETTLE. This makes
the only legal evidence path:

```text
KINEMATIC_SCAN -> SETTLE -> quiet camera -> belief -> next decision
```

The generator is reinitialized from measured joints on every scan. Root yaw
and translation are canonical; the existing controller `MotionClock` aligns
frame zero to the live robot. A dropped reference republishes cached bytes and
never reruns stochastic inference. An inference failure falls back to SETTLE.

The Vibe-SONIC controller remains the only `/lowcmd` owner and the Repose node
remains the only `/tracker/reference` publisher. The base SONIC policy and the
standalone kinematic planner node are not launched.

## Select and run

The v7.2 preset remains available as the scan-k/SETTLE control; the sim config
now selects its v7.3 successor. The tuned real config also selects v7.3 for the
requested hardware trial; set its one `version:` line to v7.2 for this control.
The ONNX is ignored by Git and must already have been installed with:

```bash
cd cyclonedds_ws/src/cpp_control
scripts/planners/setup_sonic_kinematic.sh
```

Then run the normal Repose launch:

```bash
ros2 launch cpp_control g1_repose_planner.launch.py \
  artifact:=<run>/<export> env:=sim
```

`kinematic_model:=/absolute/planner_sonic.onnx` overrides the packaged model.
Setting `version: v7` restores the old stationary sweep without changing code.
Setting `version: v7.2` restores the stepping scan with mandatory SETTLE.

## Acceptance ladder

1. Model smoke: all 27 modes plus stepping-turn yaw/translation semantics.
2. Sim: force a rejected/pose-less cube read and observe
   `settle -> scan-k -> settle`, with one reference ID per act.
3. Suspended hardware: request +/-22.5, +/-45, then +/-90 degrees; record
   requested/achieved IMU yaw, foot slip, reference translation, and RB stop.
4. Floor hardware: verify the post-scan SETTLE restores the nominal camera gaze
   and only then increases `n_reads`.
5. Disconnect: stale lowstate prevents new acts; RB during generation prevents
   the controller from accepting the result.

Go/no-go is physical tracking, not ONNX output: yaw must have the correct sign,
the feet must step rather than wind up, translation must remain bounded, and
the 50 Hz controller must keep its deadline while CPU inference runs.
