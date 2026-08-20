# cpp_control

modular, multi-robot, multi-workflow C++ controller deployment for ROS2
(humble). design, figures and rationale: [docs/ethos.md](docs/ethos.md).

## install

1. workflow setup, incl. the python venv:
   [workflows/unitree.md](workflows/unitree.md) /
   [workflows/drcl_deploy.md](workflows/drcl_deploy.md)
2. onnxruntime — [pre-built v1.22](https://github.com/microsoft/onnxruntime/releases/tag/v1.22.0),
   symlinked under [thirdparty](./thirdparty/):
   ```bash
   ln -s /PATH/TO/onnxruntime-linux-x64-1.22.0 ./cpp_control/thirdparty/
   ```
3. `colcon build --symlink-install --packages-select cpp_control`
4. CLI shims (`publish-motion`), one-time:
   `uv pip install -e cyclonedds_ws/src/cpp_control --python venv/bin/python`

## usage

```bash
# G1 locomotion
ros2 launch cpp_control g1_locomotion.launch.py

# G1 SONIC tracker (exported vibe policy, manifest-driven)
ros2 launch cpp_control g1_sonic_tracker.launch.py \
    onnx_path:=/path/to/policy.onnx motion_path:=/path/to/motion.npz

# hardware workflow (sonic + vibe-sonic): controller never dies —
# boot with no clip (stand), stream motions, A starts each one
ros2 launch cpp_control g1_sonic_tracker.launch.py motion_path:=''
publish-motion /path/to/motion.npz     # stage -> press A -> track -> RB -> repeat

# G1 vibe-SONIC (vision): each task launch owns its encoder + controller
ros2 launch cpp_control g1_vibe_repose.launch.py artifact_dir:=/path/to/checkpoint
ros2 launch cpp_control g1_vibe_uolm.launch.py artifact_dir:=/path/to/checkpoint
ros2 launch cpp_control g1_vibe_perloco_grail.launch.py artifact_dir:=/path/to/checkpoint
ros2 launch cpp_control g1_vibe_perloco_omre.launch.py artifact_dir:=/path/to/checkpoint
ros2 launch cpp_control g1_vibe_dodge.launch.py artifact_dir:=/path/to/checkpoint
publish-motion /path/to/motion.npz       # Repose / UOLM / PerLoco only

# closed-loop Repose: the same runtime, driven by the repose planner
ros2 launch cpp_control g1_repose_planner.launch.py artifact_dir:=/path/to/checkpoint
bash cyclonedds_ws/src/cpp_control/scripts/planners/repose/repose_run_console.sh   # target colour, live

# offboard from a fresh terminal at the unitree_ros2 repository root
bash cyclonedds_ws/src/cpp_control/scripts/vibe/stream_vibes.sh
bash cyclonedds_ws/src/cpp_control/scripts/vibe/stream_vibes.sh record
bash cyclonedds_ws/src/cpp_control/scripts/vibe/replay_vibes.sh [BAG_DIRECTORY]
# the typed reference independently carries root twist, per-body contact, and
# UOLM's sibling object_motion.npz goal; missing channels stay explicit

# deploy infra self-test (+ optional export smoke / clip contact report)
./build/cpp_control/deploy_selftest [policy.onnx [policy.manifest.json]]
./build/cpp_control/deploy_selftest --motion /path/to/motion.npz

# MiniPi
ros2 launch cpp_control mini_pi_locomotion.launch.py
ros2 launch cpp_control mini_pi_dive.launch.py
```

buttons (joy / unitree gamepad): `X` nominal pose · `A` policy · `B` zeroing ·
`Y` damping · `RB/R1` stand.

**prep gate** (Repose, UOLM, PerLoco): `RB` stand → `LB/L1` → `A`. `LB` swaps
stand's nominal reference for a ramp onto the clip's first frame — SONIC keeps
balancing, only the reference moves — and `A` is refused until it lands, since
the clip engages at t=0 and a robot off frame 0 gets a step input. `prep_joints`
picks what ramps: `arms` (default), `arms_waist`, `all`; anything else holds
nominal, so SONIC never has to balance on a dynamic keyframe while you wait.
Duration is `clamp(max|dq| / prep_rate, prep_min_s, prep_max_s)`. Each prep logs
the ramp *and* the residual handover step — see
[docs/trackers/custom_sonic.md](docs/trackers/custom_sonic.md#which-joints-ramp-prep_joints).
`RB` aborts, `X`/`B`/`Y` void it, a finished clip voids it — re-press `LB` per run.

**stand mode**: set `stand_onnx_path` in any g1 config yaml to a base-SONIC
export and `RB/R1` runs an actively-balancing SONIC stand
(`ControlMode::STAND`) instead of a task-owned one — no task code involved.

## docs

| doc | what |
|---|---|
| [docs/ethos.md](docs/ethos.md) | design: backends, four levels, control modes, structure, how to extend |
| [docs/onnx_policies.md](docs/onnx_policies.md) | the two ONNX serving stacks (legacy vs manifest) and when to use which |
| [docs/motion.md](docs/motion.md) | `g1::Motion` — the one reference-motion class |
| [docs/trackers/custom_sonic.md](docs/trackers/custom_sonic.md) | vibe.onnx.v1 manifest contract, export flow, tracker usage |
| [docs/vibe/experiments.md](docs/vibe/experiments.md) | real/simulation experiment commands, viewing, and recording |
| [docs/vibe/assets.md](docs/vibe/assets.md) | `$VIBE_ASSET_ROOT`: one path string for checkpoints and data on every box |
| [docs/vibe/tasks.md](docs/vibe/tasks.md) | shared Vibe runtime, task contracts, launch and reference transport |
| [docs/vibe/background.md](docs/vibe/background.md) | deployment constraints, reference lessons, and design decisions |
| [docs/vibe/roadmap.md](docs/vibe/roadmap.md) | measured hardening priorities and explicitly deferred work |
| [docs/planners/repose/planner.md](docs/planners/repose/planner.md) | the retrieval planner above SONIC — closed-loop Repose, `planner:=true` |
| [docs/planners/repose/experiments.md](docs/planners/repose/experiments.md) | the closed-loop run: depth, launch, console, what a bag carries |
| [docs/planners/repose/v7_simplified.md](docs/planners/repose/v7_simplified.md) | the OBSERVE/PLAN/ACT split — parity matrix, what was deleted, block diagram |
| [docs/planners/repose/color_calibration.md](docs/planners/repose/color_calibration.md) | RGB-D palette collection: 30 clicks/colour + 60 negatives |
| [docs/planners/repose/observe_calibration.md](docs/planners/repose/observe_calibration.md) | the observe block's theory: masks, blobs, chromaticity, the two reads |

## acknowledgements

this package deploys policies trained with, and stands on the ideas of:

- [beyondmimic](https://github.com/HybridRobotics/motion_tracking_controller) — motion tracking controller
- [textop-tracker](https://github.com/TeleHuman/Textop) — text-conditioned whole-body tracking
- [SONIC](https://github.com/NVlabs/GR00T-WholeBodyControl) — whole-body control
