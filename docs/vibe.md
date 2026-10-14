# ViBe integration

One `G1VibeSonicNode` combines robot state, reference motion and image tokens
from `vision_encoders`. The policy manifest defines ports, term order, history,
action mapping and timestep. A small task profile supplies lifecycle and command
semantics. See [architecture](ethos.md) and [policy serving](onnx_policies.md).

## launch and artifacts

```bash
ros2 launch cpp_control g1_vibe_sonic.launch.py \
  policy_path:=/path/to/policy.onnx encoder_path:=/path/to/encoder.onnx \
  motion_path:=/path/to/motion.npz
```

Each ONNX file discovers its same-stem `.manifest.json`; `policy_manifest` and
`encoder_manifest` override those locations. Absolute paths work directly.
Relative paths resolve under `VIBE_ASSET_ROOT` or packaged model directories;
missing or ambiguous assets fail instead of silently selecting another model.
In vibe_ws, `VIBE_ASSET_ROOT` points to `artifacts/`.

The older `artifact:=<run>[/<export>]` and `artifact_dir:=<directory>` selectors
remain available. Multiple pairs in a directory require `checkpoint:=<step>`.
Do not combine those selectors with `policy_path`.

| Wrapper | Manifest family | Reference / command |
|---|---|---|
| `g1_vibe_walk.launch.py` | `perloco` (Grail exports) | Motion with root twist |
| `g1_vibe_parkour.launch.py` | `perloco` (OmRe exports) | Motion with root twist |
| `g1_vibe_repose.launch.py` | `repose` | Motion and goal up-face color |
| `g1_vibe_uolm.launch.py` | `uolm` | Motion, contacts and final object pose |
| `g1_vibe_dodge.launch.py` | `dodge` | Stand-reactive; no motion or prep |

Wrappers delegate to the generic launch and check the family. `walk`/`parkour`
are user-facing names; existing exported task IDs remain unchanged. Keep the
camera and encoder contract matched to the chosen export. The four released
samples and their camera setup are documented in
[vibe_ws](https://github.com/drcl-deploy/vibe_ws#usage).

## references and external planners

`cpp_control/msg/MotionReference` carries the full reference with independent
`has_twist`, `has_contact` and `has_object_goal` flags. UoLM's object goal comes
from the sibling `object_motion.npz` and stays in the canonical clip frame.
`ControllerStatus` reports playback and operator state. See [motion](motion.md)
for the schema, heading alignment and `publish-motion` setup.

A planner runs separately and consumes/publishes those controller interfaces.
With `planner:=true`, the tracker also subscribes to
`/vibe/planner/heartbeat` (`std_msgs/msg/Empty`). A fresh heartbeat is required
to arm. Once seen, a gap of one second shuts down the tracker; this is process
shutdown, not an active damping command. Publish the heartbeat from the actual
planner tick so a stalled planning loop cannot appear healthy.

Repose's `PlannerStatus`, `CubeObservation` and `CalibrationState` belong to
[sledgehammer](https://github.com/drcl-deploy/sledgehammer). cpp_control has no
build dependency on that planner. Detailed planner status is telemetry, not the
controller's heartbeat. Start Repose through `sledgehammer planner.launch.py`;
the complete working example is in the workspace README.

Motion tasks use **RB** stand → **L1** prep → **A** run; **Y** selects damping.
Dodge uses **A** or **RB** with its nominal reference. Prep and active stand
behavior are described in the [tracker guide](trackers/custom_sonic.md).

## viewing and offboard builds

The generic launch starts the encoder, tracker and optional bridge server.
`enable_bridge:=false` allows a separately launched server. A bridge exit or
client disconnect leaves control running; launch Ctrl-C closes its children.
Viewing scripts use the selected ROS distribution and OS Python, independently
of the setup venv. Source the workspace environment before running them.

For messages and tools without ONNX/CUDA, configure the packages with
`CPP_CONTROL_BUILD_RUNTIME=OFF`, `VISION_ENCODERS_BUILD_RUNTIME=OFF`, and
`SLEDGEHAMMER_BUILD_RUNTIME=OFF`. The workspace installer selects these for
`--profile offboard`. Sledgehammer messages are needed to view planner telemetry;
the planner executable is not needed offboard.

The bundled replay tools read current bags and the pre-migration
`cpp_control/msg/PlannerStatus`, `CubeObservation` and `CalibrationState` layouts.
This compatibility is in the readers; old ROS type names are not republished.
For raw `ros2 bag play` of old types, retain the matching older workspace.

Validation commands and hardware acceptance checks are maintained in
[vibe_ws testing](https://github.com/drcl-deploy/vibe_ws/blob/main/docs/testing.md).
