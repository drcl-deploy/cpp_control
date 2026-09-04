# Hardware `cut_face` scan loop

## Incident

- Date: 2026-09-02
- Launch environment: `env:=real`
- Symptom: the top face was consistently identified as pink/magenta, but
  `pose_ok` stayed false and the planner kept scanning instead of selecting a
  clip.
- Failing run: [02Sep2026_13_57_0.db3](/home/loki/unitree_ros2/bags/02Sep2026_13_57/02Sep2026_13_57_0.db3)

The run looked like a scan/settle loop in the UI. In the recorded decisions it
was one `settle` commit followed by 18 consecutive `scan` commits; no clip was
committed.

## What the bag proves

The 95.1 s bag contains 1,901 planner-status samples:

| Signal | Result |
|---|---:|
| `pose_ok` | 0 / 1,901 |
| `sees` | 1,434 / 1,901 |
| `cut_face` | 1,626 samples |
| `blind` | 245 samples |
| `no_blob` | 30 samples |
| commits | 1 settle, 18 scans, 0 clips |

Pink was the dominant, unanimous colour vote during the active scans. Accepted
reads had median angular speed `0.073 rad/s`, below `omega_still: 0.15`, and
most scans collected 4--18 reads. The failure was therefore neither colour
classification nor a shortage of still-enough observations.

The pink region touched the bottom image boundary in every active encoder frame
examined. A yaw-only scan moves the cube horizontally but cannot recover the
missing vertical part of its top face.

## Why it repeats

For the hardware `mask` reader, visible fraction is approximately

```text
vis = min(top-plane rectangle width, height) / expected cube edge
```

The current gates in
[`g1_real.yaml`](../../../config/repose_planner/g1_real.yaml) are:

```yaml
observe:
  geometry:
    min_visible: 0.60
    min_visible_color: 0.0
```

Consequently, a partial face may supply a valid colour while failing the pose:

```text
pink colour accepted
  -> vis < 0.60
  -> reason = cut_face, pose_ok = false
  -> planner chooses scan
  -> scan changes yaw only
  -> face remains vertically cropped
  -> cut_face again
```

The exact checks are in
[`sight.cpp`](../../../src/planners/repose/sight.cpp), and the scan decision is
in [`clips.cpp`](../../../src/planners/repose/clips.cpp).

The standard planner bag records the verdict but not the numeric `vis` value,
so it proves that `vis < 0.60` on every accepted candidate, not how far below
the threshold it was.

## September 3 evidence and v8

The labelled nominal-stand corpus is
[`03Sep2026_17_48`](/home/loki/unitree_ros2/bags/repose_observe/03Sep2026_17_48).
It contains 84 final captures: 72 cube frames at four approximate ranges and 12
negative controls. Its effective onboard config was the checked-in
`g1_real.yaml` with only `min_visible: 0.60 -> 0.40`.

Reconstruction of the production `mask` stages showed the actual fault:

```text
labels >= 0 AND valid depth
  -> top-height band
  -> largest component
  -> min-area rectangle
```

Thus `mask` was still colour-seeded. D435i depth fringe plus rejected/desaturated
RGB pixels fragmented the input before geometry existed. Lowering
`min_visible` changed the final gate but could not restore the missing support.

v8 changes only `CubeSight`:

```text
all valid depth
  -> robust floor plane
  -> band one known cube edge above the floor
  -> close depth islands
  -> constrained 0.6096 m square fit
  -> RGB vote inside that square
```

The planner ladder, scan/settle behavior, clip retrieval, controller, and
reference motions are unchanged. Walking is explicitly deferred.

Production C++ replay on the corpus:

| read | `pose_ok` on cube | joint correct colour + pose | false colour on negatives |
|---|---:|---:|---:|
| hardware `mask`, `min_visible: 0.40` | 67/72 | 54/72 | 0/12 |
| v8 `plane`, `min_visible: 0.70` | **71/72** | **56/72** | **0/12** |

Pink remains 12/12 for colour and pose. The one remaining pose rejection is a
genuinely partial yellow face (`cut_face`), not colour-mask starvation.

## Safe hardware acceptance test

After building onboard, start the inert localization launch:

```bash
source ~/unitree_ros2/setup.sh
ros2 launch cpp_control g1_repose_planner_localization.launch.py \
  artifact:=g1_repose_big_cube_floor/011pgzbh env:=real
```

It holds nominal stand under `calibration_lock`; it does not start the planner.
Record and view offboard:

```bash
source ~/unitree_ros2/setup.sh
ros2 run cpp_control repose_check_observe.sh record
```

Bags land under `bags/repose_localization/`. When that evidence is healthy, run
the full planner with an explicit acceptance config:

```bash
ros2 launch cpp_control g1_repose_planner.launch.py \
  artifact:=g1_repose_big_cube_floor/011pgzbh env:=real \
  planner_config:="$(ros2 pkg prefix cpp_control)/share/cpp_control/config/repose_planner/g1_real_v8.yaml"
```
