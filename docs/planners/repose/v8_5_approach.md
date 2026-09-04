# Repose v8.5: approach

v8.5 keeps v8 localization and clip retrieval intact and inserts one guarded
robot-anchored walk when the winning clip entry is translationally unreachable.

```text
SETTLE + valid clip candidate
  entry_xy <= 0.25 m                  -> CLIP
  entry_xy > 0.25 m, |bearing| > 20°  -> locked entry turn
  entry_xy > 0.25 m, aligned           -> APPROACH

APPROACH -> SETTLE -> fresh RGB-D belief -> retrieve again
```

The gate is the winning row's translation-only entry residual in the robot
base frame. It is not raw cube range and not total retrieval cost. The latter
also includes heading and exit-horizon terms.

The source is the OGMP walk motion with SHA-256
`5869ef2903b06590a9bd95a47674d30d0df68277dcca81d4afa12b4fdc2db018`.
Windows are selected without scaling. They are sized by net root displacement,
and prefixes whose net travel differs from their initial root heading by more
than 45 degrees are rejected. Approach rows carry recorded root twist and zero
robot-to-cube contact commands.

Safety properties:

- no camera reads during APPROACH;
- a preparatory turn locks the chosen clip row/symmetry but recomputes its
  geometry from each fresh cube pose, preventing arg-min target switching;
- preparatory turns are capped at three attempts and then park as
  `approach turn blocked` instead of cycling;
- no clip is frozen across a walk;
- approach never burns a row or increments the roll counter;
- every leg returns to SETTLE and is judged from a fresh observation;
- at most two legs are allowed per rung;
- measured progress below 0.05 m is logged and repeated failure parks;
- disarm, re-arm and target changes clear approach state.

## Sim2sim smoke test

The local asset symlink currently points at an old checkout name. Point the
process at the active OGMP checkout explicitly:

```bash
source ~/unitree_ros2/setup_local.sh
export VIBE_ASSET_ROOT=/home/loki/drcl_projects/vibe_wip

ros2 launch cpp_control g1_repose_planner_approach.launch.py \
  artifact_dir:=/home/loki/drcl_projects/vibe_wip/logs/rsl_rl/g1_repose_big_cube_floor/wandb_checkpoints/011pgzbh \
  env:=sim
```

Enter POLICY normally, then drag the cube. This launch permits only
SETTLE/SCAN/APPROACH; a candidate inside the entry gate is parked as
`approach ready` and cannot flip.

Watch the terminal or run `ros2 run cpp_control repose_console.py`. The status
reports entry translation/bearing, requested/reference approach distance,
post-settle progress, walk attempts, and the preparatory turn lock/attempt
count. RB remains the immediate disarm.

After approach-only behavior is accepted, closed-loop v8.5 uses the normal
launch with the explicit v8.5 config:

```bash
cfg="$(ros2 pkg prefix cpp_control)/share/cpp_control/config/repose_planner/g1_sim_v8_5.yaml"
ros2 launch cpp_control g1_repose_planner.launch.py \
  artifact_dir:=/home/loki/drcl_projects/vibe_wip/logs/rsl_rl/g1_repose_big_cube_floor/wandb_checkpoints/011pgzbh \
  env:=sim planner_config:="$cfg"
```
