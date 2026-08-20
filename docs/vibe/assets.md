# assets — one path string on every box

Checkpoints and motion data are not tracked by this package, and the left half
of their path differs between the desktop and the Orin. So no config or command
line names it: paths are relative, and one env var says where the tree starts.

## 1. The anchor

`setup.sh` / `setup_local.sh` / `setup_default.sh` resolve the repository from
their own location and export:

| var | value | holds |
|---|---|---|
| `UNITREE_ROS2_ROOT` | the checkout | — |
| `VIBE_ASSET_ROOT` | `$UNITREE_ROS2_ROOT/vibe` | untracked checkpoints + data |

```
unitree_ros2/vibe/
  data/     sys1_clips.npz, sys1_library.npz, retargeted_motions/, ...
  models/   <run>/<export>/{*.onnx,*.manifest.json}
```

Symlink it on the desktop, `rsync` it on the Orin. Only the prefix moves.

## 2. The rule

One function, mirrored in C++ (`include/common/asset_path.hpp`) and in
`launch/g1_vibe_sonic.launch.py`:

```
""            -> ""                       "none" stays "none"
"/..."  "~/..." -> as-is                  absolute always wins
else          -> search, in order:
                   $VIBE_ASSET_ROOT/<p>       untracked, rsync'd per box
                   <share>/cpp_control/models/<p>   tracked, ships with the repo
```

| outcome | result |
|---|---|
| one root has it | that path |
| no root has it | throws, naming every root tried |
| two roots have it | throws — silent shadowing reads as tuned, behaves as stock |

Two roots because there are two kinds of asset. Packaged policies
(`onnx_path: locomotion/g1.onnx`) already travelled with `git clone` and keep
working untouched; anything under `vibe/` is yours.

## 3. Naming an artifact

`artifact:=<run>[/<export>]` searches `$VIBE_ASSET_ROOT/models/<run>` for a
directory holding an `.onnx`/`.manifest.json` pair — at either depth, so a run's
own layout (`wandb_checkpoints/<id>`, a timestamp, flat) does not matter.

```bash
ros2 launch cpp_control g1_vibe_repose.launch.py artifact:=g1_repose_big_cube_floor/011pgzbh
ros2 launch cpp_control g1_vibe_perloco_omre.launch.py artifact:=vibe_perloco_omre
```

| you type | it means |
|---|---|
| `artifact:=<run>/<export>` | that export |
| `artifact:=<run>` | its only export, else an error listing them |
| `artifact_dir:=<path>` | that directory, relative or absolute |
| both | an error — pick one |

`checkpoint:=<step>` still disambiguates *within* an export directory that holds
several pairs.

## 4. Where it applies

| surface | resolved by |
|---|---|
| `clips`, `frames.path`, `frames.library` | `src/planners/repose/{node,observe_node}.cpp` |
| `onnx_path`, `hlc_onnx_path`, `policy_path`, `motion_path`, `stand_onnx_path` | `src/config_loader.cpp` |
| `artifact`, `artifact_dir`, `motion_path` on the command line | `launch/g1_vibe_sonic.launch.py` |

## 5. When it bites

| symptom | cause |
|---|---|
| `is relative and no asset root is set` | you did not source a setup script |
| `is under none of: ...` | the file is not rsync'd, or the name is wrong |
| `is ambiguous — it exists under: ...` | the same relative name under both roots; rename one |
