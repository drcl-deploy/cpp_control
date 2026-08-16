# custom sonic tracker

deploying our custom-exported SONIC (base or LoRA-adapted) for G1 motion
tracking. one self-contained onnx + one manifest, consumed by `g1_sonic_node`.

## background — how the export works

| stage | where | what |
|---|---|---|
| model | `rsl_rl` fork (`SonicBaseModel` / `SonicWithAdapterModel`) | frozen SONIC release, ported to mjlab MJ joint order (`mocke/scripts/port_sonic_checkpoint.py`); adapter variant straps zero-init LoRA on the decoder |
| graph | `model.as_onnx()` | ONE graph: obs normalizers baked, FSQ folded, **LoRA merged into decoder weights** (no adapter branch at runtime); one named input per obs group |
| exporter | `mocke/scripts/export_sonic_onnx.py` | builds the tracking env, constructs the model directly (no runner/wandb), exports graph + manifest, then an **open-loop parity gate**: torch drives the live env, onnx shadows every step, gated at 1e-5 (measured: base 3.3e-6, zero-adapter 2.4e-6) |
| manifest | `vibe/deploy/onnx_manifest.py` (schema owner) | see below |

```bash
# in the mjlab env
python scripts/export_sonic_onnx.py                      # base    -> g1_sonic_base.{onnx,manifest.json}
python scripts/export_sonic_onnx.py --adapter --rank 16  # 0-init  -> g1_sonic_adapter0.*
```

## the manifest (vibe.onnx.v1)

written as `<model>.manifest.json` next to the onnx AND into the onnx
`metadata_props` — the deploy side reads only these two files, never the
checkpoint. structure:

```
schema        "vibe.onnx.v1"
model_class   SonicBaseModel | SonicWithAdapterModel | ...
checkpoint    provenance (path)
control       { sim_timestep, decimation, step_dt }        # step_dt must equal control_dt
action        { joint_names[29], scale, default_joint_pos,  # THE CHECKPOINT'S joint order
                stiffness, damping }                        # scale may be scalar or per-joint
inputs[]      one entry per onnx input port:
              { name, shape, groups[],
                terms[]: { name, shape, dim, offset,        # exact float layout of the port
                           history_length, flatten_history_dim,
                           source? } }                      # vision terms: encoder/sensor/slice tag
outputs[]     ["actions", ...]                              # + "attn" for extractor exports
versions      { vibe, rsl_rl }  git shas
```

rules the node enforces at startup (fail loud, never mid-run):

1. every manifest port exists in the graph, and graph dim == product(shape)
2. term `offset + dim` never overruns its port; every term name has a
   registered writer (unknown name → error listing the registry)
3. manifest `joint_names` == config joint order (G1 29-dof: unitree_hg motor
   index == mjlab MJ index, verified index-by-index)
4. gains / default pose / action scale are taken **from the manifest** — the
   checkpoint is the authority; the yaml only carries plumbing (topics, dt)

current sonic port/term layout (base; adapter adds `augmentation`):

| port | dim | terms |
|---|---|---|
| `tokenizer` | 640 | `g1_tokenizer` — 10 future frames @ skip 5, per row `[flat-chop(2J) \| pelvis-relative 6D ori]` |
| `policy` | 930 | `base_ang_vel`(30) `joint_pos`(290) `joint_vel`(290) `actions`(290) `gravity_dir`(30) — all 10-frame histories |
| `augmentation` | 290 | `motion_cmd` — future `[jp \| jv]` window, F inferred from dim, skip = `cmd_frame_skip` param |

## joint orderings — read this before loading any clip

the single most common downfall. there are exactly TWO joint orderings in this
pipeline (G1 29-dof), and one permutation between them:

| ordering | layout | who uses it |
|---|---|---|
| **MJ** (MuJoCo XML DFS) | left leg(6) · right leg(6) · waist(3) · left arm(7) · right arm(7) | mjlab, the ported SONIC checkpoint, the manifest, **G1 hardware motors** (unitree_hg index == MJ index, asserted at startup), this node's internals |
| **IL** (IsaacLab BFS) | breadth-first: both hip pitches first, then hip rolls, ... | the retargeted dataset (`motion.npz` joints AND bodies), the original textop WBC |

single source of truth for the permutation: `mocke/mdp/joint_maps.py`
(`IL2MJ`/`MJ2IL`, FK-verified); the node carries the same table baked in
(`G1_IL2MJ` in [g1_sonic.cpp](../../src/tasks/tracker/g1_sonic.cpp)).

**which is my motion.npz?**

| clue | verdict |
|---|---|
| mjlab demo clip (`/tmp/mjlab_cache/*_demo_motion.npz`), 30 bodies | MJ-native → no flag |
| vibe retargeted dataset (`data/retargeted_motions/.../motion.npz`), **37 bodies**, int64 fps | IL → `il_ordered:=true` |
| `jp[0]` slots 0,1 nearly equal (both hip pitches) while slot 3 isn't a knee-sized value | smells IL |

symptoms of getting it wrong (no crash — dims match either way!): robot
"stands cooked", limbs subtly wrong, mild hopping in place, tracking that
never resembles the clip. if it looks drunk, check the ordering first.

exotic sources: pass an explicit `motion_joint_perm` (29 ints, output slot →
source column) instead of `il_ordered` — they are mutually exclusive.

contact bodies: an **adapter** export (`augmentation` port) also commands
`bodywise_contact_cmd` — 12 per-body robot↔object flags read from the clip's
sibling `contact_matrix.npz`, resolved by NAME against its legend (see
[docs/motion.md](../motion.md)). No sibling file → zeros, and the node says so
at engage. This is a joint-order-free channel; nothing to flag.

bodies: the anchor is `anchor_body_index` **in the npz body axis**. pelvis is
index 0 in both IL and MJ body lists, so the default survives both formats;
any other anchor must be looked up per-format (`G1_TRACKED_BODIES` in
joint_maps.py for IL indices).

## build overview

```
cpp_control/
├── include/common/
│   ├── deploy_manifest.hpp     # vibe.onnx.v1 parse (yaml-cpp; JSON ⊂ YAML, no new dep)
│   ├── onnx_session.hpp        # multi-named-port ORT wrapper; buffers + Ort::Values
│   │                           #   built once at load -> zero alloc per tick
│   ├── obs_terms.hpp           # HistoryTerm: mjlab CircularBuffer semantics
│   │                           #   (oldest->newest flatten, backfill on first push)
│   └── motion_clip.hpp         # npz clip (mjlab MotionLoader schema, via vendored cnpy)
│                               #   + MotionPlayback: clock, future-frame clamp, and
│                               #   yaw heading-align at engage (clip comes to the robot)
├── src/common/                 # impls of the above
├── include/cpp_control/tasks/tracker/g1_sonic.hpp
├── src/tasks/tracker/g1_sonic.cpp
│                               # level-2 node over G1Node: binds every manifest term
│                               #   to its port slice by NAME (make_binding registry),
│                               #   verbatim sonic_g1_tokenizer op-chain, engage-reset
│                               #   on policy entry (histories, playback, heading)
├── tests/deploy_selftest.cpp   # unit checks (history/manifest/motion/chop) +
│                               #   smoke-run of any real export: port table + zero-obs
├── config/tracker/g1_sonic.yaml   # plumbing only (topics, dt, fallback gains)
├── launch/g1_sonic_tracker.launch.py
└── thirdparty/cnpy/            # vendored npz reader (MIT, zlib)
```

inference backend: ORT **CPU** EP, single intra-op thread — the full SONIC
graph measures ~1.6 ms/tick on desktop x86 (8% of the 20 ms budget). GPU/TRT
EPs become a session option when a policy actually needs them.

## usage

```bash
# terminal 1 — sim (empty-ground g1 scene)
cd ~/unitree_ros2 && source setup_local.sh
sudo ./unitree_mujoco/simulate/build/unitree_mujoco -i 0 -n lo -r g1

# terminal 2 — tracker
cd ~/unitree_ros2 && source setup_local.sh
source venv/bin/activate            # see workflows/unitree.md#python-venv
cd cyclonedds_ws && source install/setup.bash
ros2 launch cpp_control g1_sonic_tracker.launch.py \
    onnx_path:=/path/to/g1_sonic_base.onnx \
    motion_path:=/tmp/mjlab_cache/lafan1_dance1_subject1_demo_motion.npz

# pre-flight any export without the robot/sim:
./build/cpp_control/deploy_selftest /path/to/policy.onnx
# pre-flight a CLIP: contact schedule + wire round-trip (adapter policies)
./build/cpp_control/deploy_selftest --motion .../cube_frontflip/sample4/motion.npz
```

IL-ordered clips (see [joint orderings](#joint-orderings--read-this-before-loading-any-clip))
need the flag; MJ-native clips don't:

```bash
ros2 launch cpp_control g1_sonic_tracker.launch.py \
    onnx_path:=... motion_path:=.../cube_frontflip/sample4/motion.npz il_ordered:=true
```

joystick: `X` nominal pose → `A` track the clip from frame 0 (yaw-aligns to the
robot at that instant; pressing `A` again restarts) · **`RB`/`R1` stand mode** —
G1's standalone stand-only policy, independent of the SONIC checkpoint and any
vision/motion inputs · `B` zero · `Y` damp.
launch args: `manifest_path` (default: sibling of the onnx), `motion_start_frame`,
`il_ordered`, `config_path`.

note vs mjlab play: deployment cannot teleport the robot to the clip's start
pose — the clip is heading-aligned to wherever the robot stands, so engage from
a posture near the clip's frame 0.

## prep gate (`g1_vibe_sonic`)

`g1_vibe_sonic` enforces that last sentence instead of trusting it. `L1`
deliberately hands control from standalone stand to SONIC prep:

| | stand (`RB`) | prep (`L1`) |
|---|---|---|
| reference | none (stand-only policy obs) | `Motion::lead_in` — T frames, measured handoff pose → selected joints of `jp(motion_start_frame)` |
| root / twist / contact | n/a | identity, zero, zero |
| who balances | `g1::StandPolicy` | SONIC |

so the ramp is **closed-loop**: the policy carries the robot onto frame 0. An
open-loop PD walk to an arbitrary pose is a fall on hardware — the robot has no
balance authority while the setpoint drifts. The lead-in is smoothstepped, with
`jv` computed from `jp` so the two cannot disagree.

T frames, not a poked 1-frame buffer: the tokenizer reads 10 future frames @
skip 5 ≈ 1 s of lookahead, so a real clip lets the policy *anticipate* the ramp
instead of chasing a setpoint it is told is static.

### which joints ramp (`prep_joints`, default `arms`)

Only `prep_joints` chase the clip; every other joint holds its measured handoff
posture. Groups come
from `g1::{LEG,WAIST,ARM}_JOINT_INDICES` (joint_orders.hpp), name-derived from
the MJ table like `MJ2IL` — never written out as ranges.

| `prep_joints` | ramps | pose held until `A` | handover at `A` |
|---|---|---|---|
| `arms` (default) | MJ 15-28 | measured stand stance | arms continuous, **legs+waist step** |
| `arms_waist` | MJ 12-28 | measured stand legs | legs step |
| `all` | MJ 0-28 | the clip's frame 0 | fully continuous |

the default is `arms` because a flip clip's frame 0 is a **dynamic** pose —
passed through with momentum, never held. `all` asks SONIC to balance on it
quasi-statically (CoM near the support edge, weight on toe edges) for as long as
you take to press `A`; that is both OOD and physically marginal, and it is how
you get a slouch-and-fall *before* the run even starts. Under `arms`, the leg
step instead lands at `A`, into a moving in-distribution trajectory whose next
second the tokenizer is already showing — the policy drives the legs there under
momentum, which is what it was trained to do.

`arms` is not universally better: a clip whose frame 0 is stable-but-far (a deep
wide squat) is held fine by SONIC and deserves `all` for the clean handover.
Every prep logs **both** numbers so that call has a number behind it:

```
prep -> frame 0 [arms]: ramp max |dq| 1.42 rad (left_shoulder_roll_joint),
        0.95 s / 47 frames · handover step 0.31 rad (left_ankle_pitch_joint)
```

so under `all` the guarantee is "the lead-in's last frame *is* clip frame 0";
under `arms` it weakens to "the arm reference is continuous, and the leg step is
this big."

flow and refusals:

```
X ──▶ NOMINAL (joint PD)        L1 outside stand  ──▶ "press RB first"
RB ─▶ standalone policy         A without a prep  ──▶ refused, mode unchanged
L1 ─▶ SONIC prep [measured … f0] A after re-staging──▶ "press L1 again"
A ──▶ track   clip from t=0
```

the refusal is `BaseNode::allow_policy_entry()`, checked **before** the mode
flip — vetoing afterwards would mean undoing a mode change blind, which can
strand `DAMPING`/`ZEROING` inside `POLICY`. Every swap of `stand_motion_` also
arms `pending_engage_`, so `active_motion_` can never outlive its clip.

the base `g1_sonic_node` still lets `A` engage directly; only RB ownership moved
to the standalone G1 policy.
