# motion — the one reference-motion class

`common/g1/motion.hpp`: every tracker reads its reference motion through
`g1::Motion` + `g1::MotionClock`. No per-node parallel buffers, no per-tracker
loaders — one container, one clock, tracker-local "massaging" only.

```
npz / wire / stand  ──>  g1::Motion  ──views──>  jp/jv (MJ), jp_il/jv_il (IL),
                             │                   root pose, root twist (ref frame)
                             └── g1::MotionClock: the one t + heading alignment
```

## sources

| factory | input | twist | contact | used by |
|---|---|---|---|---|
| `from_npz(path, perm)` | retargeted-dataset / mjlab npz (`joint_pos`, `joint_vel`, `body_{pos,quat,lin_vel,ang_vel}_w`, `fps`) + sibling `contact_matrix.npz` — mocke `MjMotionLoader` twin; `perm = g1::MJ2IL` for IL-ordered clips | from file | from file | sonic, vibe-sonic |
| `from_wire(T, rows, cols)` | legacy motion topic rows, IL-ordered, 65 or 83 wide | 83 only | 83 only | textop (65), sonic, vibe-sonic |
| `from_wire(T, rows, 83, flags, fps)` | versioned `MotionReference`; full storage with independent twist/contact validity | explicit flag | explicit flag | sonic, vibe-sonic |
| `stand(defaults)` | nominal pose, identity anchor | zeros (truth) | zeros (truth) | every stand mode |

**The wire**, two widths (`g1::WIRE_COLS_{MIN,FULL}`, written by
`scripts/npz_motion_publisher.py`):

```
 65  jp29 | jv29 | anchor_pos3 | anchor_quat4                          (textop era)
 83  ...  | anchor_lin_vel3 | anchor_ang_vel3 | contact12              (+ sys1 cmds)
```

The publisher emits 83 iff the clip has body twist **and** a
`contact_matrix.npz`. A 65-wide clip streams an adapter policy's whole
augmentation port as zeros — legal, loud, and never what you want for
vibe-sonic.

Storage is **MJ-canonical** (hardware motor order); IL comes out of views.
Joint tables live in `common/g1/joint_orders.hpp`, built from the name lists
at static init: `v_mj[i] = v_il[MJ2IL[i]]`.

## views

| accessor | returns | frame |
|---|---|---|
| `jp(f)` / `jv(f)` | `const float*` row | MJ order |
| `jp_il(f, out)` / `jv_il(f, out)` | gather into caller buffer | IL order |
| `root_pos(f)` / `root_quat(f)` | anchor-body pose | world |
| `root_lin_vel_b(f)` / `root_ang_vel_b(f)` | anchor twist | **ref-anchor frame** |
| `contact(f)` | `const float*` 12 flags | `CONTACT_GRAPH_BODIES` order |

The `_b` twists are the sys1 command stream
(orcs `robot_root_{lin,ang}_vel_cmd`): `R(q_ref_anchor)ᵀ · v_world`, a pure
clip function. Rotating quat and vel together under a heading rebase cancels —
so they never touch live robot state and need no alignment.

`contact(f)` is the third sys1 channel (orcs `bodywise_contact_cmd`): per-body
robot↔**object** contact, mirroring `ContactSchedule.body_object_contacts`.

| | |
|---|---|
| source | `contact_matrix.npz` — `(T, N, N)` int8 + a `body_names` legend |
| object | the legend's **last** column (name varies per scene) |
| columns | `g1::CONTACT_GRAPH_BODIES`, resolved by **name** — the legend has 32 bodies, `motion.npz` 37, so positional alignment does not exist |
| order truth | orcs `tasks/uolm/sensors.py::CONTACT_GRAPH_BODY_NAMES` (mirrored in `motion.hpp` and in the publisher; a legend that drops a name throws) |
| no file | zeros + `has_contact=false` |

Reading `body_names` needs one cnpy fix (`thirdparty/cnpy/cnpy.cpp`): numpy's
`<U25` counts **codepoints**, stored UCS-4, so word size is `4×`. Upstream cnpy
reads a quarter of the bytes at the wrong offset and returns garbage silently.
Numeric dtypes are untouched.

## the clock

`g1::MotionClock(motion, anchor_body)` owns the one playback position every
consumer samples through:

| call | does |
|---|---|
| `engage(robot_quat, start)` | bakes yaw offset `yaw(robot)·yaw(ref[start])⁻¹` (deployment brings the clip to the robot; mjlab play teleports the robot to the clip), resets `t` |
| `step(dt)` | advances by `fps·dt` frames, clamps at the end |
| `frame()` / `future_frame(k)` | current / current+k clamped — mirrors mocke `FutureMotionCommand.future_frames` |
| `aligned_root_quat(f)` / `aligned_root_pos(f)` | heading-rebased reference views |

## consumers

| node | reads |
|---|---|
| `g1_sonic` | `jp/jv` (tokenizer, motion_cmd), `aligned_root_quat` (tokenizer r6d) |
| `g1_vibe_sonic` | + `root_{lin,ang}_vel_b` and `contact` — the full augmentation port |
| `g1_textop` | `jp_il/jv_il` (WBC command), `root_pos/quat` (frame alignment); pads pre/post directly on the public arrays |
| `g1::SonicStand` | `stand()` reference + clock alignment |

## invariants (deploy_selftest-checked)

1. `IL2MJ ∘ MJ2IL = id`, tables match the name lists.
2. `from_wire` → `jp_il` round-trips the wire row exactly, at both widths.
3. Twist frame: world `+x` through a `+90°`-yaw anchor reads `-y`.
4. Missing vel/contact arrays → zero-filled + `has_{twist,contact}=false` (never garbage).
5. Clock: 50 fps @ dt 0.02 advances 1 frame/tick; futures clamp at clip end.
6. An unknown wire width throws — never a silently-zeroed command stream.
7. `deploy_selftest --motion <clip.npz>`: contact is `{0,1}`, prints per-body
   coverage (parity vs `ContactSchedule.body_object_contacts`), and the npz→wire→
   parse round-trip reproduces jp/jv/twist/contact exactly.
