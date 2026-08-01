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

| factory | input | twist | used by |
|---|---|---|---|
| `from_npz(path, perm)` | retargeted-dataset / mjlab npz (`joint_pos`, `joint_vel`, `body_{pos,quat,lin_vel,ang_vel}_w`, `fps`) — mocke `MjMotionLoader` twin; `perm = g1::MJ2IL` for IL-ordered clips | from file | sonic, vibe-sonic |
| `from_wire(T, rows)` | textop topic rows `[jp29 \| jv29 \| anchor_pos3 \| anchor_quat4]`, IL-ordered | zeros (`has_twist=false`) | textop, residual v1/v2 |
| `stand(defaults)` | nominal pose, identity anchor | zeros (truth) | every stand mode |

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

The `_b` twists are the sys1 command stream
(orcs `robot_root_{lin,ang}_vel_cmd`): `R(q_ref_anchor)ᵀ · v_world`, a pure
clip function. Rotating quat and vel together under a heading rebase cancels —
so they never touch live robot state and need no alignment.

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
| `g1_vibe_sonic` | + `root_{lin,ang}_vel_b` (augmentation twist cmds) |
| `g1_textop` | `jp_il/jv_il` (WBC command), `root_pos/quat` (frame alignment); pads pre/post directly on the public arrays |
| `g1::SonicStand` | `stand()` reference + clock alignment |

## invariants (deploy_selftest-checked)

1. `IL2MJ ∘ MJ2IL = id`, tables match the name lists.
2. `from_wire` → `jp_il` round-trips the wire row exactly.
3. Twist frame: world `+x` through a `+90°`-yaw anchor reads `-y`.
4. Missing vel arrays → zero-filled + `has_twist=false` (never garbage).
5. Clock: 50 fps @ dt 0.02 advances 1 frame/tick; futures clamp at clip end.
