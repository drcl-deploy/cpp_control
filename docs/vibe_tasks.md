# Vibe task runtime

One `G1VibeSonicNode` serves every extractor-era G1 Vibe artifact. Tensor
layout comes from `vibe.onnx.v1`; `task_id` selects the small amount of
lifecycle and command behavior that shapes alone cannot express. Startup
rejects an artifact whose model class, logical groups, term order, or launch
family does not match the task contract.

| launch | artifact task | external command | lifecycle |
|---|---|---|---|
| `g1_vibe_uolm.launch.py` | UOLM | root twist + contact; final object pose query | motion, prep, run |
| `g1_vibe_perloco_grail.launch.py` | PerLoco Grail | root twist (also the aliased task query) | motion, prep, run |
| `g1_vibe_perloco_omre.launch.py` | PerLoco OmRe | same interface as Grail, different weights | motion, prep, run |
| `g1_vibe_dodge.launch.py` | Dodge | none | stand-reactive; no motion or prep |

All four use Theia Tiny image tokens. Token tag, patch grid, feature width,
CLS width, finite values, and freshness are checked before policy entry.

## Reference transport

`publish-motion` publishes `cpp_control/msg/MotionReference` on
`/tracker/reference`. Version 1 always stores the full 83-column row, with
independent `has_twist`, `has_contact`, and `has_object_goal` flags. This makes
a PerLoco twist-only clip unambiguous and moves the UOLM goal atomically with
the clip. The legacy `/tracker/motion` and `/object_goal` messages are still
emitted for older consumers; a new SONIC controller ignores legacy motion
after receiving a typed reference.

The UOLM goal is the last pose in the sibling `object_motion.npz`, in the
canonical clip environment frame. It is a query-only training input and is
therefore not heading-rebased at deployment.

## Operation

Checkpoints stay local and are not tracked by this experimental package. Start
the encoder, pass the export explicitly, then stage a clip where required:

```bash
ros2 launch vision_encoders encoder.launch.py model:=theia-tiny
ros2 launch cpp_control g1_vibe_uolm.launch.py \
    onnx_path:=/path/to/model_24999.onnx
publish-motion /path/to/motion.npz
```

UOLM and PerLoco use `RB` stand → `L1` prep → `A` run. Dodge uses `A` or
`RB` to engage the nominal SONIC reference and ignores staged clips.

## Simulator follow-up

The controller side is simulator-agnostic and is complete here. The matching
simulation node should publish the same `MotionReference` schema and preserve
the trained camera contract. UOLM/PerLoco use the shared 45-degree-down,
42.5-degree-FOV camera; Dodge was trained with its distinct 2-degree-up,
42.5-degree-FOV camera. Do not silently reuse the current Unitree MuJoCo
58-degree-FOV sensor for parity testing.
