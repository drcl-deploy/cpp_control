# Vibe deployment background

This note records the design choices behind the Vibe runtime. For operation,
see [experiments.md](experiments.md); for implemented task contracts, see
[tasks.md](tasks.md); for unfinished work, see [roadmap.md](roadmap.md).

## Deployment constraints

Vibe policies combine an adapted SONIC controller with cross-attention over a
frozen vision backbone. Deployment therefore has two inference boundaries:

1. `vision_encoders` converts camera frames into typed image tokens.
2. `cpp_control` combines those tokens with robot state and task commands in
   one exported policy graph.

The control loop must remain onboard. Offboard networking is limited to
visualization and recording, so losing the bridge client cannot interrupt the
controller.

## Lessons from reference stacks

Two earlier SONIC deployment styles informed this implementation:

| Stack | Useful lesson | Deliberate omission |
| --- | --- | --- |
| NVIDIA `gear_sonic_deploy` | real-time scheduling, decoupled command publication, cached accelerated inference, explicit safety states | its monolithic runtime and multi-engine choreography |
| Community Python sim-to-real stack | named observation terms, history owned by each term, detailed rollout recording | Python and serialized I/O/inference in the control loop |

`cpp_control` already supplied the reusable robot/task hierarchy, ROS 2 seam,
YAML configuration, and identical simulator/robot topic interface. The Vibe
runtime was built there instead of introducing a third deployment framework.

## Export contract

The exporter produces one policy ONNX graph and a sibling
`.manifest.json`. The graph owns the adapted SONIC weights, extractor,
normalization, and named inputs. The manifest describes:

- input names, shapes, logical groups, terms, and history layout;
- action joint order, scale, gains, and default pose;
- task identity and control timestep;
- output names, including attention when exported.

The vision backbone stays separate because it has its own frame preprocessing,
rate, and reusable token stream. Its tag and token geometry are checked against
the policy contract before policy use.

## Implemented decisions

- `OnnxSession` binds named ports and allocates tensor buffers once.
- `G1VibeSonicNode` derives tensor layout from the manifest and rejects task,
  term-order, joint-order, or shape mismatches.
- Small task profiles describe lifecycle differences that tensor shapes cannot
  express; one runtime serves Repose, UOLM, PerLoco, and Dodge artifacts.
- Each task wrapper owns the encoder, controller, and optional telemetry bridge.
- An artifact directory is resolved to a matching ONNX/manifest pair; ambiguous
  directories require an explicit checkpoint selector.

This is intentionally smaller than a production hard-real-time stack. It does
not imply TensorRT policy inference, real-time thread priority, a high-rate
command republisher, in-process camera capture, or per-term flight recording.
Those are roadmap items, not hidden assumptions about the current runtime.
