# onnx policies — the two serving stacks

cpp_control serves policies exported by two different pipelines, so it keeps
two ONNX stacks. Both stay: the manifest stack needs a `vibe.onnx.v1` export,
and plenty of checkpoints (collaborators', older runs) are plain onnx.

| | legacy | manifest |
|---|---|---|
| class | `ONNXPolicy` | `deploy::OnnxSession` + `deploy::DeployManifest` |
| graph io | one obs vector in, one action out | named ports, any count |
| obs layout | hand-counted in C++ (`build_*_observation`) | term tables in `<model>.manifest.json` (name/dim/offset/history) |
| gains / defaults / scale | node's yaml | the checkpoint's manifest |
| joint order | reindex tables in the node | manifest `joint_names`, asserted vs config at startup |
| wrong-layout failure | silent garbage | loud throw (port/term/dim/name mismatch) |
| consumers | `g1_locomotion`, `mini_pi_{locomotion,dive}`, textop WBC, residual v1/v2 HLC, `g1_difftrack` | `g1_sonic`, `g1_vibe_sonic`, `g1::SonicStand` |
| artifact source | any exporter | vibe `export-agent` (schema `vibe.onnx.v1`) |

## which to use

- **new vibe/mocke policy** → manifest stack, always. The exporter writes the
  manifest next to the onnx; the node binds every port by name and the yaml
  carries plumbing only.
- **external / legacy checkpoint** (no manifest) → `ONNXPolicy` + a
  hand-built obs function. Count your dims twice.

`g1_difftrack` is the middle case and worth knowing about: it serves a plain
one-in-one-out graph through `ONNXPolicy`, but its layout is not hand-counted in
C++ either — the exporter writes `difftrack_config.json` beside the graph, and
the node asserts the assembled width, the joint names, the control period and
the FK table against it at startup, then scores the whole chain against a golden
rollout in `difftrack_selftest`. Different schema from `vibe.onnx.v1`, same
principle: the checkpoint is the authority and a mismatch is loud.

## manifest contract

Full schema + export flow: [trackers/custom_sonic.md](trackers/custom_sonic.md).
The short of it:

```
model.onnx                      # the graph, all preprocessing baked in
model.manifest.json             # vibe.onnx.v1:
  control.step_dt               #   asserted vs control_dt
  action.{joint_names,scale,default_joint_pos,stiffness,damping}
  inputs[].terms[]              #   name / dim / offset / history per port
  outputs[]                     #   "actions" first; extras (e.g. "attn") ride along
```

`deploy_selftest` smoke-loads any pair:

```bash
./build/cpp_control/deploy_selftest model.onnx [model.manifest.json]
```
