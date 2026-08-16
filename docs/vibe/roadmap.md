# Vibe deployment roadmap

This document separates the working deployment baseline from possible
hardening work. Add complexity only after measurements identify a concrete
limit.

## Current baseline

- The controller is C++ and runs the policy with ONNX Runtime using reusable,
  named CPU buffers at 50 Hz.
- `vision_encoders` is a separate C++ process using ONNX Runtime CUDA at 50 Hz.
  It receives real or simulated camera frames over the same TCP wire format.
- A `vibe.onnx.v1` manifest is authoritative for policy ports, actions, task
  identity, and control timestep.
- One task launch starts the telemetry bridge and encoder, then the shared Vibe
  controller. Five task wrappers cover Repose, UOLM, PerLoco Grail, PerLoco
  OmRe, and Dodge.
- Simulation and robot workflows differ only through the sourced environment.
- Offboard tools can view attention and record selected ROS topics without
  participating in the control loop.
- Control and command publication currently share one ROS wall-timer callback;
  there is no real-time scheduling or independent command watchdog.

## Next: measure and validate

1. Record policy, encoder, and end-to-end frame-to-command latency, including
   percentiles and missed 20 ms deadlines.
2. Add a repeatable observation/action replay check between exported training
   rollouts and C++ inference.
3. Complete simulator and onboard smoke matrices for every task family,
   including token mismatch, stale-token warning, bridge disconnect, and topic
   QoS behavior.
4. Record enough provenance with each bag to identify policy manifest,
   checkpoint, encoder tag, environment, and source revision.

## Then: harden only where measurements justify it

### Control and safety

- Add a command watchdog or decoupled high-rate publisher if the robot interface
  benefits from refreshing the latest valid command faster than policy rate.
- Consider thread priority and CPU affinity after measuring scheduler jitter.
- Define automatic damping transitions only after their triggers and recovery
  behavior are exercised safely; stale vision remains warning-only for now.

### Inference

- Add CUDA or TensorRT policy execution behind the existing `OnnxSession`
  interface if CPU inference threatens the 20 ms budget.
- Cache device-local TensorRT engines by model and runtime version; never ship
  engines as portable artifacts.
- Preserve ONNX Runtime CPU as the simple parity and diagnostic backend.

### Observability

- Extend recording from selected ROS topics to named observation terms,
  actions, mode transitions, and per-stage timing when deeper parity debugging
  is needed.
- Keep high-volume diagnostics opt-in so the normal onboard path stays small.

### Camera and artifacts

- Evaluate in-process RealSense capture only if the Python camera streamer or
  TCP encoding is measured to affect latency or visual parity.
- Consider a versioned bundle descriptor joining policy and encoder artifacts;
  the current independent manifests and startup token checks remain sufficient
  until multiple backbones are deployed routinely.

## Deferred

- A small standing-policy mode shared across controllers.
- General input multiplexing, dexterous-hand support, or a production logging
  framework unrelated to current Vibe experiments.
- Replacing ROS 2 or splitting the single policy graph into multiple engines
  without a demonstrated operational need.
