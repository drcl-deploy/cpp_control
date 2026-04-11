# G1 Loco-Manipulation Controller

Dual-policy controller for the G1 humanoid that runs a locomotion policy and a loco-manipulation policy side by side, switchable at runtime.

The controller subscribes to XR (VR) controller poses for end-effector targets and accepts velocity commands from a joystick or the Unitree wireless gamepad.

## XR controller node (optional)

If an XR node is running, it should publish hand poses on:

```bash
/xr/left_controller    # geometry_msgs/Pose
/xr/right_controller   # geometry_msgs/Pose
```

If no XR node is connected, the controller falls back to a fixed default hand pose in front of the torso and continues to operate normally.

## launch

```bash
# default models from config/locomanip/g1.yaml and config/locomotion/g1.yaml
ros2 launch cpp_control g1_locomanip.launch.py

# custom models
ros2 launch cpp_control g1_locomanip.launch.py \
    onnx_model_path:=/path/to/locomotion.onnx \
    locomanip_onnx_path:=/path/to/locomanip.onnx

# custom XR topics
ros2 launch cpp_control g1_locomanip.launch.py \
    left_xr_topic:=/my/left_hand \
    right_xr_topic:=/my/right_hand
```

## swapping models

1. Place the new `.onnx` file under `models/loco-manipulation/`
2. Update `config/locomanip/g1.yaml` — change the `onnx_path` field:

```yaml
onnx_path: "loco-manipulation/your_new_model.onnx"
```

3. Verify `num_obs` and `num_actions` in the config match the new model's input/output dimensions:

```yaml
# locomanip observation layout (must equal model input size)
# ang_vel(3) + gravity(3) + cmd_vel(3) +
# left_hand_pos(3) + left_hand_quat(4) +
# right_hand_pos(3) + right_hand_quat(4) +
# joint_pos(29) + joint_vel(29) + last_action(29) = 110
num_obs: 110
num_actions: 29
```

The locomotion policy is configured separately in `config/locomotion/g1.yaml`.

---

## using joystick and gamepad together

Both inputs can be connected at the same time. They share the same velocity command variables, so they need a priority rule to avoid overwriting each other.

**Joystick takes priority for velocity when connected.**

- While the joystick is publishing (i.e., `/joy` messages arrive), the gamepad velocity inputs are ignored. The gamepad can still be used for mode switching (X, A, Y, B, up).
- If the joystick stops publishing for more than 0.5 seconds, the gamepad resumes velocity control automatically.

This makes the recommended workflow straightforward:

```
[gamepad]  B → X → up (stand up and enter locomotion)
[joystick] drive and switch modes with A / X / LB
```

The gamepad can be kept plugged in throughout — it will handle the initial stand-up without interfering with joystick velocity once the joystick is active.

---

## button mapping

### Unitree wireless gamepad

The gamepad is the primary interface for stand-up and initial mode entry.

#### mode switching

| button | result | notes |
|---|---|---|
| X | → `NOMINAL_POSE` | smooth interpolation to default standing angles (PD control); must do this before entering any policy |
| up (d-pad) | → locomotion `POLICY` | start locomotion; also use this to switch back from locomanip |
| A | → `LOCOMANIP_POLICY` | start loco-manipulation |
| Y | → `DAMPING` | soft hold, motors compliant |
| B | → `ZEROING` | all joints zero torque — use with caution |

#### velocity commands

Both `cmd_vel` (locomotion) and `locomanip_cmd_vel` are updated simultaneously, so the active policy always receives fresh commands.

| stick | locomotion mode | locomanip mode |
|---|---|---|
| left stick ↑↓ (`ly`) | vx (forward / back) × 0.5 | vx (forward / back) × 0.5 |
| left stick ←→ (`lx`) | vy (lateral) × −0.5 | vy (lateral) × −0.5 |
| right stick ←→ (`rx`) | yaw rate × −0.5 | yaw rate × −0.5 |

#### recommended startup sequence (gamepad)

```
power on → [B] zeroing → [X] nominal pose → wait for stand-up → [up] locomotion policy
                                                                → [A]  locomanip policy
```

---

### joystick (Xbox-style, X-mode layout)

The joystick does **not** have a d-pad, so mode switching is handled differently.
In particular, **X on the joystick skips the stand-up and jumps directly to locomotion policy** —
it is intended as a mid-session switch back from locomanip, not for initial stand-up.
Use the gamepad for the initial stand-up sequence.

#### mode switching

| button | result | notes |
|---|---|---|
| X | → locomotion `POLICY` | **no stand-up** — direct switch; use to exit locomanip back to locomotion |
| A | → `LOCOMANIP_POLICY` | switch to loco-manipulation |
| LB (left bumper) | → locomotion `POLICY` | alternative switch back to locomotion |
| Y | → `DAMPING` | soft hold |
| B | → `ZEROING` | all joints zero torque — use with caution |

#### velocity commands

| input | locomotion mode | locomanip mode |
|---|---|---|
| left stick ↑↓ | vx (forward / back) × 0.5 | vx (forward / back) × 0.5 |
| left stick ←→ | vy (lateral) × 0.5 | vy (lateral) × 0.5 |
| right stick ←→ | yaw rate × −0.5 | — (right stick unused) |
| L1 trigger | — | yaw + (left trigger contribution) |
| R1 trigger | — | yaw − (right trigger contribution) |

> In locomanip mode, yaw is controlled by `L1 − R1`, freeing the right stick.
> In locomotion mode, yaw comes from the right stick.

#### typical session flow with joystick only

Use the gamepad for stand-up, then switch to joystick:

```
[gamepad] B → X → up (stand up and enter locomotion)
[joystick] drive with sticks → [A] enter locomanip → [X or LB] back to locomotion
```
