# Local patches to upstream checkouts

Changes this package needs in code it does not own. Each one is here so that it
is reviewable, re-appliable after a `git pull`, and obviously a delta rather
than something that quietly diverged.

```bash
cd $UNITREE_ROS2/unitree_mujoco
git apply /path/to/cpp_control/workflows/patches/unitree_mujoco-startup-window.patch
git apply /path/to/cpp_control/workflows/patches/unitree_mujoco-f9-recording.patch
git apply /path/to/cpp_control/workflows/patches/unitree_mujoco-headless.patch
cd simulate/build && make unitree_mujoco -j4
```

**In that order.** All three touch `simulate/src/main.cc`, and each patch's
context lines are the previous one's output. `git apply --check` first if the
checkout has moved on.

---

## `unitree_mujoco-startup-window.patch`

**Why:** `unitree_mujoco` integrates physics on a robot nothing is commanding,
for as long as it takes its DDS bridge to come up. A quadruped shrugs that off.
A G1 does not.

Measured here, on the shipped `g1_29dof` scene:

| | simulated time before the first `LowState` |
|---|---|
| upstream | **716 ms** |
| with this patch, `usleep` + DDS hoist | ~336 ms |
| with this patch and `--wait-for-cmd` | **0 ms** |

And measured against `cpp_control`'s difftrack hold (ramp from the model's reset
pose into the policy's nominal pose, shipped gains), as a function of how long
the robot was left limp first:

| limp window | outcome |
|---|---|
| ≤ 0.36 s | held, pelvis 0.776 m, zero torque saturation |
| ≥ 0.38 s | fell, unrecoverable |

716 ms is well past that cliff, so *every* difftrack sim2sim run on the
unmodified simulator measured a startup race rather than a policy. The
give-away, if you ever see it again: the controller's first state shows the
pelvis still up at ~0.79 m but the hips already a third of a radian out of
place, and no gain recovers it.

**What it changes**, all three deliberately small:

1. `usleep(500000)` → `usleep(1000)` in the bridge's "wait for mujoco data"
   poll. Half a second is a long time to wait for a pointer that the thread
   next to you has already set.
2. `ChannelFactory::Init` hoisted out of the bridge thread into `main()`, before
   the physics thread starts. Creating the DDS participant needs nothing MuJoCo
   owns, so it does not belong inside the window.
3. A new **`--wait-for-cmd` / `-c`** flag: hold the simulation at its reset state
   until the first `LowCmd` arrives, then start the clock. **Off by default** —
   without the flag, upstream behaviour is unchanged and a simulator started on
   its own still runs. `run_difftrack_sim2sim.sh -W unitree` passes it, because
   the alternative is publishing numbers that are partly a measurement of how
   fast DDS came up.

(1) and (2) are plain bug fixes and would be worth upstreaming. (3) is a
harness feature; it is opt-in for that reason.

**What it does NOT change:** the physics, the model, the message contract, the
PD law in the bridge, the order of anything after startup.

---

## `unitree_mujoco-f9-recording.patch`

**Why:** two things, one of which is a plain bug.

**1. `F9` records an mp4**, start and stop, to `DRCL_RECORD_DIR`. The viewer is
MuJoCo's stock `simulate`, which has no recorder; the drcl plant's equivalent
was `cpp_control/scripts/frame_recorder.py` and went with that plant. This is
the same design in C++ and for the same reasons: the scene is rendered a SECOND
time into an offscreen buffer and the raw RGB is piped to `ffmpeg`, rather than
screen-grabbing the window. Offscreen means the video resolution is independent
of the window size, the capture survives occlusion and minimisation, and it does
not care what the compositor reports the drawable size to be (this rig runs
XWayland, where that goes wrong).

What is in the video is what the viewer is showing: the camera and the
visualisation options are read from the live `sim->cam` / `sim->opt` every
frame, so the follow camera, a mouse orbit and a toggled visualisation all show
up. MuJoCo's own on-screen UI panels are drawn straight to the window and never
reach the offscreen buffer.

Frames are emitted on a **simulation-time** schedule with the last render
duplicated to fill a gap, so the file is a constant-rate stream that plays back
at 1x and cuts pauses out. `DRCL_RECORD_CLOCK=wall` records what you watched
instead. Settings come from the same environment variables
`frame_recorder.py` reads, so a shell set up for the mj_sim recorder drives this
one unchanged:

| | default | |
|---|---|---|
| `DRCL_RECORD_DIR` | `recordings` | output directory |
| `DRCL_RECORD_SIZE` | `1280x720` | clamped to the scene's offscreen buffer |
| `DRCL_RECORD_FPS` | `30` | |
| `DRCL_RECORD_ENCODER` | `libx264` | try `h264_nvenc` |
| `DRCL_RECORD_CRF` | `23` | lower is better |
| `DRCL_RECORD_FFMPEG` | `ffmpeg` | |
| `DRCL_RECORD_PREFIX` | `unitree_mujoco` | file name prefix |
| `DRCL_RECORD_CLOCK` | `sim` | `sim` or `wall` |
| `DRCL_RECORD_AUTOSTART` | `0` | `1` = F9 at t=0, for an unattended run |

`DRCL_RECORD_AUTOSTART` is what `run_difftrack_sim2sim.sh -R` sets, and it is
also the only way to record on a machine with no keyboard on the window.

The **offscreen buffer is a model property** (`<visual><global offwidth=
offheight=>`, default 640x480) and a viewport larger than it is silently
clipped, so a 720p recording off a stock scene would come out cropped rather
than scaled. The recorder clamps and says so; `make_sim2sim_scene.py --flavor
unitree` sets 1920x1080.

Threading is the part worth reading before changing it. The key callback runs on
the UI thread, which owns the window's GL context, so `F9` only sets an atomic;
the recorder owns a context of its own — a hidden window created on the main
thread, because GLFW requires window creation there, and made current on the
capture thread. `mjv_updateScene` needs `m` and `d` alive so it runs under
`sim->mtx`; the render and the readback do not and stay outside it.

**2. `glfwSetKeyCallback` no longer throws MuJoCo's keyboard away.** GLFW allows
exactly one key callback per window, and the line at the bottom of `main()`
replaced `GlfwAdapter`'s — which is the whole of MuJoCo's keyboard UI. So
`space` to pause, `[` / `]` to cycle cameras, Esc for the free camera, F1 for
help and Ctrl+P for a screenshot have all been dead in `unitree_mujoco`, and
silently: a key that does nothing looks like a key you are pressing wrong. The
patch keeps the callback GLFW returns and calls it after its own, which restores
all of it. Upstream-worthy on its own.

**What it does NOT change:** the physics, the model, the message contract, the
PD law in the bridge, or anything about startup. Recording costs one extra scene
draw plus a readback at the capture rate (30 Hz), on a thread of its own.

---

## `unitree_mujoco-headless.patch`

**Why:** every sim2sim run opened a MuJoCo window. For a sweep -- the estimator
channel ablation in `recordings/estimator_ablation/` is ~70 runs -- a window per
run is pure cost: a GL context on a GPU that is usually training something, a
window taking focus every run, and no run at all on a machine without a display.

**What it adds:** `--headless` / `-H`. Load the scene, `mj_forward`, then step
at 1x against the wall clock with the viewer loop's own re-sync policy (re-sync
instead of bursting once more than 0.1 s behind, and say so on stdout). No GLFW,
no `mj::Simulate`. `run_difftrack_sim2sim.sh -H` passes it.

**Why a separate loop rather than a hidden window:** `mj::Simulate::Load` hands
the model to the RENDER thread and blocks until that thread has taken it, and the
`GlfwAdapter` that `Simulate` is built on creates its window in the constructor.
So the headless path branches off in `main()` before either exists.

**Unchanged:** the DDS bridge (it reads `m`/`d` and never took `sim.mtx`),
`--wait-for-cmd`, the model, the physics. **Not applied** headless: the elastic
band (the rig never enables it; a warning is printed if the config does) and F9
recording (nothing to record -- `-H -R` is refused by the rig).

**Measured:** `g1_jumps9`, stand entry, 15 s, ground truth: 748/748 steps, mean
root error 0.287 m, min pelvis height 0.694 m -- inside the spread of the same
policy's windowed runs (0.27-0.32 m, 0.69 m) -- with zero re-syncs.
