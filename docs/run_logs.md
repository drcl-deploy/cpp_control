# Run logs — keeping sim2sim and sim2real runs

One npz per run, written by the controller, holding every control step of it:
the reference the policy was given, the state it observed, the action it
produced, and the command that went out. The same node writes the same file in
sim2sim and on the robot, which is the whole point — **a transfer result is one
policy measured the same way in two plants**, and that is only possible if both
runs were kept.

```bash
# sim2sim: already on. Every run lands in recordings/runs/
bash scripts/run_difftrack_sim2sim.sh -s -d 10 g1_walk

# hardware: also already on. Tag it with what it is
ros2 launch cpp_control g1_difftrack.launch.py motion:=g1_walk \
    config_path:=$(ros2 pkg prefix cpp_control)/share/cpp_control/config/tracker/g1_difftrack_hw.yaml \
    optitrack_topic:=/optitrack_adaptor/mocap_frame optitrack_rigid_body_id:=1 \
    run_tag:=hw_optitrack record_dir:=~/runs

# then
python3 scripts/analyze_tracking.py ~/runs/*.npz
python3 scripts/analyze_tracking.py --compare <sim>.npz <robot>.npz -o out/
```

---

## 1. Why it is inside the controller

`scripts/record_policy_actions.py` makes the opposite choice on purpose: it
recovers the actions from `/lowcmd` so that measuring a run cannot change it.
That works because an action **is** on the wire. A tracking error is not.

The reference is a clip placed on the robot by an anchor resolved at the instant
`A` was pressed, in a frame the node chose, at a step index the node owns. None
of that is published and none of it can be reconstructed from a bag afterwards.
Neither can the world pose the policy actually observed: it has been through a
staleness check, a finite difference over the *measured* mocap interval, and
possibly a body→world rotation.

The second reason is alignment. Off the wire, a state stream and a command
stream have to be joined by timestamp, and a dropped message silently deletes a
control step. Here **one row is one control step** — observation, action,
reference, measured state and published command are the same tick by
construction, and a step that did not happen leaves a visible gap in `t_wall`
and `state_tick` instead of a quietly shortened run.

**What it costs the control loop**, which is the part that matters on hardware:

| | |
|---|---|
| allocation | none after `begin()`. The whole run goes into one buffer reserved up front; a run that outgrows it stops appending and reports `truncated` rather than reallocating under a walking robot |
| serialisation | none in the control thread. `end()` moves the buffer to a writer thread and returns. This matters because a run ends on the tick the clip does, which is the tick the rest state has to catch a robot at ~1 m/s — exactly where a 30 ms `fwrite` must not be |
| per step | a `memset` of the row and a few dozen stores |

---

## 2. Where the files go, and what a run is

| parameter | default | |
|---|---|---|
| `record` | `true` | off with `record:=false` |
| `record_dir` | `$CPP_CONTROL_RECORD_DIR`, else `<cwd>/recordings/runs` | |
| `run_tag` | `run` | free-form, in every file name and index line |
| `record_obs` | `true` | also log the policy's 849-dim input |
| `record_max_seconds` | `60` | hard capacity of one run |
| `record_tail` | `3.0` | seconds to keep logging after the rest state takes the robot |

`run_difftrack_sim2sim.sh` passes `record_dir:=<pkg>/recordings/runs` and
`run_tag:=sim2sim_<-E mode>`; `RUN_LOG=0` turns it off and `RUN_LOG_DIR` moves
it.

```
recordings/runs/
  20260908-171304_sim2sim_estimator_g1_jumps12_r01.npz
  20260908-171330_hw_optitrack_g1_walk_r02.npz
  index.jsonl
```

`index.jsonl` is one line per run — tag, motion, train run, world source, host,
steps, mean/max error, min height, whether it fell and why — so a session is a
table without opening any of the files:

```bash
jq -r '[.file, .motion, .steps, .mean_err, .fell_at] | @tsv' recordings/runs/index.jsonl
```

**A run is one engage-to-handover cycle.** It starts when `A` is pressed, or
when an unattended run engages, and it is written to disk on whichever of these
comes first:

| the run ends | `closed_by` | |
|---|---|---|
| the clip finishes | `clip_over` | plus `record_tail` seconds, so the handover is in the file |
| the robot falls | `fell` | plus the same tail, with the node in damping |
| `B` or `Y` | `aborted (zeroing)` / `aborted (damping)` | closed on that tick. `X` and `R1` do the same — anything that takes the robot out of the policy |
| `A` again | `superseded` | the next run opens immediately, numbered `r02` |
| Ctrl-C, or `kill` | `node shutdown` | rclcpp handles both SIGINT and SIGTERM, so the node unwinds and the file is written |

The tail exists because the clip ending is not the run ending. With `exit_hold`
or `exit_ramp` set the policy is still driving in between, and a robot that
tracked cleanly and then went down as the stand caught it is a transfer failure
that belongs in the same file.

An abort gets no tail. `Y` is the reaction to a run going wrong, and what
happens after it is a robot being put down rather than a controller being
measured. The run's statistics are still reported: the counters live until the
next engage, so an aborted file carries the mean and maximum error over the
steps that did happen, with `reason` saying how it ended.

Every control step in that window is a row, whatever mode ran: the entry ramp,
the tracking, the exit, the stand. `mode` and `phase` say which, and `phase` is
the phase the step **ran in** rather than the one it left behind — so
`policy_tick == 1 & phase == 1 & clip_step >= 0` selects exactly the steps the
node's own `SUMMARY` line averaged, and the two numbers can be checked against
each other.

---

## 3. The columns

Raw capture, nothing derived. Joint columns are in the robot's **motor** order
exactly as it reports and receives them; the reference and the action are in
**policy** order, and `joints.policy_to_motor` in the metadata is the
permutation between them (the identity on the G1, resolved by name anyway —
a permutation is the one error in this pipeline that produces no error at all).

| column | width | |
|---|---|---|
| `t`, `t_wall` | 1, 1 | seconds since the run began, ROS clock and steady clock. A stalled loop shows in the second |
| `step`, `clip_step` | 1, 1 | episode step and step within the clip (negative during a lead-in). NaN on a row with no inference |
| `phase`, `mode` | 1, 1 | see `phase_enum` / `mode_enum` in the metadata |
| `policy_tick` | 1 | 1 when the row carries an inference |
| `state_tick` | 1 | the plant's own counter, relative to the run's first — a gap is a dropped state message |
| `world_valid`, `world_age` | 1, 1 | whether the world pose was usable, and how old it was |
| `root_pos`, `root_quat`, `root_lin_vel`, `root_ang_vel` | 3, 4, 3, 3 | the world state the **controller used**, after the staleness check and any frame rotation — not the raw topic |
| `ref_root_pos`, `ref_root_quat`, `ref_dof_pos` | 3, 4, nA | the reference, anchored into the world exactly as the policy saw it |
| `imu_quat`, `imu_gyro`, `imu_accel` | 4, 3, 3 | raw, body frame for the gyro |
| `q`, `dq`, `tau`, `temp` | nM each | measured position, velocity, `tau_est`, motor temperature (°C, hardware only — every simulator leaves it 0) |
| `cmd_q`, `cmd_dq`, `cmd_tau`, `cmd_kp`, `cmd_kd` | nM each | the command that was published. The gains are what identify the mode: the policy drives through the checkpoint's trained stiffness, every hold mode through the yaml's |
| `action`, `target` | nA, nA | raw policy output, and `default + scale * action` |
| `obs` | nObs | the policy's input, when `record_obs` is on |
| `meta_json` | — | the run's metadata, uint8 |
| `difftrack_config_json` | — | the export's own config, verbatim |

The export config travels **inside** every run file. It is 16 kB, it carries the
kinematic table the body-space metrics need, and a run analysed six months later
on another machine will not have the model directory beside it.

`meta_json` carries the export identity (source run, train run, variant,
checkpoint iteration), both gain tables, the default poses, every task
parameter, the world-pose source in force, the anchor the clip was played at,
the node's own `SUMMARY`, and the column layout. Read one in three lines:

```python
import json, numpy as np
z = np.load("recordings/runs/<run>.npz")
meta = json.loads(bytes(z["meta_json"]).decode())
err = np.linalg.norm(z["root_pos"] - z["ref_root_pos"], axis=1)
```

---

## 4. Reading them

```bash
python3 scripts/analyze_tracking.py recordings/runs/*.npz         # one table per run
python3 scripts/analyze_tracking.py --compare A.npz B.npz         # the transfer gap
python3 scripts/analyze_tracking.py -o out/ A.npz B.npz           # + figures
python3 scripts/analyze_tracking.py --json  ... | jq              # everything
```

The per-run table is in two halves, and the split is the point:

**tracking** — where the robot was against where the reference said to be.
`E_root` is the same quantity the node's `SUMMARY` reports; the rest splits it
into the parts that fail differently. `E_z` and `E_rot` are observable to an
onboard estimator and `E_xy` is not, so a run on `/odom` that looks bad in
`E_xy` and fine in the others is an **estimator** result rather than a policy
one. `E_jpos` is the reference joint angles against the measured ones — the
error the policy is actually paid for, where the root error is partly the
plant's. `mpjpe` is the same thing in the world through the export's own
kinematic table, and `local` is that with the root error taken out: how wrong
the *shape* was rather than where it was.

**execution** — what the two plants did not share. Torque saturation against the
export's own limits, control-period jitter, dropped state messages, motor
temperature, the PD loop's own following error (`E_cmd`), and the action
reversal rate. This is where a sim2real gap actually comes from, and none of it
is visible in any tracking number.

`--compare` pairs two runs **by clip step** — not by time, which drifts between
plants, and not by row, which shifts the moment either run misses a control step
— and prints both runs and their difference on the steps they have in common. It
warns when the two files are not the same export, because "this policy on a
robot" and "a different policy" are different questions and nothing downstream
would say so. With `record_obs` on in both, it also reports how far apart the
policy's own **input** was, split into the character block (what the robot
changed) and the reference block (what it did not).

---

## 5. On the robot

The recipe, and the reason it is this one:

1. **Leave `record` on and set `run_tag`.** The tag is what tells a simulated
   run from a hardware one in a directory of both, and the metadata records the
   world-pose source, the host and the gains rather than trusting the tag for
   anything that matters.
2. **Run the same clip in sim2sim first**, with `-E estimator` if the robot will
   be on the onboard estimator, so the two runs differ in the plant and not in
   the world-pose source as well. A run measured against the simulator's ground
   truth is an upper bound, not a transfer result.
3. **Set `record_dir` to real storage.** The default is relative to the working
   directory; on a robot's own computer that can be a small overlay.
4. **`record_obs` costs about 3.4 kB per control step** — a 30 s run is 7 MB with
   it and 2 MB without. Leave it on. The observation is recomputable from the
   rest of the row in principle, and having it means not having to.
5. **Every end is a clean end except one.** The clip finishing, a fall, `B` or
   `Y`, `A` again, Ctrl-C and a plain `kill` all write the file, and `closed_by`
   says which it was (see the table above). `SIGKILL` does not: the buffer is in
   memory until the run ends, so a `kill -9` loses that run and only that run.

### Limits, stated

- **The buffer is a hard bound.** `record_max_seconds` (60 by default, and
  automatically raised to cover a longer `play_duration`) is reserved up front
  and never grows. A longer run stops appending, sets `truncated`, and says so
  on the console and in the index.
- **Nothing is written until a run ends.** That is what keeps the file system out
  of the control loop, and it is why a power cut loses the run in progress.
- **`temp` is empty in simulation** and `world_age` is zero unless the world pose
  came from outside the state stream. Both are hardware-only by construction,
  not by accident.
- **Two buffers are reserved**, so a session's memory is `2 × record_max_seconds ×
  rate × row`, printed at startup in the `run log ->` block.
