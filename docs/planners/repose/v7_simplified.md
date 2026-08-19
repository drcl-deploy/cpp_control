# The planner v7 — the simplification build

Same algorithm, same numbers, one less idea. v7 shipped with OBSERVE, PLAN and ACT
fused at the controller's `finished` edge, and every special case in the planner existed to
patch that fusion. This build splits the three and deletes the patches.

Design of record: [planner.md](planner.md). This doc is the **delta**: what changed,
why, and what it is allowed to cost. Nothing here restates §3.

---

## 1. Parity matrix

sim2sim, same artifact, same 78-row table, same goal colour. Left column is the
claim; right column is what actually backs it.

| axis | before | after | verdict |
|---|---|---|---|
| retrieval cost, ladder, 4-symmetry search | `retrieve()` | untouched | **bit-identical** — `repose_planner_selftest` re-asserts `entry_yaw` == the cost's heading term |
| reference bytes (still + clip rows, lead-in, warp) | `writer.cpp` | one identifier renamed | **bit-identical** |
| decision RATE | one per `finished` | one per `finished` | **unchanged** — ACT still fires on the same edge |
| evidence PER decision | 1 read, the last one | ~12 reads, voted | **changed on purpose** |
| SCAN sweep rate | 90 f, 0.96 rad/s | 110 f, 0.98 rad/s | **unchanged** — the extra 0.4 s is all quiet time |
| SETTLE | 40 f | 40 f | unchanged |
| solve rate, sim2sim | v7 | v7-simplified | **indistinguishable** (operator call, one session) |
| wire | `PlannerStatus` @ 1 Hz | `PlannerStatus` @ 20 Hz, +10 fields | additive; `ControllerStatus`, `MotionReference` untouched |
| binary | v5 / v6 / v7 | v6 / v7 (+ v7.1) | one dead config removed; v7.1 landed after, [planner.md §7.1](planner.md) |

Parity was the goal. **The changes are all in what happens when something goes
wrong** — a lost camera, a flickered read, a dropped reference — and sim2sim on the
happy path is the evidence that nothing else moved.

---

## 2. What changed, and why

### 2.1 The split

| | before | after |
|---|---|---|
| observe | inside the commit branch, 4 frames per cycle | every tick the camera is quiet |
| plan | inside the commit branch, once per cycle | `decide()` is PURE, every tick |
| act | the same branch | `commit()` only, on `the controller.finished` |

`decide()` being pure is the whole trick: it can run at 20 Hz for a live intent
preview and be committed later without the answer having drifted. The clip is
burned and the roll counted in `commit()`, at the moment a reference is actually
published. `repose_planner_selftest` asserts twenty `decide()` calls answer identically and
burn nothing.

### 2.2 The belief — the one new mechanism

`include/cpp_control/planners/repose/belief.hpp`, ~100 lines. A vote over the last `belief_window` reads
taken while the camera was quiet. It is not an improvement to the perception
stack; it is a **replacement for four rules**, listed in [planner.md §3](planner.md).

The quiescence gate is the physical condition the old read window was a proxy for:

```
omega_cam = |imu_state.gyroscope| + sum |dq| over the 3 waist joints
```

held stance ~0.05 rad/s, a 90 deg sweep 0.96 — an order of magnitude apart, so the
gate needs no tuning to separate them. `omega_still = 0.15` sits between them by
estimate, not by measurement; `omega_cam` and `accept_rate` are on the wire so the
first real bag replaces it.

### 2.3 The deletions

| gone | why it existed | what replaced it |
|---|---|---|
| `Cfg::v5()` | the A/B | finished; v6 is the only ablation left |
| `stall_limit` | forced a rung swap after N stalls | built, measured, shipped OFF in the Python too. A ladder that advances without a tip desynchronises from the cube |
| `min_visible_color < 0` sentinel | "follow `min_visible`" = v5 | `color_gate()` is now `return min_visible_color;` |
| the read window (`read_tail` as a *read* trigger) | the camera is only usable at a still's end | the quiescence gate, which is true wherever it is true |
| camera staleness as a park condition | a blind planner should not act | a blind planner **plans the stance** — a camera lost mid-clip now puts the robot back on its feet instead of freezing it in a half-turned exit pose |
| eyes-shut-during-a-clip as control flow | a clip's frames are not readable | still true, still enforced — but as one gate condition at the top of `observe()`, not a branch in the loop. The mandatory post-clip settle now falls out as a *consequence*: clip ends -> belief empty -> `STILL(0)` |
| `ClipTable::nominal_stand()` | never called | — |

### 2.4 The re-sizings

| knob | was | now | reason |
|---|---|---|---|
| `read_tail` -> `hold_tail` | 4 | **15** | the tail is now sized in EVIDENCE, not in camera timing: 15 frames at 50 fps is 0.6 s, ~12 reads at a 20 Hz tick |
| `scan_steps` | 90 | **110** | absorb the longer tail at an unchanged sweep rate, so nothing new arrives for the interpolation lane to soak up |
| `settle_steps` | 40 | 40 | its whole hold is already quiescent |

### 2.5 The fixes that were not design

1. **Data race.** `last_frame_` was an `rclcpp::Time` written by the RX thread and
   read by the timer, unlocked. Now `std::atomic<uint64_t> frame_seq_` (release)
   + `std::atomic<int64_t> last_frame_ns_`; the tick reads the seq to decide
   whether a frame is new at all.
2. **Reference resend backoff.** An unechoed `reference_id` re-published every
   tick. Now once per 4 ticks, with the warn throttled.
3. **`R` was dead in the console.** `key.lower() in KEYS and key != "R"` — `R`
   lowercases into `KEYS`, and the guard then excluded it. `R` now always re-arms,
   which also aborts a trial that went wrong.
4. **Console padding.** `f"{vote:<12}"` on a string containing ANSI escapes:
   `len()` counts the escapes, so the column collapsed. Pad the plain text, then
   colour it.

Not this build: `sight.{hpp,cpp}` (palette-calibration lane) and `writer.cpp`'s
interpolation work.

---

## 3. The block diagram

```
  camera 5555          +-------------------------------------------+
  30-50 Hz  ---------> | RX thread   decode -> bgr / depth / intr   |
                       |             last_frame_ns_, frame_seq_++   |
                       +---------------------+---------------------+
                                             | newest frame only
  /lowstate            +---------------------+---------------------+
  500 Hz    ---------> | on_lowstate  q_il, dq_il -> LiveState      |
                       |              omega_cam = |gyro| + |dq_wst| |
                       +---------------------+---------------------+
                                             |
                       +---------------------v---------------------+
                       | tick()   20 Hz                            |
                       |                                           |
                       |  GUARD    the controller accepting - lowstate fresh |
                       |           - nominal stance known          |
                       |                                           |
                       |  OBSERVE  mode != CLIP - seq is new       |
                       |           - omega_cam < omega_still       |
                       |              CubeSight -> Sight -> Belief |
                       |                                           |
                       |  PLAN     clips.observe(belief)  ladder   |
                       |           cand = clips.decide(belief) PURE|
                       |                                           |
                       |  ACT      the controller.finished -> commit(cand)   |
                       |              belief.clear()               |
                       +------+------------------------------+-----+
                              | commit                       | every tick
                              v                              v
                   ReferenceWriter                    /vibe/planner/status
                   lead-in + rows + yaw ramp          (mode, belief, gate,
                              |                        plan, candidate)
                   /tracker/reference (v2)                    |
                              v                               v
                   +--------------------------+        repose_console.py
                   | the controller  g1_vibe_sonic 50 Hz|        (sets goal_color)
                   +------------+-------------+
                                | /vibe/controller/status {finished, reference_id}
                                +---------> back into ACT
```

Two modes on the wire is a bag-readability choice: structurally there is a CLIP
and a STILL whose yaw is zero (SETTLE) or swept (SCAN).

---

## 4. Build summary

| | |
|---|---|
| branch | `lok-i/simplify-sys1-repose` off `lok-i/sys1-repose` |
| new file | `include/cpp_control/planners/repose/belief.hpp` |
| touched | `repose/{cfg,clips,node,table,writer}` + `PlannerStatus.msg`, `g1_repose.yaml`, `repose_console.py`, `repose_planner_selftest.cpp`, `docs/planners/repose/{planner,experiments}.md` |
| net | +712 / -312 across 16 files, of which docs are ~210 |
| selftest | **52 checks, 0 failures** on the real 78-row table, v6 and v7 both |
| boot | `the planner v7 ready: 78 clips (F15 B4 L18 R19) \| pattern RRF \| target blue \| belief 3/8 reads under 0.15 rad/s` |
| sim2sim | run by the operator; behaviour indistinguishable from pre-split v7 |
| unmeasured | `omega_still = 0.15` is an estimate. Read `gate` in the console: ~0% of frames = too tight (safe, goes nowhere), 40-80% during a SETTLE = healthy, ~100% = accepting sweep frames |

Cost of the split, per tick: one `decide()` (~50 us, pure) and one status publish.
The planner does ~2 ms of work in a 50 ms budget.
