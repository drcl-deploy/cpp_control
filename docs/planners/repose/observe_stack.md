# The observe stack, and the tool that shows every layer of it

Status: **design + handoff**, 2026-09-03. Nothing here is built yet.
Read with [`hardware_cut_face_scan_loop.md`](hardware_cut_face_scan_loop.md) (the incident)
and [`planner.md`](planner.md) (the loop above this stack).

## Why

`pose_ok` is 0/1901 on a 95 s bag while colour is unanimous. Three `min_visible`
trials (0.60 -> 0.55 -> 0.40) changed nothing. **The gate is not the disease.**

Geometry says a clean bottom-boundary crop admits at these thresholds:

| `min_visible` | admits a cube centred at |
|---|---:|
| 0.60 | >= 0.346 m |
| 0.55 | >= 0.315 m |
| 0.40 | >= 0.224 m |

0.224 m is between the robot's feet. **If 0.40 still says `cut_face`, `vis` is
not measuring a border crop** — something upstream hands `minAreaRect` a sliver.
We cannot tell which, because the loop publishes the VERDICT and not the number.

Three live hypotheses. Each dies at a different layer, so the tool must expose
every layer, not the ends:

| # | hypothesis | dies at | tell |
|---|---|---|---|
| H1 | border crop (cube too close, cut by image bottom) | L7 band + L9 fit | mask touches the bottom row; `vis` ~ `(d+0.02)/0.61` |
| H2 | **`z_top` set by something higher than the cube** — the G1's own hands are in a 45 deg-down head cam's view | L6 | `z_top` sits above cube height; band is a RIM, not a face |
| H3 | depth dropout on the near part of the face (min-Z, grazing incidence) | L4 | labelled pixels present, `pts_` missing there |

H2 is the leading suspect. **No camera on the market fixes H2.**

## The stack

`env:=real` runs the MASK read. `edge` = 2 * `half_extent` = 0.6096 m.
All geometry is in the **gravity-aligned robot base frame** (x fwd, y left, z up).

```
 L0  WIRE        colour BGR + depth f32 (m) + Intrinsics per plane   node.cpp recv loop
 L1  RESIZE      proc_width (real: 0 = native); colour -> depth size sight.cpp:153
 L2  CLASSIFY    -> labels_ (H,W) int16 in {-1..5}                   sight.cpp:94
                 gates min_value 30, min_rel_sat 0.15, chroma_reject 0.06
                 nearest of 12 chromaticity refs in (r,g)
 L3  RAYS        pinhole (u-cx)/fx, -(v-cy)/fy, -1   NO DISTORTION   sight.cpp:130
 L4  UNPROJECT   labelled px, finite z > z_min_m -> pts_ (n,3)       sight.cpp:340
                 p_base = R_bc*(ray*z) + t_bc ; R,t = FK(waist) + IMU  node.cpp:241
 L5  HINT        atan2(mean y, mean x) over ALL live points          sight.cpp:365
                 survives every rejection below
 L6  Z_TOP       97th pct (mask_top_pct) of pts z, ACROSS ALL COLOURS sight.cpp:376
 L7  BAND        |z - z_top| <= mask_band_m (0.12) -> binary image   sight.cpp:378
 L8  COMPONENT   largest connectedComponents, erode(mask_erode 1)    sight.cpp:386
 L9  FIT         minAreaRect over surviving (x,y); plane_normal -> up_dot  sight.cpp:423
 L10 GATES       up_dot>=0.90 | big<=1.4 | vis>=0 (color_ok) | vis>=min_visible (ok)
 L11 SIGHT       color=modal, pos=(rect.cx, rect.cy, median z - half_extent),
                 phi=fold(rect.angle), n_px                          sight.cpp:445
 L12 BELIEF      push iff omega_cam < omega_still; ring 8; vote      node.cpp:406
 L13 DECIDE      3 branches -> STILL(yaw) | CLIP                     clips.cpp
```

BLOBS differs only at L6-L9: per-colour loop, top slab =
`(min_px-th highest z) - slab_frac*edge`, one rect per colour, **highest wins**.

### What is exposed today

| | published | where |
|---|---|---|
| L0 colour | `/vibe/planner/observe/color/compressed` | observe node ONLY |
| L0 depth | `/vibe/planner/observe/depth` + `/camera_info` | observe node ONLY |
| L2 labels | `/vibe/planner/observe/labels` | observe node ONLY |
| L9/L10 per-candidate `up_dot, vis, big, cx, cy, n_px` | `CubeObservation.candidate_*` | observe node ONLY |
| L11 result | `CubeObservation` / `PlannerStatus` | both |
| L12/L13 | `PlannerStatus` | planner node |

**Gaps that matter:**

1. **L6, L7, L8 are invisible everywhere.** H2 and H3 live there.
2. `PlannerStatus` carries `reason` but **not `vis` / `up_dot` / `big`**. During a
   real run you are blind to the number you are tuning.
3. On the BLOBS path a `cut_face` returns before `s.pos` is set, so `range_m` is
   **0** for every rejected sample. You cannot tell how close the cube was.
4. No **border adjacency**: nothing distinguishes "mask edge is a cube edge" from
   "mask edge is the image boundary". This is the single most valuable missing bit —
   see Decisions D2/D3.
5. No achieved-vs-commanded yaw, so SCAN performance on hardware is unmeasured
   (28.5 deg/90 is a SIM number).

## The tool

**Goal: one window, every layer, one frame at a time, live or from a bag.**

Build on what exists — do NOT start a new viewer. `scripts/planners/repose/repose_observe_viewer.py`
already subscribes to colour/depth/labels/`CubeObservation` and tiles them.

### Node side — 3 new image topics + 3 status fields

| topic | encoding | shows |
|---|---|---|
| `/vibe/planner/observe/band` | mono8 | L7: which pixels are within `mask_band_m` of `z_top` |
| `/vibe/planner/observe/mask` | mono8 | L8: the surviving component after erode |
| `/vibe/planner/observe/height` | mono8 (colormapped) | L4: per-pixel base-frame z, so `z_top` is READABLE |

Add to `PlannerStatus`: `float32 vis`, `float32 up_dot`, `float32 big`, and
`float32 scan_yaw_achieved` (IMU delta over the scan vs `plan.yaw_offset`).
`last_` in `sight.cpp` already holds the first three — they are simply not copied out.

### Viewer panels

```
 +---------------+---------------+---------------+
 | 1 RGB         | 2 DEPTH       | 3 HEIGHT z    |   z_top drawn as a contour
 |   + fit rect  |   colormap    |   colormap    |   <- H2 dies or survives here
 +---------------+---------------+---------------+
 | 4 LABELS      | 5 BAND        | 6 MASK        |   border pixels drawn RED
 |   6 colours   |   L7          |   L8 + rect   |   <- H1 dies or survives here
 +---------------+---------------+---------------+
 | 7 TOP-DOWN (x,y) base frame: points, minAreaRect, the 0.6096 m square       |
 |   the robot at origin, the gaze band 0.59-1.13 m as two arcs                |
 +-----------------------------------------------------------------------------+
 | 8 NUMBERS  vis up_dot big n_px z_top range bearing phi reason               |
 |            omega_cam accept_rate votes/reads mode label                     |
 +-----------------------------------------------------------------------------+
```

Panel 7 is the one that ends the argument: it shows the point set, the rect that
was fitted to it, and the square that SHOULD fit — side by side, in metres.

### Run

```bash
ros2 launch cpp_control g1_repose_planner_calibration.launch.py env:=real   # live
ros2 run  cpp_control repose_observe_viewer.py                               # panels
# offline, no robot:
ros2 run  cpp_control repose_replay_observe.py --bag <bag> --config <yaml>
```

The replay path matters more than the live one: **the failing bag already exists**
(`bags/02Sep2026_13_57/`). The tool must answer H1/H2/H3 off that bag with no robot.

## Decisions taken this session

| id | decision |
|---|---|
| D1 | **`vis` is one number for three diseases.** Border crop, depth dropout and segmentation loss all shrink `min(w,h)`. No threshold can separate them — stop tuning `min_visible` |
| D2 | **Border adjacency is free and is the key missing bit.** `idx_[n]` is still the pixel index, so `row = idx/cols, col = idx%cols`. Mark mask pixels on the image boundary; exclude those boundary segments from the fit. Then `vis` means "fraction of the square's boundary that was REAL" |
| D3 | **A border-cut face is reconstructable.** With phi from a real edge and the known 0.6096 m side, slide the rect to full extent AWAY from the crop. Recovers the centre and gives the bias SIGN |
| D4 | **`cut_face` already fills `s.pos`** on the mask path (`color_gate()` is 0.0, so the early return never fires). A cut read carries a usable range/bearing — enough to APPROACH, not enough to warp |
| D5 | **Gate the approach on `color_ok`, not `pose_ok`.** Weakest read that suffices for the act — the same principle as `min_visible_color = 0`. Dissolves the "can't size a reach without a pose" deadlock |
| D6 | **The cut bias points the WRONG WAY.** Cut at the near edge => surviving strip is the FAR half => range OVER-estimated by ~`(1-vis)*edge/2` (0.18 m at vis 0.4, not the 0.08 m in planner.md). Approach naively and you step TOWARD a cube already too close. Must sign-correct, and the sign comes from D2 |
| D7 | **`phi` from a cut rect is corrupt, not merely biased.** The hull's long axis snaps to the crop boundary, and `phi` feeds retrieval's SE(2) cost AND the warp. Lowering `min_visible` retrieves against the image border's angle |
| D8 | **`minAreaRect` ignores the strongest prior available.** It estimates 5 params (cx,cy,w,h,theta) from hull extremes with no residual, when the truth is 3 (x,y,phi) with a KNOWN 0.6096 m side over thousands of points. Replace with a constrained square fit once D2 lands |
| D9 | **A wider-FOV camera does not fix this.** 42->58 deg V moves the whole-face near edge 0.59 -> 0.49 m; the alphabet's entry band starts at 0.40. ~70 deg is needed, at ~3.3x fewer pixels on the face, with no distortion model (`build_rays` is pure pinhole and the error is worst at the border, which is exactly where cut faces live). And a wider colour FOV puts MORE arm in frame — worse under H2 |
| D10 | **A pitch scan beats the lens at the near end, for free.** Gaze 45 deg covers 0.59-1.13 m; adding 55 deg (waist pitch +10) covers 0.47-0.64. Union **0.47-1.13 m, contiguous**, vs a 58 deg lens's 0.49-1.93. The far reach is range the alphabet never uses. Direct twin of v7, which was also a gaze fix |
| D11 | **SCAN's reference contains no gait.** `writer.cpp:210-223` memcpys ONE `stand_row` `hold` times and rotates only the anchor quat — not a single joint channel. The reference asks "hold this static stance AND rotate the root 90 deg", which is physically inconsistent. It is the only mode whose reference is not real recorded frames. Fix = a recorded turn-in-place take, windowed by yaw, exactly as the walk take is windowed by distance |
| D12 | **REACH is CHEAPER on hardware than in sim.** The tokenizer reads `jp/jv/root ORIENTATION` only, so root position is unobservable and the sim's robot-anchored SE(2) warp is not needed at all. A hardware reach = turn to bearing, play window k, stop. Also: planner.md's stated reason for REACH's absence ("the robot has a real locomotion layer above it") is **false for this launch** — `g1_repose_planner.launch.py` -> `g1_vibe_sonic.launch.py` composes no locomotion |

## Order of work

Instrument, then diagnose, then fix. Do not reorder.

1. **`vis`/`up_dot`/`big` onto `PlannerStatus`** — ~10 lines, `last_` already holds them
2. **band / mask / height topics + the 7-panel viewer** — this doc's tool
3. **Answer H1 vs H2 vs H3 off `bags/02Sep2026_13_57/`** — no robot needed
4. **D2 border adjacency, then D3 un-crop** — turns `cut_face` into a degraded-but-usable read
5. **D5 approach on `color_ok`** (needs a `Belief` accessor for "freshest agreeing read that
   carried a colour" — it has `pose()` requiring `ok`, and `newest()` meaning anything, but
   nothing in between), with the **D6 sign correction**
6. **D11 turn take**, and **D12 walk take**, baked into `ClipTable` beside the 78 clips
7. D8 constrained square fit, D10 pitch scan — only if 3-6 leave residue

## Cross-repo notes

Training twin lives in `vibe` at `src/vibe/tasks/repose/planner/` (`sight.py` is
`sight.cpp`'s twin; `privileged.py` is the oracle's closed-form version of the same
three gates). Parity ledger: vibe `docs/planner/repose.md`.

Two fixes exist in the vibe twins and were **never ported here** — unrelated to this
incident, but they are open:

| gap | vibe | here |
|---|---|---|
| `has_hint` (no hint => sweep FULL, not a 22.5 deg ratchet) | `clips.py`, `batch.py` | **absent** |
| `settle_ready` (SETTLE ends on evidence, not on its 40-frame clock) | `clips.py:177`, `batch.py` | **absent** — the ACT boundary is `controller_.finished`, `node.cpp:511` |

vibe `docs/planner/repose.md` claims both landed on both twins. They did not.

Also: the vibe-side yaml (`config/repose_planner/g1_sim.yaml`) and this one do not
cross-load (`seam.lead_in_rate` is deployment-only; `clips`/`frames`/`camera`/`mount`
are ROS-only). Deliberate — but there is **no `g1_real.yaml` on the vibe side**, so the
`mask` read that actually ships has no simulated twin.
