#!/usr/bin/env bash
# Unattended sim2sim for the difftrack tracker: a simulator + the controller, no
# joystick, one SUMMARY line per run.
#
#   bash scripts/run_difftrack_sim2sim.sh [options] [motion ...]
#
#   -W WORKFLOW  which plant, and therefore which message path — see WORKFLOWS
#                below. unitree (default); drcl is gone with its workspace.
#   -w           WATCH: bring both processes up, leave the MuJoCo viewer
#                running and track until Ctrl-C, streaming the controller's
#                own lines to this terminal. One motion, no summary — this is
#                the mode to use when you want to LOOK at the transfer rather
#                than measure it. Everything else is held the same.
#   -s           STAND CYCLE: the operator-driven session, and the one that
#                matches what happens on a robot. The controller boots into its
#                REST state and holds the robot there; YOU press `A` and it
#                plays the clip for -d seconds, fades its gains back out and
#                returns to rest; press `A` again for another run. Nothing is
#                automated, nothing exits on its own, and the viewer stays up
#                until Ctrl-C. Implies -w (one motion, no summary).
#   -e MODE      entry: rsi (default) or stand   — see below
#   -d SECONDS   tracking budget: per measured run, or per `A` press under -s
#                                                (default 20; ignored under -w)
#   -r N         repeats per motion              (default 1; ignored under -w)
#   -l SECONDS   lead-in: ramp the reference from rest onto the clip's entry
#                velocity (rsi entry only)       (default 0)
#   -p SECONDS   entry ramp onto the clip's first frame, stand entry only;
#                0 (default) engages straight from the nominal pose, which
#                measures far better — see below
#   -f           free-run the simulator instead of pacing it to the wall clock.
#                Faster wall-clock sweeps and IDENTICAL physics -- the policy
#                gets its 20 ms of physics per control step either way, because
#                state_decimation ties the control loop to the state stream --
#                but the viewer plays at 1.4x-1.8x and the gait looks like a jog.
#                -W drcl only.
#   -R           RECORD an mp4 of the viewer to cpp_control/recordings/, one
#                file per run, named after the motion. Equivalent to pressing
#                F9 in the viewer at t=0 and again at the end; F9 still works
#                by hand. The video is clocked on SIMULATION time, so it plays
#                back at 1x even under -f. Both plants: run_mj_sim.py's
#                recorder under drcl, unitree_mujoco's F9 recorder (from
#                workflows/patches/) under unitree.
#   -S PATH      -s only: back the REST state with a SONIC stand POLICY instead
#                of the nominal-pose PD hold. A path under models/ (e.g.
#                tracker/sonic/g1_sonic_base.onnx) or an absolute one.
#   -b SECONDS   MOTION BLEND IN: for the first SECONDS, show the policy the clip
#                plus the robot's own disagreement with clip frame 0, faded out
#                on a smoothstep -- so the reference starts exactly on the pose
#                the robot is standing in rather than on a frame it is a clip's
#                worth of mid-stride away from. The clip's clock still runs at 1x
#                throughout: nothing is delayed and nothing is played slow, which
#                is what separates this from -l and is why it works where -l does
#                not. Around 1 s is the measured optimum and it is a real one --
#                on g1_fight, 0.5 s and 1.0 s reach clip step 114 and 113 against
#                73 with no blend, while 1.5 s and 2.5 s drop back to 61 and 58.
#                Off (0) by default -- see MOTION BLENDS below.
#   -o SECONDS   MOTION BLEND OUT: over the last SECONDS of the run, interpolate
#                the REFERENCE off the clip and onto a standing frame -- the
#                policy's default pose, upright on the clip's heading, at the
#                height that pose stands at -- so the rest state is handed a
#                robot that has been asked to stop rather than one cut off
#                mid-stride. The mirror of -b, and the reference-side counterpart
#                to -B, which moves the arms AFTER the handover. Unlike EXIT_HOLD
#                it gives the policy somewhere to go instead of freezing it on a
#                single-support frame: on g1_dance15s it takes the robot from 2.2
#                to 1.2 rad away from the stand's pose at the handover, with the
#                run itself unharmed (650/650 clean, mean error 0.138 m against
#                0.175 m without). Off (0) by default.
#   -B SECONDS   -s only: once the stand has taken the robot, interpolate the
#                ARM position targets onto it over this long instead of snapping
#                them there in one control period. Off (0) by default. The clip
#                is untouched: it plays to its last frame with the policy owning
#                every joint, and only then does this start. Arms only, and
#                positions only -- gains, legs and waist go to the stand on the
#                handover tick either way. For a clip that does not end where
#                the stand's nominal pose is: g1_dance30s ends 1.4-2.8 rad away
#                at the arms, and it looks like the snap it is.
#   -E MODE      where the WORLD BASE POSE comes from — see WORLD STATE below.
#                estimator (default) | ground_truth | compare | mixed
#   -H           HEADLESS: unitree_mujoco with no viewer window — no GLFW, no
#                display, no GL context on the GPU. Same physics, same 1x
#                pacing, same numbers; for sweeps. Needs the binary built with
#                workflows/patches/unitree_mujoco-headless.patch. Not with -R.
#   -k           keep the logs and say where they are
#   motion ...   directories under models/tracker/difftrack (default: all)
#
# MOTION BLENDS
#
#   BOTH ARE OFF UNLESS YOU ASK FOR THEM, on every clip, and there is no
#   per-motion default anywhere: -b and -o replace the reference the tracking
#   error is measured against, so a run taken with one is not comparable with a
#   run taken without one -- and every number in
#   docs/trackers/difftrack_running.md and recordings/icra_q1/ was taken without.
#   A default that switched itself on for some clips and not others would make
#   the column mean two different things down the same table.
#
#   The clip they were built for is g1_fight, and it is worth knowing what they
#   do and do not buy it. From the rest state that clip opens with a 170 deg spin
#   at up to 1.5 m/s; a standing robot cannot start that, loses the phase in the
#   first second and goes down at clip step 73 of 748. `-b 1.0 -o 1.0` removes
#   the step change at engage -- max root error 1.82 m -> 0.76 m, and the fall
#   moves out to step 113 -- but it does NOT make the clip survivable from a
#   stand. Under -e rsi, where the robot is teleported onto frame 0 already in
#   the motion, the same policy plays all 748 steps. See
#   docs/trackers/difftrack_running.md.
#
# THE RUN LOG
#
#   Every run also writes an npz of its own control steps -- the reference, the
#   state, the action and the command, one row per step -- into
#   cpp_control/recordings/runs/, tagged `sim2sim_<-E mode>`. The SUMMARY line
#   this script prints is four numbers out of that file; the file is what a
#   hardware run is compared against later, because the robot writes the same
#   one from the same node.
#
#       python3 scripts/analyze_tracking.py recordings/runs/*.npz
#       python3 scripts/analyze_tracking.py --compare <sim>.npz <robot>.npz
#
#   RUN_LOG=0        turn it off
#   RUN_LOG_DIR=DIR  put the files somewhere else
#
#   See docs/run_logs.md.
#
# THE FALL CUT-OUT
#
#   A run ends -- and the robot goes limp in DAMPING -- the first tick the root
#   drops below the export's termination height (0.3 m for the g1 clips).
#   FALL_HEIGHT=0 turns that off, FALL_HEIGHT=0.2 just lowers it; see the notes
#   at the assignment below.
#
# WORLD STATE
#
#   These policies observe an ABSOLUTE world pose and twist (they were trained
#   with global_obs=true), so something has to supply one. The simulator can
#   hand over ground truth. A robot cannot, and a lab with no motion capture has
#   only what the robot can work out for itself.
#
#   That gap is why -E defaults to `estimator` rather than to the ground truth
#   this script used to measure against: a number measured on a pose no robot
#   can produce is an upper bound, not a transfer result.
#
#   estimator     (default) docker/estimator publishes nav_msgs/Odometry on
#                 /odom from legged_control2's contact-aided Kalman filter over
#                 the robot's own IMU and encoders, and the controller runs on
#                 THAT. The same estimate, from the same container, that the
#                 robot will run. Needs the image built once:
#                     bash docker/estimator/build.sh
#                 g1_jumps*/g1_run* run with the parameters tuned for jumping
#                 and running (docker/estimator/.../config/g1_dynamic.yaml,
#                 mounted with run.sh -c); everything else with the stock
#                 g1.yaml. ESTIMATOR_PARAMS=stock|<file> overrides that.
#                 The controller's world-state guard (world_guard:=true,
#                 WorldStateGuard in difftrack_obs.hpp) holds a glitching
#                 estimate out of the observation on every clip.
#
#   ground_truth  the simulator's own pose (SportModeState + IMU), which is what
#                 this script measured before. Still the right mode for
#                 separating a policy problem from an estimator problem: if a
#                 clip fails under `estimator` and survives here, the estimator
#                 is what changed.
#
#   compare       the controller runs on GROUND TRUTH while the estimator runs
#                 alongside untrusted, and scripts/compare_odom_ground_truth.py
#                 scores one against the other. Run this FIRST on any new clip:
#                 it tells you what the estimator costs without letting it drive.
#
#   mixed         ONE CHANNEL AT A TIME. The policy reads four things out of the
#                 world state that the IMU does not give it -- pos_xy (under
#                 every tar_obs frame), pos_z (root height), vel_xy and vel_z --
#                 and `estimator` swaps all four at once. ODOM_MIX picks a
#                 source per channel; unnamed ones stay on ground truth:
#                     ODOM_MIX="vel_z=est" bash scripts/run_difftrack_sim2sim.sh -E mixed ...
#                 scripts/odom_channel_mixer.py does the mixing, and logs ground
#                 truth and the estimate side by side, one csv per run, to
#                 MIX_CSV_DIR (default <run log dir>/odom_mix). With every
#                 channel on `est` it is `estimator` plus that log.
#
#   See docs/trackers/difftrack_state_estimation.md for what the estimator can
#   and cannot observe (height and tilt yes; x, y and heading drift).
#
# WORKFLOWS
#
#   Two different plants AND two different message paths, and the second half is
#   why both exist. Everything above the transport — the policy, the observation
#   builder, the entry logic, the reference clip — is the same code either way.
#
#   drcl     GONE. mj_sim, `messages/G1State` and the `assets` package lived in
#            a second colcon workspace (drcl/) that this package no longer sits
#            in; cpp_control now builds inside unitree_ros2/cyclonedds_ws
#            against unitree_hg alone. The numbers it produced are still the
#            reference in docs/trackers/difftrack_running.md, and the code that
#            produced them is still in this script and in git — reinstate the
#            three packages and it runs again. -W drcl says so rather than
#            failing somewhere inside a missing `import mj_sim`.
#
#   unitree  (default) unitree_mujoco, over unitree_hg LowState/LowCmd on CycloneDDS,
#            config g1_difftrack_unitree.yaml. THE HARDWARE PATH: the same
#            transport, the same IDL, the same motor order, the same CRC and the
#            same mode_machine the real G1 uses, so the half of the stack that
#            g1_difftrack_hw.yaml selects is exercised before it is pointed at a
#            robot. Needs workflows/unitree.md installed; this script says so if
#            the binary is missing.
#
#            One thing it does not have, belonging to run_mj_sim.py rather than
#            to the plant: a speed override (-f). Its viewer is MuJoCo's stock
#            `simulate`, which paces itself to 1x and reports the ratio it is
#            actually achieving. -R works here too, via the F9 recorder that
#            workflows/patches/ adds to unitree_mujoco's own main.cc. The generated scene adds
#            a `track` camera (mode trackcom) AND selects it as the scene's
#            initial camera, because simulate's free camera does not follow and
#            a G1 at 1 m/s leaves the frame in about four seconds. It holds a
#            constant offset from the robot, so the robot stays put on screen
#            and the background moves. `[` / `]` cycles, Esc frees the camera.
#
#            The numbers are NOT expected to match drcl's, and a difference is
#            not a bug in either. It is a different MuJoCo model (unitree's own
#            g1_29dof, with its own armature, damping and joint friction) on a
#            different integrator, driven through a PD loop that lives in
#            unitree_mujoco's 1 kHz bridge thread rather than in the simulator's
#            own actuators.
#
# ENTRY MODES
#
#   rsi    The comparison against diffsimrl's own numbers. The scene is
#          generated with the robot standing ON the clip's first frame —
#          reference-state initialisation, which is how these policies were
#          trained and evaluated — so the only things the deploy stack changes
#          are the plant and the controller. Everything else is held equal.
#
#          One thing RSI cannot carry across: velocity. MuJoCo resets qvel to
#          zero and g1_walk enters at 0.97 m/s, so the robot starts in the right
#          pose but at rest. `-l 0.5` ramps the reference onto the clip's speed
#          instead of handing the policy a 1 m/s error on step 0.
#
#   stand  The realistic entry: the robot starts in the model's own pose, the
#          controller holds the policy's nominal pose, then anchors the clip to
#          wherever the robot is standing and tracks it in the clip's own frame.
#          This is the hardware sequence.
#
#          What does NOT work is `-p 2`: ramping the joints onto the clip's
#          first frame before engaging. Every shipped clip starts mid-stride, in
#          single support, and statically posing a standing robot into that
#          topples it (measured: falls at step 0). Engaging from the nominal
#          pose and letting the policy find the clip works instead.
#
# Under -W drcl both modes disable the scene's `world_root` weld (see
# make_sim2sim_scene.py): the shipped scene hangs the robot in the air for
# bring-up, and anything a weld holds up is not a transfer result. unitree's
# scene has no weld; its equivalent is the elastic band, which unitree_mujoco's
# config.yaml ships disabled and this script does not enable.
#
# A run is clean when fell_at=none. mean_err is the root tracking error in
# metres against the reference the policy was actually given.
#
# A LIMP ROBOT IS A PROCESS PROBLEM, NEVER A POLICY ONE, under both workflows.
# mj_sim re-reads the publisher count on /robot_command every iteration and,
# finding none, drives kp=0 kd=0.1 on all 29 joints
# (RosNode.check_topic_publishers). unitree_mujoco does not even check: its
# bridge applies the last LowCmd it received, and a default-constructed one is
# kp=kd=tau=0, so a G1 with no controller is limp there too. Either way, the two
# ways to watch a heap on the floor are a controller that never started and a
# controller that has already exited, and this script refuses the first and does
# not linger on the second.
set -eo pipefail

ENTRY=rsi
DURATION=20
REPEATS=1
LEADIN=0.0
ENTRY_RAMP=0.0
KEEP=0
WATCH=0
STANDCYCLE=0
STAND_ONNX=""
ARM_BLEND=0.0
BLEND_IN=0.0
BLEND_OUT=0.0
FREERUN=0
RECORD=0
HEADLESS=0
WORKFLOW=unitree
# Where the WORLD BASE POSE comes from, and it defaults to the honest one.
#
# These policies observe an absolute world pose and twist (global_obs=true).
# The simulator can hand over ground truth, and for a long time that is what
# this rig measured — but no robot can, and a lab with no motion capture has
# only the onboard estimator. Measuring against ground truth therefore reports
# a transfer that does not exist on hardware, so `estimator` is the default and
# `ground_truth` is the deliberate opt-out.
#
#   estimator     docker/estimator's contact-aided KF on /odom, the same
#                 estimate the robot will run. THE DEFAULT.
#   ground_truth  the simulator's own pose (SportModeState + IMU). The old
#                 behaviour; an upper bound on what the policy can do, and not
#                 reachable on hardware.
#   compare       the controller runs on GROUND TRUTH while the estimator runs
#                 alongside, and scripts/compare_odom_ground_truth.py scores one
#                 against the other. This is how you find out what the estimator
#                 costs before letting it drive.
ESTIMATOR=estimator
while getopts "e:d:r:l:p:W:S:B:b:o:E:kwsfRHh" opt; do
  case $opt in
    e) ENTRY=$OPTARG ;;
    d) DURATION=$OPTARG ;;
    r) REPEATS=$OPTARG ;;
    l) LEADIN=$OPTARG ;;
    p) ENTRY_RAMP=$OPTARG ;;
    W) WORKFLOW=$OPTARG ;;
    E) ESTIMATOR=$OPTARG ;;
    k) KEEP=1 ;;
    w) WATCH=1 ;;
    s) STANDCYCLE=1; WATCH=1 ;;
    S) STAND_ONNX=$OPTARG ;;
    B) ARM_BLEND=$OPTARG ;;
    b) BLEND_IN=$OPTARG ;;
    o) BLEND_OUT=$OPTARG ;;
    f) FREERUN=1 ;;
    R) RECORD=1 ;;
    H) HEADLESS=1 ;;
    h) sed -n '2,/^set -eo pipefail/p' "$0" | sed '$d'; exit 0 ;;
    *) exit 2 ;;
  esac
done
shift $((OPTIND - 1))
case "$ENTRY" in rsi|stand) ;; *) echo "unknown entry mode '$ENTRY'"; exit 2 ;; esac
case "$WORKFLOW" in
  unitree) ;;
  drcl)
    echo "-W drcl needs the drcl_deploy workspace (mj_sim + messages + assets), and this"
    echo "  package does not live in one any more: cpp_control is now a package inside"
    echo "  unitree_ros2/cyclonedds_ws and builds against unitree_hg alone."
    echo "  Everything the drcl plant did is still in this script and in git history."
    echo "  Run the unitree plant instead (the default):  $0 -d $DURATION $*"
    exit 2 ;;
  *) echo "unknown workflow '$WORKFLOW'"; exit 2 ;;
esac
case "$ESTIMATOR" in
  estimator|ground_truth|compare|mixed) ;;
  *) echo "unknown -E mode '$ESTIMATOR' (estimator | ground_truth | compare | mixed)"; exit 2 ;;
esac
# -E mixed: which of the four estimator-fed channels come from the estimate.
# The run tag names the channels on `est`, so a directory of a sweep reads.
RUN_TAG_MODE=$ESTIMATOR
MIX_ARGS=()
MIX_TAG=""
if [ "$ESTIMATOR" = "mixed" ]; then
  declare -A MIX=([pos_xy]=gt [pos_z]=gt [vel_xy]=gt [vel_z]=gt)
  for kv in ${ODOM_MIX//,/ }; do
    ch=${kv%%=*}; src=${kv#*=}
    case "$ch" in
      pos_xy|pos_z|vel_xy|vel_z) ;;
      *) echo "ODOM_MIX: unknown channel '$ch' (pos_xy pos_z vel_xy vel_z)"; exit 2 ;;
    esac
    case "$src" in
      gt|est) ;;
      *) echo "ODOM_MIX: $ch must be gt or est, got '$src'"; exit 2 ;;
    esac
    MIX[$ch]=$src
  done
  est_ch=()
  for ch in pos_xy pos_z vel_xy vel_z; do
    MIX_ARGS+=("--${ch/_/-}" "${MIX[$ch]}")
    if [ "${MIX[$ch]}" = "est" ]; then est_ch+=("$ch"); fi
  done
  if [ ${#est_ch[@]} -eq 0 ]; then MIX_TAG=none; else MIX_TAG=$(IFS=-; echo "${est_ch[*]}"); fi
  RUN_TAG_MODE="mixed_est-${MIX_TAG}"
elif [ -n "${ODOM_MIX:-}" ]; then
  echo "ODOM_MIX is only read under -E mixed (this is -E $ESTIMATOR)."; exit 2
fi
if [ "$HEADLESS" = "1" ] && [ "$RECORD" = "1" ]; then
  echo "-R records the viewer, and -H has none."; exit 2
fi
# The estimator listens to rt/lowstate, which only the unitree plant publishes.
if [ "$ESTIMATOR" != "ground_truth" ] && [ "$WORKFLOW" != "unitree" ]; then
  echo "-E $ESTIMATOR needs the unitree plant (it reads LowState)."; exit 2
fi
if [ -n "$STAND_ONNX" ] && [ "$STANDCYCLE" != "1" ]; then
  echo "-S is the stand cycle's rest-state policy; it does nothing without -s."
  exit 2
fi
if [ "$ARM_BLEND" != "0.0" ] && [ "$ARM_BLEND" != "0" ] && [ "$STANDCYCLE" != "1" ]; then
  echo "-B interpolates the arms onto the REST state, and only -s has one."
  exit 2
fi
SPEED=1.0
[ "$FREERUN" = "1" ] && SPEED=0
# The lead-out, and both halves are env-overridable so a session can be tuned
# without editing this script:
#   EXIT_HOLD  seconds with the reference clock frozen -- the policy braking
#   EXIT_RAMP  seconds fading the gains back to the yaml's hold gains
# Both default to 0 -- hand the robot over on the tick the clip ends -- and that
# is a measurement, not a shrug. See the notes in finish(): every tick spent
# freezing the reference or fading gains is a tick the robot is NOT being
# balanced, and it ends a walk clip at ~1 m/s. With the SONIC stand as the rest
# state, 0/0 stands the robot up and 1.0/0.5 puts it on the floor.
EXIT_HOLD=${EXIT_HOLD:-0.0}
EXIT_RAMP=${EXIT_RAMP:-0.0}

# The fall cut-out. The node watches the root height every tick and, the first
# time it drops below this, calls it a fall: the run ends and the control mode
# goes to DAMPING, which is the robot going limp on the floor. The threshold
# comes from the export (`termination_height` in difftrack_config.json, 0.3 m
# for the g1 clips) unless something overrides it, and <0 here means exactly
# that -- keep the export's number.
#
#   FALL_HEIGHT=0    disable the cut-out entirely: no fall is ever declared, no
#                    DAMPING, and the policy keeps being asked to track from
#                    wherever the robot ended up. This is what you want when a
#                    clip legitimately goes low -- a jump landing, a crouch --
#                    or when you want to WATCH the recovery instead of having
#                    the run cut at the moment it got interesting. It also
#                    means a genuine fall no longer stops anything, so the
#                    SUMMARY's fell/fell_at columns go quiet.
#   FALL_HEIGHT=0.2  keep the cut-out, just lower.
FALL_HEIGHT=${FALL_HEIGHT:--1.0}

# The per-step run log (docs/run_logs.md). On by default and in the same place
# every time, because the comparison this rig exists to support is a sim2sim run
# against a hardware one, and that is only possible if the sim2sim runs were
# kept. RUN_LOG=0 turns it off; RUN_LOG_DIR moves it.
RUN_LOG=${RUN_LOG:-1}

# One-off launch arguments, appended after everything this script sets, so they
# WIN over it -- `ros2 launch` takes the last spelling of a repeated argument.
# For trying a parameter the script has no flag for without editing it:
#
#   EXTRA_LAUNCH_ARGS="observe_in_reference_frame:=true anchor_yaw_to_robot:=true" \
#       bash scripts/run_difftrack_sim2sim.sh -e rsi g1_fight
#
# Deliberately unvalidated and deliberately last: it is an escape hatch for an
# experiment, not a supported configuration, and anything worth keeping belongs
# in a flag.
read -r -a extra_args <<< "${EXTRA_LAUNCH_ARGS:-}"

# The stand cycle is a STAND entry by construction: the robot is standing in the
# rest state when you press A, not posed on the clip's first frame. Asking for
# rsi here would generate a scene with the robot already mid-stride and then
# hold it there with the stand -- which is neither entry.
if [ "$STANDCYCLE" = "1" ]; then
  ENTRY=stand
fi

# -f belongs to run_mj_sim.py, not to the measurement. Under the unitree
# workflow the viewer is MuJoCo's own stock `simulate` and has no speed
# override. Refusing is the point: accepting it and quietly doing nothing is how
# a run gets reported at "1x" when nothing was pacing it.
#
# -R is supported on BOTH plants now. Under unitree it is the F9 recorder in
# unitree_mujoco's main.cc (workflows/patches/unitree_mujoco-f9-recording.patch)
# driven by DRCL_RECORD_AUTOSTART, which is F9 pressed at t=0 and again at the
# end -- the same contract -R has always had.
if [ "$WORKFLOW" = "unitree" ]; then
  if [ "$FREERUN" = "1" ]; then
    echo "-f is a run_mj_sim.py flag; unitree_mujoco has no speed override."
    echo "  Its viewer is MuJoCo's stock simulate, which paces to 1x and shows the"
    echo "  ratio it is actually achieving in its own panel."
    exit 2
  fi
fi

# The backstop has to cover a run that now takes its DURATION in REAL seconds,
# plus the pose ramp and the settle dwell before tracking starts.
TIMEOUT=$((DURATION * 3 + 120))

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd "$HERE/.." && pwd)"
ESTIMATOR_IMAGE=${ESTIMATOR_IMAGE:-g1-estimator}
# The estimator's parameters. Jumps and run get the file tuned on them
# (recordings/estimator_tuning/) whenever it exists; every other clip keeps the
# image's stock g1.yaml. ESTIMATOR_PARAMS overrides the choice for every clip:
#   auto (default) | stock | <path to a legged_odom yaml>
DYNAMIC_ESTIMATOR_PARAMS=$PKG/docker/estimator/ws/src/legged_odom/config/g1_dynamic.yaml
ESTIMATOR_PARAMS=${ESTIMATOR_PARAMS:-auto}
# The middleware is chosen HERE, before anything starts. `setup.sh` in sim mode
# switches to rmw_cyclonedds_cpp on `lo` — without that the controller and
# unitree_mujoco are on different middlewares, every node starts cleanly, and not
# one message is ever delivered.
#
# `sim` is passed explicitly and is not negotiable for this script: it starts
# unitree_mujoco with `-n lo` further down, so a shell left in robot mode by a
# previous `source setup.sh robot` would put the two halves of the rig on
# different interfaces. Stating it here makes the script independent of whatever
# the calling shell was last set to.
source "$PKG/../../../setup.sh" sim > /dev/null

# Everything below that differs between the two plants, in one place.
if [ "$WORKFLOW" = "unitree" ]; then
  STATE_TOPIC=/lowstate
  CMD_TOPIC=/lowcmd
  CONFIG_PATH="$(ros2 pkg prefix cpp_control)/share/cpp_control/config/tracker/g1_difftrack_unitree.yaml"
  SIM_BIN="$UNITREE_MUJOCO/simulate/build/unitree_mujoco"
  if [ ! -x "$SIM_BIN" ]; then
    echo "no unitree_mujoco binary at $SIM_BIN"
    echo "  build it:  cd $UNITREE_MUJOCO/simulate && mkdir -p build && cd build && cmake .. && make -j4"
    echo "  see cpp_control/workflows/unitree.md"
    exit 1
  fi
  # -R needs the F9 recorder patch in the binary. Checked here rather than
  # discovered as a missing mp4 after the run: the string is in main.cc's
  # environment lookup, so it is present exactly when the patch is built in.
  # -H likewise needs its patch built in; the option's own name is in the binary
  # exactly when it is.
  if [ "$HEADLESS" = "1" ] && ! grep -qa "headless,H" "$SIM_BIN"; then
    echo "-H needs unitree_mujoco's --headless, and $SIM_BIN was built without it."
    echo "  Apply it and rebuild:"
    echo "      cd $UNITREE_MUJOCO"
    echo "      git apply $PKG/workflows/patches/unitree_mujoco-headless.patch"
    echo "      cd simulate/build && make unitree_mujoco -j4"
    exit 2
  fi
  if [ "$RECORD" = "1" ] && ! grep -qa DRCL_RECORD_AUTOSTART "$SIM_BIN"; then
    echo "-R needs the F9 recorder, and $SIM_BIN was built without it."
    echo "  Apply it and rebuild:"
    echo "      cd $UNITREE_MUJOCO"
    echo "      git apply $PKG/workflows/patches/unitree_mujoco-f9-recording.patch"
    echo "      cd simulate/build && make unitree_mujoco -j4"
    exit 2
  fi
else
  STATE_TOPIC=/robot_state
  CMD_TOPIC=/robot_command
  CONFIG_PATH=""   # the launch file's own default (g1_difftrack.yaml)
fi

# mj_sim's viewer is GLFW: under a Wayland session it takes the Wayland backend,
# fails to load libdecor's GTK plugin and segfaults on startup. Force the X11
# backend (XWayland is fine). Its --headless flag is not an alternative: mj_sim's
# reset() unconditionally touches self.viewer, which headless never creates.
export DISPLAY=${DISPLAY:-:1}
unset WAYLAND_DISPLAY XDG_SESSION_TYPE

MODELS="$(ros2 pkg prefix cpp_control)/share/cpp_control/models/tracker/difftrack"

# The run log, tagged with the world-state mode: a run measured against the
# simulator's ground truth and one measured against the onboard estimator are
# different measurements, and the tag is in every file name and every index
# line so a directory of both stays readable.
RUN_LOG_DIR=${RUN_LOG_DIR:-$PKG/recordings/runs}
record_args=(record:=false)
if [ "$RUN_LOG" = "1" ]; then
  record_args=(record:=true record_dir:="$RUN_LOG_DIR" run_tag:="sim2sim_${RUN_TAG_MODE}")
fi
MIX_CSV_DIR=${MIX_CSV_DIR:-$RUN_LOG_DIR/odom_mix}

# THE REST STATE for -s. Default to the SONIC stand, because it is the only
# thing here that can catch a robot that is still moving when the clip ends —
# measured: with it the G1 is standing at 0.785 m ten seconds after the
# handover, without it the nominal-pose hold has it on the floor.
#
# But only if the weights are actually on disk. Every .onnx under models/ is
# git-lfs-tracked (.gitattributes), and an unfetched one is a 130-byte text
# POINTER that exists, is readable, and is not a model. Fall back with a real
# explanation rather than starting a session whose rest state cannot hold.
if [ "$STANDCYCLE" = "1" ] && [ -z "$STAND_ONNX" ]; then
  _sonic="$(ros2 pkg prefix cpp_control)/share/cpp_control/models/tracker/sonic/g1_sonic_base.onnx"
  if [ -f "$_sonic" ] && ! head -c 40 "$_sonic" | grep -q "git-lfs"; then
    STAND_ONNX=tracker/sonic/g1_sonic_base.onnx
  else
    echo "no SONIC stand weights at $_sonic"
    echo "  -- falling back to the nominal-pose PD hold as the rest state."
    echo "  That hold cannot catch a robot that is still walking when the clip ends;"
    echo "  it holds a standing robot indefinitely and nothing more. To fix it:"
    echo "      cd $PKG && git lfs install --local && git lfs pull -I 'models/tracker/sonic/*'"
    echo "      bash workflows/conda_env/build.sh --packages-select cpp_control"
    echo
  fi
fi
if [ $# -gt 0 ]; then
  MOTIONS=("$@")
else
  mapfile -t MOTIONS < <(cd "$MODELS" && ls -d */ 2>/dev/null | tr -d /)
fi
if [ ${#MOTIONS[@]} -eq 0 ]; then
  echo "no exported motions under $MODELS"
  echo "  export one:  python scripts/export_to_drcl_cpp_control.py <run_dir> --out <pkg>/models/tracker/difftrack"
  echo "  then rebuild — the launches read the INSTALLED copy"
  exit 1
fi

# Watching is one robot in one window: sweeping is what the measured mode is for.
if [ "$WATCH" = "1" ]; then
  if [ ${#MOTIONS[@]} -gt 1 ]; then
    echo "-w watches one motion; you named ${#MOTIONS[@]}. Pick one of:"
    printf '  %s\n' "${MOTIONS[@]}"
    exit 2
  fi
  REPEATS=1
  KEEP=1
fi

LOGDIR=$(mktemp -d -t difftrack_sim2sim.XXXXXX)
# mj_sim's stock config, which the drcl plant is configured by copying. The
# unitree plant has no such file -- unitree_mujoco reads its own
# simulate/config.yaml and everything this script overrides is a CLI flag -- so
# do not even import mj_sim there.
if [ "$WORKFLOW" = "drcl" ]; then
  STOCK_CFG=$(python3 -c "
import os, mj_sim
print(os.path.join(os.path.dirname(mj_sim.__file__), 'config', 'G1.yml'))")
fi

# SIGTERM then SIGKILL: mj_sim does not reliably die on TERM (rclpy installs its
# own signal handling and the simulator's main loop is a bare `while True`), and
# a survivor keeps publishing /robot_state into the NEXT run.
# Both simulator spellings: this rig's runner, and a stale `ros2 run mj_sim main`
# left over from a hand-started session. Either one publishing /robot_state is
# enough to interleave two robots into the controller.
kill_sims() {
  local sig=$1
  pkill "$sig" -f "run_mj_sim.py" 2>/dev/null || true
  pkill "$sig" -f "lib/mj_sim/main" 2>/dev/null || true
  # unitree_mujoco is the same hazard under a different name: it survives an
  # interrupted run, keeps publishing rt/lowstate, and the next controller then
  # tracks two interleaved robots. Killed unconditionally, whichever workflow
  # this invocation is running, because a leftover from the OTHER one is
  # exactly the case that is hard to see.
  pkill "$sig" -f "simulate/build/unitree_mujoco" 2>/dev/null || true
}
ESTIMATOR_CONTAINER=difftrack-estimator
# The estimator is a CONTAINER, so it does not die with this shell and pkill
# cannot see it. A survivor publishing a stale /odom is the same hazard as a
# survivor publishing /robot_state, and worse in one way: the controller cannot
# tell a frozen estimate from a live one that happens to be standing still.
stop_estimator() {
  docker rm -f "$ESTIMATOR_CONTAINER" >/dev/null 2>&1 || true
}
start_estimator() {
  local log=$1 params=${2:-}
  local param_args=()
  if [ -n "$params" ]; then param_args=(-c "$params"); fi
  if ! docker image inspect "$ESTIMATOR_IMAGE" >/dev/null 2>&1; then
    echo "  no '$ESTIMATOR_IMAGE' image. Build it once:"
    echo "      bash $PKG/docker/estimator/build.sh"
    echo "  or measure against the simulator's ground truth instead:  -E ground_truth"
    return 1
  fi
  stop_estimator
  # Loopback: the whole rig is on this machine and setup.sh already pins DDS
  # to localhost, so the estimator has to be on `lo` too or it never hears
  # rt/lowstate. On the robot it is the robot's own interface — see
  # docs/trackers/difftrack_state_estimation.md.
  bash "$PKG/docker/estimator/run.sh" \
       -n "$ESTIMATOR_CONTAINER" -t "$ESTIMATOR_IMAGE" "${param_args[@]}" \
       -i "${ESTIMATOR_IFACE:-lo}" -D > "$log" 2>&1 || return 1

  # Then CHECK IT IS STILL THERE. `docker run -d` succeeds the moment the
  # container is created, so a node that dies on its first state message —
  # a bad URDF path, an unplaceable joint name, a model-layout mismatch —
  # exits 0 here and takes its log with it, because --rm has already removed
  # the container by the time anyone thinks to look. What that leaves behind
  # is a run with no /odom and a controller reporting a stale world pose,
  # which reads as a networking problem and is not one.
  local waited=0
  while [ "$waited" -lt 30 ]; do
    if ! docker ps --format '{{.Names}}' | grep -qx "$ESTIMATOR_CONTAINER"; then
      echo "  the estimator container exited immediately. Its log:"
      docker logs "$ESTIMATOR_CONTAINER" 2>&1 | grep -v "type hash" | tail -n 25 | sed 's/^/    /' \
        || echo "    (gone with --rm; rerun without -D:  bash $PKG/docker/estimator/run.sh -i lo)"
      return 1
    fi
    # Up is not the same as publishing. The node logs its banner once the model
    # and the joint map are resolved, which is everything that can fail before
    # the first LowState arrives.
    if docker logs "$ESTIMATOR_CONTAINER" 2>&1 | grep -q "legged_odom: .* -> "; then
      return 0
    fi
    sleep 0.5
    waited=$((waited + 1))
  done
  echo "  the estimator container is up but never logged its banner — see $log"
  docker logs "$ESTIMATOR_CONTAINER" 2>&1 | grep -v "type hash" | tail -n 15 | sed 's/^/    /'
  return 1
}
# Python ROS tools that touch a unitree message (the channel mixer, the odom
# comparator) have to run on the interpreter the workspace was BUILT for.
# $WS/install's setup files were generated by a colcon run under conda's
# drclros python, so after setup.sh the python3 on PATH is that env's 3.11 and
# PYTHONPATH carries its Humble rclpy ahead of /opt/ros/jazzy's. That rclpy then
# loads unitree_go's Jazzy/CPython-3.12 typesupport and SEGFAULTS in
# create_subscription -- silently, in a background process, while the rest of
# the rig looks healthy. pubcounts() above only needs rclpy and is unaffected.
ros_py() {
  local pp="" entry IFS=:
  for entry in ${PYTHONPATH:-}; do
    case "$entry" in *conda*) ;; *) pp="${pp:+$pp:}$entry" ;; esac
  done
  env PYTHONPATH="$pp" /usr/bin/python3 "$@"
}

# -E mixed's channel mixer. A survivor is the same hazard as a surviving
# estimator: it keeps publishing /odom_mixed into the next run. The bracket in
# the pkill pattern keeps it from matching pkill's own command line.
MIXER_PID=""
start_mixer() {
  local csv=$1 log=$2
  mkdir -p "$(dirname "$csv")"
  ros_py -u "$HERE/odom_channel_mixer.py" "${MIX_ARGS[@]}" --out /odom_mixed \
    --csv "$csv" > "$log" 2>&1 &
  MIXER_PID=$!
  local _
  for _ in $(seq 1 50); do
    if grep -q "odom mixer:" "$log"; then return 0; fi
    if ! kill -0 "$MIXER_PID" 2>/dev/null; then break; fi
    sleep 0.2
  done
  echo "  the odom channel mixer never came up — see $log"
  tail -n 15 "$log" | sed 's/^/    /'
  return 1
}
stop_mixer() {
  if [ -n "$MIXER_PID" ]; then
    kill -TERM "$MIXER_PID" 2>/dev/null || true
    wait "$MIXER_PID" 2>/dev/null || true
    MIXER_PID=""
  fi
  pkill -f "[o]dom_channel_mixer.py" 2>/dev/null || true
}
cleanup() {
  kill_sims -TERM
  pkill -f g1_difftrack_node 2>/dev/null || true
  stop_mixer
  stop_estimator
  sleep 1
  kill_sims -KILL
  pkill -9 -f g1_difftrack_node 2>/dev/null || true
}
trap cleanup EXIT

# How many publishers /robot_state actually has, asked of the DDS graph rather
# than through `ros2 topic info`: the ros2 daemon caches participants and keeps
# reporting a publisher for minutes after its process is gone, which would fail
# every run here for no reason.
# Both directions, in one query, because both go wrong the same way. A stale
# SIMULATOR interleaves two robots into one state stream; a stale CONTROLLER
# fights the live one for /robot_command and the loser is the robot — the last
# command in wins, and one of the two is a node still sitting in ZEROING, so the
# G1 goes down limp while a perfectly healthy controller reports tracking.
# Prints "<n_robot_state> <n_robot_command>".
pubcounts() {
  python3 - "$STATE_TOPIC" "$CMD_TOPIC" <<'PYCOUNT'
import sys, rclpy, time
from rclpy.node import Node
state_topic, cmd_topic = sys.argv[1:3]
rclpy.init()
n = Node("difftrack_sim2sim_pubcount")
time.sleep(1.5)
print(len(n.get_publishers_info_by_topic(state_topic)),
      len(n.get_publishers_info_by_topic(cmd_topic)))
rclpy.shutdown()
PYCOUNT
}

# Kill leftovers BEFORE starting, not only after. A simulator that survived an
# interrupted run keeps publishing /robot_state, and a controller that receives
# two interleaved simulations produces numbers that look plausible and mean
# nothing — the single easiest way to get a wrong answer out of this script.
#
# Then WAIT for the graph to agree, rather than sleeping a fixed second: a
# killed process's DDS participant outlives it by a moment, and back-to-back
# invocations of this script otherwise trip their own guard.
cleanup
for _ in $(seq 1 20); do
  read -r n_state n_cmd < <(pubcounts)
  if [ "${n_state:-1}" = "0" ] && [ "${n_cmd:-1}" = "0" ]; then break; fi
  sleep 1
done
if [ "${n_state:-1}" != "0" ] || [ "${n_cmd:-1}" != "0" ]; then
  echo "still ${n_state:-?} publishers on $STATE_TOPIC and ${n_cmd:-?} on $CMD_TOPIC"
  echo "  after killing everything — something outside this script is running."
  echo "  A leftover controller is the one that looks like a policy failure: it"
  echo "  fights the live one for /robot_command and the robot goes down limp."
  exit 1
fi

printf '%-26s %4s  %-9s %-9s %-9s %-9s %s\n' motion rep steps mean_err max_err min_h fell_at
for motion in "${MOTIONS[@]}"; do

  # One scene per motion: the RSI pose is that clip's own first frame. Same
  # bake either way -- `ref = theta` plus the matching body-frame rotation, so
  # the reset pose IS the clip and the encoders agree -- only the model it is
  # baked into differs.
  scene_args=(--out "$LOGDIR/scene_$motion" --quiet)
  if [ "$ENTRY" = "rsi" ]; then
    scene_args+=(--init-from "$MODELS/$motion/difftrack_config.json")
  fi

  if [ "$WORKFLOW" = "unitree" ]; then
    # unitree_mujoco takes ONE scene path, and resolves it against
    # unitree_robots/<robot>/ only when it is relative -- so an absolute one is
    # what keeps the generated copy out of the upstream checkout.
    scene_args+=(--flavor unitree)
    SCENE_ABS=$(python3 "$HERE/make_sim2sim_scene.py" "${scene_args[@]}" | tail -1)
  else
    read -r SCENE_ROOT SCENE_REL < <(python3 "$HERE/make_sim2sim_scene.py" "${scene_args[@]}" | tail -1)

    SIMCFG="$LOGDIR/${motion}_sim.yml"
    python3 - "$STOCK_CFG" "$SIMCFG" "$SCENE_REL" <<'PYCFG'
import sys, yaml
stock, out, scene = sys.argv[1:4]
cfg = yaml.safe_load(open(stock))
cfg['sim']['model_path'] = scene
cfg['robot']['verbose'] = False
yaml.safe_dump(cfg, open(out, 'w'), sort_keys=False)
PYCFG
  fi

  for rep in $(seq 1 "$REPEATS"); do
    simlog="$LOGDIR/${motion}_${rep}_sim.log"
    ctllog="$LOGDIR/${motion}_${rep}_ctl.log"

    if [ "$ENTRY" = "rsi" ]; then
      # The robot IS the clip's first frame, at its recorded world heading, so
      # the observation stays in the world frame with (very nearly) the identity
      # anchor — exactly the configuration diffsimrl's own evaluation and the
      # golden trace used. The anchor is still resolved rather than forced to
      # the identity, because a lead-in has to solve for it.
      entry_args=(entry:=direct
                  anchor_motion_to_robot:=true anchor_yaw_to_robot:=false
                  observe_in_reference_frame:=false)
    else
      entry_args=(entry:=pose entry_ramp:="$ENTRY_RAMP"
                  anchor_motion_to_robot:=true anchor_yaw_to_robot:=true
                  observe_in_reference_frame:=true)
    fi

    # The reference's own shape, and it is the same question in both entry modes:
    # what the policy is shown between the robot standing there and the clip
    # running. All three are 0 unless asked for, so the reference is the clip
    # from its first frame to its last and every number this rig has ever
    # reported still means what it meant.
    #
    # The lead-in used to be rsi-only because the node refused it alongside the
    # reference-frame observation; it is built in the clip's frame now and works
    # from a stand too.
    entry_args+=(lead_in_duration:="$LEADIN"
                 motion_blend_in:="$BLEND_IN"
                 motion_blend_out:="$BLEND_OUT")

    # The workflow's config. Empty under drcl, where the launch file's own
    # default (g1_difftrack.yaml) already selects drcl_deploy.
    config_args=()
    if [ -n "$CONFIG_PATH" ]; then config_args=(config_path:="$CONFIG_PATH"); fi
    config_args+=(fall_height:="$FALL_HEIGHT")

    # Where the world base pose comes from. Under `estimator` the controller
    # reads /odom and the simulator's ground-truth path is turned OFF at the
    # launch line — leaving `sportmode_imu` on as well would have both writing
    # robot_state_.base_*, and the base state would be whichever message
    # arrived last. Under `compare` the controller stays on ground truth (a
    # known-good run) and the estimator only rides along to be scored.
    odom_args=()
    est_log="$LOGDIR/${motion}_${rep}_est.log"
    # Which estimator parameters this clip runs with (see ESTIMATOR_PARAMS).
    est_params=""
    case "$ESTIMATOR_PARAMS" in
      auto)
        case "$motion" in
          g1_jumps*|g1_run*)
            if [ -f "$DYNAMIC_ESTIMATOR_PARAMS" ]; then est_params=$DYNAMIC_ESTIMATOR_PARAMS; fi ;;
        esac ;;
      stock) ;;
      *) est_params=$ESTIMATOR_PARAMS ;;
    esac
    case "$ESTIMATOR" in
      estimator)
        odom_args=(odom_topic:=/odom odom_twist_frame:=child unitree_world_state:=none)
        start_estimator "$est_log" "$est_params" || exit 2
        ;;
      compare)
        start_estimator "$est_log" "$est_params" || exit 2
        ;;
      mixed)
        # The mixer publishes a WORLD-frame twist; see odom_channel_mixer.py.
        odom_args=(odom_topic:=/odom_mixed odom_twist_frame:=world unitree_world_state:=none)
        start_estimator "$est_log" "$est_params" || exit 2
        start_mixer "$MIX_CSV_DIR/$(date +%Y%m%d-%H%M%S)_${motion}_r$(printf %02d "$rep")_est-${MIX_TAG}.csv" \
                    "$LOGDIR/${motion}_${rep}_mix.log" || exit 2
        ;;
    esac

    # THE CONTROLLER STARTS FIRST, and that ordering is not incidental. The node
    # boots limp (ZEROING) and mj_sim resets the robot to a standing pose; every
    # millisecond the sim runs without a controller is the robot falling over,
    # and a run that begins from a heap on the floor measures nothing. With
    # auto_engage the node holds a pose from its very first command, so bringing
    # the sim up second means the robot is held from step 0.
    if [ "$STANDCYCLE" = "1" ]; then
      # NOTHING is automated here, and that is the point: this is the sequence a
      # robot session actually has. The controller boots into its rest state on
      # the first state message (start_in_stand) — so the very first command out
      # of it holds the robot rather than dropping it — and then waits for a
      # button. -d is the clip budget per press, not a run length: the node
      # returns to rest when it expires and takes `A` again.
      #
      # auto_engage is OFF and must stay off: it drives the FSM with nobody
      # having pressed anything, which is the one thing this mode exists to not
      # do.
      stand_args=()
      if [ -n "$STAND_ONNX" ]; then stand_args=(stand_onnx_path:="$STAND_ONNX"); fi
      ros2 launch cpp_control g1_difftrack.launch.py \
        motion:="$motion" auto_engage:=false start_in_stand:=true \
        play_duration:="$DURATION" exit_hold:="$EXIT_HOLD" exit_ramp:="$EXIT_RAMP" \
        arm_blend:="$ARM_BLEND" \
        "${stand_args[@]}" "${config_args[@]}" "${odom_args[@]}" "${entry_args[@]}" \
        "${record_args[@]}" "${extra_args[@]}" > "$ctllog" 2>&1 &
    elif [ "$WATCH" = "1" ]; then
      # No budget, no self-shutdown, no timeout: play_duration<0 runs a looping
      # clip until you stop it, and a one-shot clip ends HOLDING the nominal pose
      # rather than exiting — which matters, because a controller that exits is a
      # /robot_command with no publisher, and that is the limp robot.
      ros2 launch cpp_control g1_difftrack.launch.py \
        motion:="$motion" auto_engage:=true play_duration:=-1.0 \
        "${config_args[@]}" "${odom_args[@]}" "${entry_args[@]}" \
        "${record_args[@]}" "${extra_args[@]}" > "$ctllog" 2>&1 &
    else
      timeout -s KILL "$TIMEOUT" \
        ros2 launch cpp_control g1_difftrack.launch.py \
          motion:="$motion" auto_engage:=true play_duration:="$DURATION" \
          exit_when_finished:=true "${config_args[@]}" "${odom_args[@]}" "${entry_args[@]}" \
          "${record_args[@]}" "${extra_args[@]}" > "$ctllog" 2>&1 &
    fi
    ctlpid=$!
    # `if`, not `grep ... && break`: under `set -e` a failing && list at the end
    # of a loop body kills the script on the first poll that finds nothing.
    #
    # And the RESULT IS CHECKED, which it did not used to be. Falling out of this
    # loop and starting mj_sim anyway is the worst thing this script can do:
    # nothing publishes /robot_command, mj_sim zeroes kp and kd on every
    # iteration, and you watch a limp robot fold onto the floor for the whole
    # run — which reads as a policy that cannot stand rather than as a process
    # that never started. A missing rebuild after an export lands here.
    loaded=0
    for _ in $(seq 1 150); do
      if grep -q "difftrack loaded" "$ctllog"; then loaded=1; break; fi
      if ! kill -0 "$ctlpid" 2>/dev/null; then break; fi
      sleep 0.2
    done
    if [ "$loaded" != "1" ]; then
      echo
      echo "  the controller never reached 'difftrack loaded' — NOT starting the simulator."
      echo "  A simulator with no controller is a limp robot on the floor, not a result."
      echo "  Last lines of $ctllog:"
      tail -n 20 "$ctllog" | sed 's/^/    /'
      kill "$ctlpid" 2>/dev/null || true
      exit 1
    fi

    if [ "$WORKFLOW" = "unitree" ]; then
      # unitree_mujoco, exactly as workflows/unitree.md runs it, minus the sudo:
      # that is only needed to bind a real network interface, and this is `lo`.
      #
      #   -r g1   picks the G1Bridge, which is what sets LowState.mode_machine
      #           (5 for the 29-dof G1) and adds the rt/secondary_imu publisher.
      #   -t 1    unitree_hg IDL, stated rather than inferred from nu > 20.
      #   -i/-n   the DDS domain and interface, which MUST match what setup.sh
      #           put in ROS_DOMAIN_ID and CYCLONEDDS_URI or the two processes
      #           never see each other.
      #   -s      the generated scene, absolute so it is not resolved against
      #           the upstream unitree_robots/ tree.
      #
      # Everything else comes from unitree_mujoco's own simulate/config.yaml,
      # whose shipped defaults are already what a measurement wants:
      # use_joystick 0 and enable_elastic_band 0. A band left on would hold the
      # robot up, which is this plant's version of the mj_sim world_root weld.
      # `env -u LD_LIBRARY_PATH`, and it is not optional. unitree_mujoco is a
      # plain C++ program built against the SYSTEM toolchain; it is not a ROS
      # node and wants nothing from the conda env. But setup.sh has just put
      # $CONDA_PREFIX/lib at the front of LD_LIBRARY_PATH, and the binary then
      # loads conda's libstdc++ and libyaml-cpp while its boost still resolves
      # to the system one. It dies with `free(): invalid pointer` moments after
      # the DDS bridge starts -- which reads as the simulator crashing on our
      # traffic, and is nothing of the kind.
      #   -c      hold the physics at the reset pose until the first LowCmd.
      #           Without it the robot integrates limp while unitree_mujoco
      #           brings its DDS bridge up, and a G1 does not survive that: it
      #           arrives at the controller's first command with its hips a
      #           third of a radian out of place, and no gain recovers it. This
      #           is the flag that makes the number a measurement of the policy
      #           rather than of the simulator's startup.
      #
      # -R: DRCL_RECORD_AUTOSTART is F9 at t=0. The recorder stops itself when
      # the simulator exits, so the mp4 covers exactly the run. One file per
      # run, named for what is in it, in the same recordings/ directory the
      # mj_sim recorder used; DRCL_RECORD_* from the caller's shell still
      # overrides the size, fps, encoder and clock.
      rec_env=()
      if [ "$RECORD" = "1" ]; then
        rec_env=(DRCL_RECORD_AUTOSTART=1
                 "DRCL_RECORD_DIR=${DRCL_RECORD_DIR:-$PKG/recordings}"
                 "DRCL_RECORD_PREFIX=${motion}_${ENTRY}_${rep}")
      fi
      sim_flags=()
      if [ "$HEADLESS" = "1" ]; then sim_flags=(--headless); fi
      env -u LD_LIBRARY_PATH "${rec_env[@]}" \
        "$SIM_BIN" -r g1 -t 1 -c -i "$ROS_DOMAIN_ID" -n lo -s "$SCENE_ABS" "${sim_flags[@]}" \
        > "$simlog" 2>&1 &
      simpid=$!
    else
      # scripts/run_mj_sim.py, not `ros2 run mj_sim main`: same simulator, same
      # node, same loop, plus a camera that stays on the robot. Upstream's free
      # camera does not move, so the G1 walks out of frame in about four seconds.
      record_args=()
      if [ "$RECORD" = "1" ]; then
        # One file per run, named for what is in it. The recorder's own defaults
        # (720p30, simulation clock, cpp_control/recordings/) hold otherwise, and
        # DRCL_RECORD_* from the caller's shell still overrides them.
        record_args=(--record)
        export DRCL_RECORD_PREFIX="${motion}_${ENTRY}_${rep}"
      fi
      SIM_ASSETS_PATH="$SCENE_ROOT" python3 -u "$HERE/run_mj_sim.py" --cfgpath "$SIMCFG" \
        --speed "$SPEED" "${record_args[@]}" > "$simlog" 2>&1 &
      simpid=$!
    fi

    # Exactly one simulator, or the state stream is two robots interleaved.
    #
    # Asked of the DDS graph directly rather than through `ros2 topic info`:
    # the ros2 daemon caches participants and keeps reporting a publisher for
    # minutes after its process is gone, which would fail every run here for no
    # reason.
    #
    # WAITED for, rather than slept on. A fixed sleep encodes how long ONE
    # simulator takes to come up on ONE machine: mj_sim is publishing in about a
    # second, unitree_mujoco takes ~2.5 s to compile the G1's meshes before its
    # bridge exists at all, and a cold page cache moves both. The old `sleep 2`
    # failed this check on unitree_mujoco every time, and reported it as "a
    # stale simulator is running" -- a real symptom's message on a plainly
    # different cause.
    for _ in $(seq 1 30); do
      read -r n_state n_cmd < <(pubcounts)
      if [ "${n_state:-0}" != "0" ]; then break; fi
      if ! kill -0 "$simpid" 2>/dev/null; then break; fi
    done
    if [ "${n_state:-0}" != "1" ] || [ "${n_cmd:-0}" != "1" ]; then
      echo "  expected exactly 1 publisher each on $STATE_TOPIC and $CMD_TOPIC,"
      echo "  found ${n_state:-0} and ${n_cmd:-0} — a stale simulator or controller is running"
      kill -9 "$simpid" 2>/dev/null || true
      kill "$ctlpid" 2>/dev/null || true
      exit 1
    fi

    # ... and the controller is still there to answer it. It loaded a moment ago,
    # but ONNX Runtime and the model load both happen before the first state
    # arrives, so a crash between the two would otherwise be reported as a
    # tracking failure rather than as a dead process.
    if ! kill -0 "$ctlpid" 2>/dev/null; then
      echo "  the controller exited between loading and the first state — see $ctllog"
      tail -n 20 "$ctllog" | sed 's/^/    /'
      kill -9 "$simpid" 2>/dev/null || true
      KEEP=1
      exit 1
    fi

    if [ "$WATCH" = "1" ]; then
      echo
      if [ "$STANDCYCLE" = "1" ]; then
        echo "  $motion — the robot is STANDING in its rest state and waiting for you."
        echo
        echo "    A          play the clip for ${DURATION}s, then back to the rest state"
        echo "    A again    play it again, from wherever the robot came to rest"
        echo "    X          nominal pose      B  zeroing (limp)      Y  damping"
        echo
        echo "  With a gamepad on the simulator (use_joystick: 1 in"
        echo "  unitree_mujoco/simulate/config.yaml) those are the pad's own buttons."
        echo "  Without one, publish /joy from another terminal in this environment —"
        echo "  X-mode indices, A=0 B=1 X=2 Y=3, and only the RISING edge counts:"
        echo
        echo "    ros2 topic pub --once /joy sensor_msgs/msg/Joy \"{axes: [0,0,0,0,0,0], buttons: [1,0,0,0,0,0]}\""
        echo "    ros2 topic pub --once /joy sensor_msgs/msg/Joy \"{axes: [0,0,0,0,0,0], buttons: [0,0,0,0,0,0]}\""
        echo
        echo "  (the second one releases it, so the next press is a new edge)"
      else
        echo "  watching $motion — entry=$ENTRY, tracking until Ctrl-C."
      fi
      echo "  controller log: $ctllog"
      echo "  simulator log:  $simlog"
      echo
      # The controller's own throttled tracking line, live. It never exits on its
      # own here (no exit_when_finished, play_duration<0), so Ctrl-C is the end,
      # and cleanup() then kills the simulator BEFORE the controller — that order
      # is why the last thing in the viewer is a robot still being held.
      tail -n 0 -f "$ctllog" &
      tailpid=$!
      wait "$ctlpid" 2>/dev/null || true
      kill "$tailpid" 2>/dev/null || true
      exit 0
    fi

    # Score the estimator WHILE the run is happening, not after: both the
    # simulator's ground truth and /odom stop existing the moment those
    # processes die, so there is nothing to compare once the run is over.
    # Slightly shorter than the tracking budget so it finishes first.
    if [ "$ESTIMATOR" = "compare" ]; then
      cmp_secs=$(python3 -c "print(max(3.0, $DURATION - 1.0))")
      ros_py "$HERE/compare_odom_ground_truth.py" -d "$cmp_secs" \
              --csv "$LOGDIR/${motion}_${rep}_odom.csv" 2>&1 | sed 's/^/  /' &
      cmppid=$!
    fi

    # exit_when_finished makes the node shut down on its own SUMMARY; the
    # timeout above is a backstop for a run that never gets there at all.
    wait "$ctlpid" 2>/dev/null || true
    if [ -n "${cmppid:-}" ]; then wait "$cmppid" 2>/dev/null || true; cmppid=""; KEEP=1; fi

    # SIGKILL with no grace period, deliberately. From the moment the controller
    # exits, /robot_command has no publisher and mj_sim is driving kp=0 kd=0.1,
    # so every second of politeness here is the robot folding onto the floor in
    # front of whoever is watching. The measurement is already in the log.
    # disown first, or bash reports "Killed <the whole command line>" into the
    # middle of the results table when it reaps the job.
    disown "$simpid" 2>/dev/null || true
    kill -9 "$simpid" 2>/dev/null || true
    kill_sims -KILL
    stop_mixer

    line=$(grep -o 'SUMMARY .*' "$ctllog" | tail -1 || true)
    if [ -z "$line" ]; then
      printf '%-26s %4s  %s\n' "$motion" "$rep" "NO SUMMARY — see $ctllog"
      KEEP=1
      continue
    fi
    get() { sed -n "s/.*$1=\([^ ]*\).*/\1/p" <<< "$line"; }
    printf '%-26s %4s  %-9s %-9s %-9s %-9s %s\n' \
      "$motion" "$rep" "$(get steps)" "$(get mean_err)" "$(get max_err)" \
      "$(get min_height)" "$(get fell_at)"
  done
done

echo
echo "workflow=$WORKFLOW  plant=$([ "$WORKFLOW" = unitree ] && echo unitree_mujoco || echo mj_sim)  \
state=$STATE_TOPIC  cmd=$CMD_TOPIC"
echo "entry=$ENTRY  duration=${DURATION}s  lead_in=${LEADIN}s  entry_ramp=${ENTRY_RAMP}s  \
speed=$([ "$FREERUN" = 1 ] && echo free-run || echo 1x)"
echo "motion blend: in=${BLEND_IN}s  out=${BLEND_OUT}s"
echo "world state: -E $ESTIMATOR$([ "$ESTIMATOR" = mixed ] && echo " (on the estimator: $MIX_TAG; csvs in $MIX_CSV_DIR)")  \
viewer: $([ "$HEADLESS" = 1 ] && echo headless || echo window)"
if [ "$ESTIMATOR" != "ground_truth" ]; then
  echo "estimator params: ESTIMATOR_PARAMS=$ESTIMATOR_PARAMS$([ "$ESTIMATOR_PARAMS" = auto ] && \
echo " (g1_jumps*/g1_run*: $([ -f "$DYNAMIC_ESTIMATOR_PARAMS" ] && echo g1_dynamic.yaml || echo 'stock -- no g1_dynamic.yaml yet'); others: stock g1.yaml)")"
fi
if [ "$RUN_LOG" = "1" ]; then
  echo "run logs: $RUN_LOG_DIR  (one npz per run, tagged sim2sim_${RUN_TAG_MODE})"
  echo "          python3 $HERE/analyze_tracking.py $RUN_LOG_DIR/*.npz"
fi
echo "to watch one instead of measuring all of them:  $0 -w -e $ENTRY <motion>"
echo "to drive it yourself from a standing robot:     $0 -s -d $DURATION <motion>"
if [ "$KEEP" = "1" ]; then
  echo "logs: $LOGDIR"
else
  rm -rf "$LOGDIR"
fi
