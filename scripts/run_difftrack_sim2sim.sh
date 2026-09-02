#!/usr/bin/env bash
# Unattended sim2sim for the difftrack tracker: mj_sim + the controller, no
# joystick, one SUMMARY line per run.
#
#   bash scripts/run_difftrack_sim2sim.sh [options] [motion ...]
#
#   -e MODE      entry: rsi (default) or stand   — see below
#   -d SECONDS   tracking budget per run         (default 20)
#   -r N         repeats per motion              (default 1)
#   -l SECONDS   lead-in: ramp the reference from rest onto the clip's entry
#                velocity (rsi entry only)       (default 0)
#   -p SECONDS   entry ramp onto the clip's first frame, stand entry only;
#                0 (default) engages straight from the nominal pose, which
#                measures far better — see below
#   -k           keep the logs and say where they are
#   motion ...   directories under models/tracker/difftrack (default: all)
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
# Both modes disable the scene's `world_root` weld (see make_sim2sim_scene.py):
# the shipped scene hangs the robot in the air for bring-up, and anything a weld
# holds up is not a transfer result.
#
# A run is clean when fell_at=none. mean_err is the root tracking error in
# metres against the reference the policy was actually given.
set -eo pipefail

ENTRY=rsi
DURATION=20
REPEATS=1
LEADIN=0.0
ENTRY_RAMP=0.0
KEEP=0
while getopts "e:d:r:l:p:kh" opt; do
  case $opt in
    e) ENTRY=$OPTARG ;;
    d) DURATION=$OPTARG ;;
    r) REPEATS=$OPTARG ;;
    l) LEADIN=$OPTARG ;;
    p) ENTRY_RAMP=$OPTARG ;;
    k) KEEP=1 ;;
    h) sed -n '2,41p' "$0"; exit 0 ;;
    *) exit 2 ;;
  esac
done
shift $((OPTIND - 1))
case "$ENTRY" in rsi|stand) ;; *) echo "unknown entry mode '$ENTRY'"; exit 2 ;; esac

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd "$HERE/.." && pwd)"
source "$PKG/workflows/conda_env/runenv.sh" > /dev/null

# mj_sim's viewer is GLFW: under a Wayland session it takes the Wayland backend,
# fails to load libdecor's GTK plugin and segfaults on startup. Force the X11
# backend (XWayland is fine). Its --headless flag is not an alternative: mj_sim's
# reset() unconditionally touches self.viewer, which headless never creates.
export DISPLAY=${DISPLAY:-:1}
unset WAYLAND_DISPLAY XDG_SESSION_TYPE

MODELS="$(ros2 pkg prefix cpp_control)/share/cpp_control/models/tracker/difftrack"
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

LOGDIR=$(mktemp -d -t difftrack_sim2sim.XXXXXX)
STOCK_CFG=$(python3 -c "
import os, mj_sim
print(os.path.join(os.path.dirname(mj_sim.__file__), 'config', 'G1.yml'))")

# SIGTERM then SIGKILL: mj_sim does not reliably die on TERM (rclpy installs its
# own signal handling and the simulator's main loop is a bare `while True`), and
# a survivor keeps publishing /robot_state into the NEXT run.
cleanup() {
  pkill -f "lib/mj_sim/main" 2>/dev/null || true
  pkill -f g1_difftrack_node 2>/dev/null || true
  sleep 1
  pkill -9 -f "lib/mj_sim/main" 2>/dev/null || true
  pkill -9 -f g1_difftrack_node 2>/dev/null || true
}
trap cleanup EXIT

# How many publishers /robot_state actually has, asked of the DDS graph rather
# than through `ros2 topic info`: the ros2 daemon caches participants and keeps
# reporting a publisher for minutes after its process is gone, which would fail
# every run here for no reason.
pubcount() {
  python3 - <<'PYCOUNT'
import rclpy, time
from rclpy.node import Node
rclpy.init()
n = Node("difftrack_sim2sim_pubcount")
time.sleep(1.5)
print(len(n.get_publishers_info_by_topic("/robot_state")))
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
  if [ "$(pubcount)" = "0" ]; then break; fi
  sleep 1
done

printf '%-26s %4s  %-9s %-9s %-9s %-9s %s\n' motion rep steps mean_err max_err min_h fell_at
for motion in "${MOTIONS[@]}"; do

  # One scene per motion: the RSI pose is that clip's own first frame.
  scene_args=(--out "$LOGDIR/scene_$motion" --quiet)
  if [ "$ENTRY" = "rsi" ]; then
    scene_args+=(--init-from "$MODELS/$motion/difftrack_config.json")
  fi
  read -r SCENE_ROOT SCENE_REL < <(python3 "$HERE/make_sim2sim_scene.py" "${scene_args[@]}" | tail -1)

  SIMCFG="$LOGDIR/${motion}_sim.yml"
  python3 - "$STOCK_CFG" "$SIMCFG" "$SCENE_REL" <<'PY'
import sys, yaml
stock, out, scene = sys.argv[1:4]
cfg = yaml.safe_load(open(stock))
cfg['sim']['model_path'] = scene
cfg['robot']['verbose'] = False
yaml.safe_dump(cfg, open(out, 'w'), sort_keys=False)
PY

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
                  observe_in_reference_frame:=false
                  lead_in_duration:="$LEADIN")
    else
      entry_args=(entry:=pose entry_ramp:="$ENTRY_RAMP"
                  anchor_motion_to_robot:=true anchor_yaw_to_robot:=true
                  observe_in_reference_frame:=true)
    fi

    # THE CONTROLLER STARTS FIRST, and that ordering is not incidental. The node
    # boots limp (ZEROING) and mj_sim resets the robot to a standing pose; every
    # millisecond the sim runs without a controller is the robot falling over,
    # and a run that begins from a heap on the floor measures nothing. With
    # auto_engage the node holds a pose from its very first command, so bringing
    # the sim up second means the robot is held from step 0.
    timeout -s KILL $((DURATION * 3 + 90)) \
      ros2 launch cpp_control g1_difftrack.launch.py \
        motion:="$motion" auto_engage:=true play_duration:="$DURATION" \
        exit_when_finished:=true "${entry_args[@]}" > "$ctllog" 2>&1 &
    ctlpid=$!
    # `if`, not `grep ... && break`: under `set -e` a failing && list at the end
    # of a loop body kills the script on the first poll that finds nothing.
    for _ in $(seq 1 150); do
      if grep -q "difftrack loaded" "$ctllog"; then break; fi
      sleep 0.2
    done

    SIM_ASSETS_PATH="$SCENE_ROOT" ros2 run mj_sim main -- --cfgpath "$SIMCFG" \
      > "$simlog" 2>&1 &
    simpid=$!

    # Exactly one simulator, or the state stream is two robots interleaved.
    #
    # Asked of the DDS graph directly rather than through `ros2 topic info`:
    # the ros2 daemon caches participants and keeps reporting a publisher for
    # minutes after its process is gone, which would fail every run here for no
    # reason.
    sleep 2
    npub=$(pubcount)
    if [ "${npub:-0}" != "1" ]; then
      echo "  /robot_state has ${npub:-0} publishers, expected 1 — a stale mj_sim is running"
      kill -9 "$simpid" 2>/dev/null || true
      kill "$ctlpid" 2>/dev/null || true
      exit 1
    fi

    # exit_when_finished makes the node shut down on its own SUMMARY; the
    # timeout above is a backstop for a run that never gets there at all.
    wait "$ctlpid" 2>/dev/null || true

    kill "$simpid" 2>/dev/null || true
    sleep 1
    kill -9 "$simpid" 2>/dev/null || true
    pkill -9 -f "lib/mj_sim/main" 2>/dev/null || true

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
echo "entry=$ENTRY  duration=${DURATION}s  lead_in=${LEADIN}s  entry_ramp=${ENTRY_RAMP}s"
if [ "$KEEP" = "1" ]; then
  echo "logs: $LOGDIR"
else
  rm -rf "$LOGDIR"
fi
