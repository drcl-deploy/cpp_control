#!/usr/bin/env bash
# Run one or more difftrack policies in sim2sim, capture the ACTIONS they emit,
# and compare how smooth those actions are.
#
#   bash scripts/compare_action_smoothness.sh [options] [motion ...]
#
#   -d SECONDS   tracking budget per run                 (default 13)
#   -r N         repeats per motion                      (default 1)
#   -e MODE      entry: rsi (default) or stand           — see run_difftrack_sim2sim.sh
#   -E MODE      world pose: estimator (default) | ground_truth
#   -o DIR       where the npz files and the figures go
#                (default recordings/action_smoothness/<stamp>)
#   -a           ANALYSE ONLY: re-read the npz files already in -o and redraw.
#                No simulator, no robot, seconds instead of minutes.
#   -j JOINTS    comma-separated joint names to put in the trace figure
#                (default: the ones the runs are actually jittery on)
#   -z A:B       seconds of the run to zoom the trace figure on (default 4:7)
#   motion ...   directories under models/tracker/difftrack
#                (default: g1_dance15s g1_walk)
#
# WHAT IT MEASURES, AND WHY OFF THE WIRE
#
# The action is the policy's whole output, and first-order smoothness is a
# property of the SEQUENCE of them: a[t] - a[t-1]. Nothing about a single
# inference tells you whether the robot will buzz. So this records every action
# of a run in order, from /lowcmd, where the controller has already put it:
#
#     q*[m] = default_angles[p] + action_scale * a[p]
#
# is invertible, exact, and needs no change to the controller — which matters
# more than convenience. A logging build is a different binary from the one
# being measured, and the thing being measured here is timing-sensitive.
#
# Actions are recorded ONLY while the policy is driving. Every hold mode
# publishes a LowCmd too, and its q is a joint target as well; the gains are
# what tell them apart (see record_policy_actions.py).
#
# THE RUNS ARE HELD EQUAL
#
# Same plant, same entry, same world-state source, same tracking budget, same
# controller build. What differs is the checkpoint and the clip it tracks —
# which is the comparison. Two things it is NOT:
#
#   * not a comparison of two runs of the SAME clip. g1_dance15s and g1_walk
#     ask for different motion, so the raw action traces are not overlaid; the
#     smoothness statistics are what is compared, and each one is reported
#     against its own clip's reference so "the clip is busier" cannot pass for
#     "the policy is jitterier".
#   * not repeatable to the last bit. -E estimator puts the onboard estimator in
#     the loop, and it is the honest transfer condition; -E ground_truth takes
#     it back out when you want the policy's own contribution alone. Use -r 2 or
#     more before believing a small difference either way.
set -eo pipefail

DURATION=13
REPEATS=1
ENTRY=rsi
ESTIMATOR=estimator
OUTDIR=""
ANALYSE_ONLY=0
JOINTS=""
ZOOM="4:7"

while getopts "d:r:e:E:o:j:z:ah" opt; do
  case $opt in
    d) DURATION=$OPTARG ;;
    r) REPEATS=$OPTARG ;;
    e) ENTRY=$OPTARG ;;
    E) ESTIMATOR=$OPTARG ;;
    o) OUTDIR=$OPTARG ;;
    j) JOINTS=$OPTARG ;;
    z) ZOOM=$OPTARG ;;
    a) ANALYSE_ONLY=1 ;;
    h) sed -n '2,/^set -eo/p' "$0" | sed '$d'; exit 0 ;;
    *) exit 2 ;;
  esac
done
shift $((OPTIND - 1))

if [ $# -gt 0 ]; then MOTIONS=("$@"); else MOTIONS=(g1_dance15s g1_walk); fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd "$HERE/.." && pwd)"

if [ "$ANALYSE_ONLY" = "1" ] && [ -z "$OUTDIR" ]; then
  echo "-a re-reads an existing run; say which one with -o DIR"
  exit 2
fi
[ -n "$OUTDIR" ] || OUTDIR="$PKG/recordings/action_smoothness/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUTDIR"

# One source per shell, and `sim` for the same reason run_difftrack_sim2sim.sh
# passes it: the recorder has to be on the same middleware and the same
# interface as the controller, or it subscribes to a topic nobody is on and
# reports a policy that emitted no actions.
source "$PKG/../../../setup.sh" sim > /dev/null

MODELS="$(ros2 pkg prefix cpp_control)/share/cpp_control/models/tracker/difftrack"

plot() {
  local npzs=("$OUTDIR"/*.npz)
  if [ ! -e "${npzs[0]}" ]; then
    echo "no recordings in $OUTDIR — nothing to plot."
    exit 1
  fi
  local plot_args=(--out "$OUTDIR" --zoom "$ZOOM")
  [ -n "$JOINTS" ] && plot_args+=(--joints "$JOINTS")
  python3 "$HERE/analyze_action_smoothness.py" "${npzs[@]}" "${plot_args[@]}"
}

if [ "$ANALYSE_ONLY" = "1" ]; then
  plot
  exit 0
fi

for motion in "${MOTIONS[@]}"; do
  CFG="$MODELS/$motion/difftrack_config.json"
  if [ ! -f "$CFG" ]; then
    echo "no export at $MODELS/$motion"
    echo "  known:  $(cd "$MODELS" && ls -d */ | tr -d / | tr '\n' ' ')"
    exit 1
  fi
done

echo "recording ${#MOTIONS[@]} policy/policies x $REPEATS into $OUTDIR"
echo "  ${DURATION}s each, entry=$ENTRY, world pose=$ESTIMATOR"
echo

for motion in "${MOTIONS[@]}"; do
  for rep in $(seq 1 "$REPEATS"); do
    tag="$motion"
    [ "$REPEATS" -gt 1 ] && tag="${motion}_r${rep}"
    npz="$OUTDIR/$tag.npz"
    runlog="$OUTDIR/$tag.runlog"

    # The recorder FIRST, and it is a subscriber only: it publishes nothing, so
    # run_difftrack_sim2sim.sh's "exactly one publisher each" guard still sees
    # exactly one of each and this cannot be mistaken for a stale controller.
    # Starting it second would lose the entry, which is where a policy that is
    # about to be jittery usually says so first.
    python3 -u "$HERE/record_policy_actions.py" \
      --config "$MODELS/$motion/difftrack_config.json" \
      --out "$npz" --motion "$motion" --label "$tag" \
      --note "entry=$ENTRY world=$ESTIMATOR duration=${DURATION}s rep=$rep" \
      > "$OUTDIR/$tag.reclog" 2>&1 &
    recpid=$!

    echo "── $tag ──"
    bash "$HERE/run_difftrack_sim2sim.sh" \
        -d "$DURATION" -e "$ENTRY" -E "$ESTIMATOR" -k "$motion" 2>&1 | tee "$runlog" | sed 's/^/  /'

    # The recorder stops itself when /lowcmd goes quiet (--idle-timeout), which
    # is a few seconds after the controller exits. Give it that, then TERM it —
    # its handler saves what it has rather than throwing the run away.
    for _ in $(seq 1 40); do
      kill -0 "$recpid" 2>/dev/null || break
      sleep 0.5
    done
    kill -TERM "$recpid" 2>/dev/null || true
    wait "$recpid" 2>/dev/null || true
    sed 's/^/  /' "$OUTDIR/$tag.reclog" | grep -v "^\s*\[INFO\]" | tail -n 4 || true
    echo
  done
done

plot
