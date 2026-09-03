#!/usr/bin/env bash
# Build the estimator image. Run from anywhere.
#
#   bash docker/estimator/build.sh [--tag NAME] [extra docker build args...]
#
# The build context is the colcon workspace's src/, which is what lets the
# image compile unitree_hg from the SAME message definitions the rest of the
# workspace uses. Building with a narrower context is the one modification that
# silently produces a container that discovers /lowstate and then deserialises
# it wrong.
set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
src=$(cd "$here/../../.." && pwd)          # .../cyclonedds_ws/src
tag=g1-estimator

args=()
while [ $# -gt 0 ]; do
    case "$1" in
        --tag) tag=$2; shift 2 ;;
        *) args+=("$1"); shift ;;
    esac
done

if [ ! -d "$src/unitree/unitree_hg" ]; then
    echo "build.sh: no unitree_hg under $src/unitree — wrong workspace?" >&2
    exit 1
fi

echo "context : $src"
echo "tag     : $tag"

# --network=host for the BUILD, which is not the usual advice and is needed
# here. The build has to reach three apt repositories, and a container on
# docker's default bridge inherits the host's /etc/resolv.conf while NOT being
# able to reach the nameservers in it whenever those are a VPN's — which is the
# state this machine is in most of the time. The symptom is not a DNS error: apt
# prints `Ign:` for every source, waits out a 60 s timeout each, and eventually
# fails on a package it "cannot find". Host networking sidesteps the whole
# question, and this build pulls nothing that wants isolating.
docker build --network=host -f "$here/Dockerfile" -t "$tag" "${args[@]+"${args[@]}"}" "$src"

cat <<MSG

built: $tag

  run it:      bash docker/estimator/run.sh
  or in sim:   bash scripts/run_difftrack_sim2sim.sh -E estimator -d 15 g1_walk
MSG
