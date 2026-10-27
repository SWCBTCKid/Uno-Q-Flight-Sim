#!/bin/sh
set -eu

# Non-display S4 smoke, unprivileged (no sudo, LightDM untouched): the full
# S4 layout (JSBSim global5000, FIFO sim/I/O, render thread) with the
# synthetic neutral stick and the render thread drawing on the render node
# (--display offscreen). Fed to `ssh BOARD sh -s -- BUNDLE SECONDS`.
bundle=${1:?usage: m1_s4_smoke.sh BUNDLE [SECONDS]}
seconds=${2:-30}
case $seconds in *[!0-9]*|'') echo "bad seconds" >&2; exit 2;; esac
cd "$bundle"
sha256sum --quiet -c files.sha256
mkdir -p smoke
log=smoke/s4-smoke-$(date -u +%Y%m%dT%H%M%SZ).jsonl
./bin/gx-app s4 --display offscreen --stick neutral --autostart --quiet \
  --seconds "$seconds" --log "$log" --jsbsim-root ./jsbsim
echo "SMOKE_LOG=$bundle/$log"
