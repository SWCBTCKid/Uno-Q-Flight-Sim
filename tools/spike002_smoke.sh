#!/bin/sh
set -eu

# SPIKE-002 non-display smoke (unprivileged; no sudo, LightDM untouched):
# the full S4 layout (JSBSim global5000, FIFO sim/I/O, render thread) with
# the synthetic neutral stick, the render thread drawing offscreen on the
# render node, the scripted 10 s warm-up + 4 x 30 s camera trace and one PPM
# capture per segment, plus the 1 Hz DRM/memory sampler.
#   SCENE treatment: surveyed world + GXVISUAL photo pass
#   SCENE control:   the same surveyed world, S4 flat-colour renderer
# Fed to the board as `ssh BOARD sh -s < wrapper` (no metacharacters on ssh).
bundle=${1:?usage: spike002_smoke.sh BUNDLE [SECONDS] [treatment|control]}
seconds=${2:-136}
scene=${3:-treatment}
case $seconds in *[!0-9]*|'') echo "bad seconds" >&2; exit 2;; esac
cd "$bundle"
sha256sum --quiet -c files.sha256
ts=$(date -u +%Y%m%dT%H%M%SZ)
out=smoke/spike002-$scene-$ts
mkdir -p "$out/captures"
case $scene in
  treatment) set -- --world spike002/sfo-real.gxworld --visual spike002/sfo-photoreal.gxvisual;;
  control) set -- --world spike002/sfo-real.gxworld;;
  *) echo "scene: treatment or control" >&2; exit 2;;
esac
{ cat /proc/meminfo; cat /proc/loadavg; ps -eo pid,comm,rss --sort=-rss | head -12; grep -E 'Cached|Mapped' /proc/meminfo; } > "$out/pre.txt"
start=$(cut -d' ' -f1 /proc/uptime)
aircraft=global5000
[ -r aircraft ] && aircraft=$(head -n 1 aircraft)
./bin/gx-app s4 --display offscreen --stick neutral --autostart --quiet --seconds "$seconds" --aircraft "$aircraft" \
  --log "$out/run.jsonl" --jsbsim-root ./jsbsim "$@" --trace --capture-dir "$out/captures" > "$out/stdout.txt" 2> "$out/stderr.txt" &
pid=$!
sh ./spike002_sampler.sh "$pid" > "$out/sampler.jsonl" &
spid=$!
rc=0
wait "$pid" || rc=$?
wait "$spid" || true
{ cat /proc/meminfo; cat /proc/loadavg; } > "$out/post.txt"
echo "{\"scene\":\"$scene\",\"rc\":$rc,\"process_start_uptime_s\":$start,\"seconds\":$seconds}" > "$out/result.json"
echo "SMOKE_DIR=$bundle/$out rc=$rc"
