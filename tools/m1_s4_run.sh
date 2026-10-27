#!/bin/sh
set -eu

# Candidate invoked only by m1_s4_launch.sh through the S4 watchdog controller.
# It runs unprivileged after LightDM/Xorg have released DRM master.
bundle=${1:?usage: m1_s4_run.sh BUNDLE SECONDS [DEVICE] [AIRCRAFT] [BW_LADDER] [MOUSE|none]}
seconds=${2:?seconds}
device=${3:-/dev/input/by-id/usb-Logitech_Logitech_Extreme_3D-event-joystick}
aircraft=${4:-global5000}
bw_ladder=${5:-off}
# AP-001 panel mouse, read-ACL-granted by the launcher; none = no mouse.
mouse=${6:-none}
ev=${M0C_EV:?watchdog did not provide M0C_EV}

case $seconds in *[!0-9]*|'') echo "bad seconds: $seconds" >&2; exit 20;; esac
[ "$seconds" -ge 30 ] && [ "$seconds" -le 10800 ] || { echo "seconds must be 30..10800" >&2; exit 20; }
case $bw_ladder in on|off) ;; *) echo "bandwidth ladder must be on/off" >&2; exit 20;; esac
[ "$bw_ladder" = off ] || [ "$seconds" = 300 ] || { echo "bandwidth ladder requires 300 seconds" >&2; exit 20; }
cd "$bundle"
[ -r "$device" ] || { echo "joystick is not readable: $device" >&2; exit 21; }
[ ! -e "$ev/s4-run.jsonl" ] || { echo "run log already exists" >&2; exit 22; }
# A mouse we cannot read is not fatal: fly without it (the panel shows NO MOUSE).
[ "$mouse" = none ] || [ -r "$mouse" ] || { echo "mouse is not readable, running without it: $mouse" >&2; mouse=none; }
sha256sum --quiet -c files.sha256
# ISS-038: inside this very unit, the sim/I-O threads must get SCHED_FIFO;
# otherwise stop now (the controller restores the desktop).
./bin/gx-app rtprobe > "$ev/rtprobe.json" 2>&1 || { echo "realtime envelope missing in the candidate unit:" >&2; cat "$ev/rtprobe.json" >&2; exit 23; }
# SPIKE-002 bundle only (spike002/ is hash-checked above): the surveyed SFO
# world and the photoreal renderer. A plain S4 bundle runs exactly as before.
if [ -f spike002/sfo-photoreal.gxvisual ]; then
  set -- --world spike002/sfo-real.gxworld --visual spike002/sfo-photoreal.gxvisual
else
  set --
fi
# Optional full-simulation bandwidth probe.  It preallocates/first-touches its
# fixed 3 x 256 MiB STREAM working set, then waits on the gate.  The app and
# helper share the candidate cgroup; the helper uses only CPUs 0..2 and never
# CPU 3 (the render core).  Normal runs do not start or require this helper.
helper_pid=
gate=$ev/stream-ladder
if [ "$bw_ladder" = on ]; then
  [ -x ./bin/global-express-m0-budget ] || { echo "bandwidth ladder helper missing" >&2; exit 24; }
  mkdir -p "$ev/stream-raw"
  ./bin/global-express-m0-budget --mode stream_ladder --run-id "$(basename "$ev")" \
    --duration "$seconds" --interval 1 --stream-size 256M --kernel triad --thp on \
    --gate "$gate" --out "$ev/stream-ladder.jsonl" --raw-dir "$ev/stream-raw" \
    > "$ev/stream-ladder.stdout" 2> "$ev/stream-ladder.stderr" &
  helper_pid=$!
  printf '%s\n' "$helper_pid" > "$ev/stream-ladder.pid"
  i=0
  while [ ! -s "$gate.ready" ]; do
    kill -0 "$helper_pid" 2>/dev/null || { wait "$helper_pid"; echo "bandwidth ladder failed before readiness" >&2; exit 24; }
    i=$((i + 1)); [ "$i" -le 600 ] || { kill -TERM "$helper_pid" 2>/dev/null; wait "$helper_pid" 2>/dev/null; echo "bandwidth ladder readiness timeout" >&2; exit 24; }
    sleep .05
  done
fi
# A SPIKE-002 bundle carries a read-only 1 Hz process/DRM/memory sampler.
# Keep plain S4 unchanged when that optional helper is absent.  The candidate
# remains the unit's workload; systemd's KillMode=control-group covers both
# children if the watchdog stops the unit.
./bin/gx-app s4 --display kms --seconds "$seconds" --log "$ev/s4-run.jsonl" \
  --device "$device" --aircraft "$aircraft" --mouse "$mouse" --jsbsim-root ./jsbsim "$@" &
app_pid=$!
if [ "$bw_ladder" = on ]; then
  printf 'schema=gx-stream-ladder-gate/1\napp_pid=%s\nhelper_pid=%s\nmonotonic_s=%s\n' \
    "$app_pid" "$helper_pid" "$(cut -d' ' -f1 /proc/uptime)" > "$gate.go"
fi
sampler_pid=
if [ -x ./spike002_sampler.sh ]; then
  sh ./spike002_sampler.sh "$app_pid" "$helper_pid" > "$ev/sampler.jsonl" &
  sampler_pid=$!
fi
set +e
wait "$app_pid"; app_rc=$?
set -e
[ -z "$sampler_pid" ] || wait "$sampler_pid" || true
[ -z "$helper_pid" ] || {
  if [ "$app_rc" -ne 0 ] && kill -0 "$helper_pid" 2>/dev/null; then kill -TERM "$helper_pid" 2>/dev/null || true; fi
  set +e; wait "$helper_pid"; helper_rc=$?; set -e
  printf '%s\n' "$helper_rc" > "$ev/stream-ladder.exit"
  [ "$helper_rc" -eq 0 ] || [ "$app_rc" -ne 0 ] || app_rc=24
}
exit "$app_rc"
