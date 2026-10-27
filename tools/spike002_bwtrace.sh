#!/bin/sh
# Root-only, read-only capture of the Qualcomm CPU interconnect bandwidth
# monitor's existing kernel tracepoint.  This does not reprogram BWMON and it
# is not total DRAM traffic: qcom_bwmon_update reports CPU-path measurements
# when the in-kernel governor crosses a bandwidth/OPP threshold.
set -eu
runroot=${1:?usage: spike002_bwtrace.sh RUNROOT RUN_ID SECONDS READY_FILE CANDIDATE_UNIT}
rid=${2:?run id}
seconds=${3:?seconds}
ready=${4:?ready file}
cand_unit=${5:?candidate unit}
case $rid in *[!A-Za-z0-9._-]*|'') exit 2;; esac
case $seconds in *[!0-9]*|'') exit 2;; esac
case $cand_unit in *[!A-Za-z0-9._@-]*|'') exit 2;; esac
[ "$(id -u)" = 0 ] || exit 2

trace=/sys/kernel/debug/tracing
event=$trace/events/qcom_bwmon/qcom_bwmon_update/enable
tmp=/run/m1-s4-bwtrace-$rid
rm -rf "$tmp"; mkdir "$tmp"
status=unavailable
reason=
armed=0
owned=0
stop_trace() {
  if [ "$armed" = 1 ]; then
    echo 0 > "$trace/tracing_on" 2>/dev/null || true
    echo 0 > "$event" 2>/dev/null || true
    armed=0
  fi
}
finalize() {
  stop_trace
  if [ "$owned" = 1 ] && [ -w "$trace/trace" ]; then
    : > "$trace/trace" 2>/dev/null || true
    owned=0
  fi
}
trap 'finalize' EXIT
trap 'exit 3' INT TERM HUP

if [ ! -w "$event" ] || [ ! -w "$trace/tracing_on" ] || [ ! -w "$trace/trace" ]; then
  reason=tracepoint_unavailable
elif [ "$(cat "$trace/tracing_on" 2>/dev/null || echo 1)" != 0 ]; then
  reason=tracing_already_active
elif [ "$(cat "$event" 2>/dev/null || echo 1)" != 0 ]; then
  reason=event_already_active
elif [ "$(cat "$trace/current_tracer" 2>/dev/null || echo unknown)" != nop ]; then
  reason=non_nop_tracer_configured
elif [ -n "$(cat "$trace/set_event" 2>/dev/null || echo unreadable)" ]; then
  reason=other_events_configured
elif grep -qv '^#' "$trace/trace" 2>/dev/null; then
  reason=trace_buffer_not_empty
else
  : > "$trace/trace"
  echo 1 > "$event"
  echo 1 > "$trace/tracing_on"
  armed=1; owned=1; status=armed; reason=cpu_interconnect_transition_samples
fi
printf '%s\n' "$status:$reason" > "$ready"

# Wait for the separately managed candidate unit, then until it exits. The
# helper's systemd RuntimeMaxSec is an independent upper bound.
i=0
while ! systemctl is-active --quiet "$cand_unit"; do
  i=$((i + 1)); [ "$i" -lt 600 ] || { reason=candidate_never_started; break; }
  sleep .1
done
if systemctl is-active --quiet "$cand_unit"; then
  while systemctl is-active --quiet "$cand_unit"; do sleep 1; done
fi

stop_trace
ev=$runroot/$rid
i=0
while [ ! -d "$ev" ]; do
  i=$((i + 1)); [ "$i" -lt 100 ] || exit 4
  sleep .1
done
if [ "$status" = armed ]; then
  cat "$trace/trace" > "$tmp/bwmon-trace.txt"
  : > "$trace/trace"
  owned=0
else
  : > "$tmp/bwmon-trace.txt"
fi
{
  echo "schema=gx-spike002-bwmon/1"
  echo "status=$status"
  echo "reason=$reason"
  echo "scope=Qualcomm CPU interconnect only; transition-triggered samples, not continuous whole-system DRAM bandwidth"
  echo "event=qcom_bwmon_update"
} > "$tmp/bwmon-meta.txt"
chown arduino:arduino "$tmp/bwmon-trace.txt" "$tmp/bwmon-meta.txt"
chmod 644 "$tmp/bwmon-trace.txt" "$tmp/bwmon-meta.txt"
mv "$tmp/bwmon-trace.txt" "$tmp/bwmon-meta.txt" "$ev/"
rmdir "$tmp"
exit 0
