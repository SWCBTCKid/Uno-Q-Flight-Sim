#!/bin/sh
set -eu
bundle=${1:?usage: m1_s4_remote_collectable.sh BUNDLE RUN_DIR}
run=${2:?run dir}
case $bundle in /home/arduino/global-express-m0/m1/s4/deploy/*) ;; *) exit 30;; esac
case $run in /home/arduino/global-express-m0/m1/s4/runs/*) ;; *) exit 31;; esac
! pgrep -x gx-app >/dev/null 2>&1
[ "$(systemctl is-active m1-s4-window 2>/dev/null || true)" != active ]
[ "$(systemctl is-active m0-001c-candidate 2>/dev/null || true)" != active ]
[ "$(systemctl is-active m1-s4-bwtrace 2>/dev/null || true)" != active ]
[ -s "$run/s4-run.jsonl" ]
[ -s "$run/runctl.log" ]
[ -s "$run/state-before.txt" ]
[ -s "$run/state-after.txt" ]
grep -q 'RESTORE OK' "$run/runctl.log"
if [ -x "$bundle/spike002_bwtrace.sh" ]; then
  [ -s "$run/sampler.jsonl" ]
  [ -s "$run/bwmon-meta.txt" ]
  [ -f "$run/bwmon-trace.txt" ]
fi
if [ -e "$run/stream-ladder.ready" ] || [ -e "$run/stream-ladder.jsonl" ]; then
  [ -s "$run/stream-ladder.jsonl" ]
  [ -s "$run/stream-ladder.ready" ]
  [ -s "$run/stream-ladder.go" ]
  [ -s "$run/stream-ladder.pid" ]
  [ -s "$run/stream-ladder.exit" ]
  [ "$(cat "$run/stream-ladder.exit")" = 0 ]
  grep -q '"test":"stream_ladder_summary".*"full_validation":true' "$run/stream-ladder.jsonl"
fi
sh "$bundle/m1_s4_kms_window.sh" verify
