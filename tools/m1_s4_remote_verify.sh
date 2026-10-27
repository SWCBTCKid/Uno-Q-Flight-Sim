#!/bin/sh
set -eu

# Non-disruptive S4 deploy verification (no sudo). Display readiness is
# reported but is not a deploy failure: the real launcher repeats it and
# refuses before mutation.
#
# Realtime (ISS-038). The ssh session's own `ulimit -r` says nothing about the
# visible run: the candidate runs in a systemd unit whose RLIMIT_RTPRIO comes
# from the unit, not from PAM limits.d. So this check tests the parts of the
# real path that an unprivileged user can test, and says which part it
# cannot:
#   1. the bundled S4 controller (hash-pinned) passes LimitRTPRIO=60 and
#      LimitNICE=30 to the candidate unit, and the launcher's realtime
#      preflight uses the identical properties;
#   2. the system manager's default is still RTPRIO 0 (why the property is
#      needed), and the kernel has no RT group scheduling, so an RLIMIT is
#      all a unit needs for SCHED_FIFO;
#   3. gx-app rtprobe, the run's own sim/I-O thread setup, gets FIFO 50/40
#      with RLIMIT_RTPRIO 60 and is refused (other_nice0) with RLIMIT_RTPRIO 0,
#      the S3 visible-run condition: the probe detects the failure mode.
# The definitive test, a transient system unit with the candidate's exact
# properties, needs root: the launcher runs it in the Chief's sudo `run`
# before the desktop is touched, and the runner repeats it inside the
# candidate unit (rtprobe.json in the evidence).
remote=${1:?usage: m1_s4_remote_verify.sh REMOTE DEVICE CONTROLLER_SHA256}
device=${2:?device}
controller_sha=${3:?controller sha256}
cd "$remote"
sha256sum --quiet -c files.sha256
chmod 700 bin/gx-app m1_s4_run.sh m1_s4_launch.sh m1_s4_kms_window.sh m1_s4_board_tests.sh m1_s4_smoke.sh
if [ -f bin/global-express-m0-budget ]; then chmod 700 bin/global-express-m0-budget; fi
for helper in spike002_sampler.sh spike002_bwtrace.sh; do
  [ ! -e "$helper" ] || { chmod 700 "$helper"; dash -n "$helper"; }
done
for t in tests/*; do case "$t" in *.so|*.so.*) ;; *) chmod 700 "$t";; esac; done
sha256sum m1_s4_kms_window.sh | grep -q "^$controller_sha " || { echo "S4 controller hash differs" >&2; exit 42; }
dash -n m1_s4_run.sh m1_s4_launch.sh m1_s4_kms_window.sh m1_s4_board_tests.sh m1_s4_smoke.sh
test -e "$device" && test -r "$device"
groups=$(id -Gn)
for g in input video render; do echo "$groups" | tr ' ' '\n' | grep -qx "$g" || { echo "missing group: $g" >&2; exit 40; }; done
test -r /dev/dri/card0 && test -w /dev/dri/card0
test -r /dev/dri/renderD128 && test -w /dev/dri/renderD128
./bin/gx-app s4 --help >/dev/null

# 1. The candidate unit's limits (static: the hash pin fixes the content).
grep -q '^      -p LimitRTPRIO=60 -p LimitNICE=30 \\$' m1_s4_kms_window.sh || { echo "controller does not set LimitRTPRIO on the candidate" >&2; exit 43; }
grep -q "^RT_PROPS='-p LimitRTPRIO=60 -p LimitNICE=30'\$" m1_s4_launch.sh || { echo "launcher realtime preflight properties differ" >&2; exit 44; }
# 2. Manager default and kernel.
default_rt=$(systemctl show -p DefaultLimitRTPRIO --value 2>/dev/null || echo unknown)
rt_runtime=$(cat /proc/sys/kernel/sched_rt_runtime_us)
if ls /sys/fs/cgroup/*/cpu.rt_runtime_us /sys/fs/cgroup/cpu.rt_runtime_us >/dev/null 2>&1; then
  echo "RT group scheduling present: a unit cgroup may have no RT budget" >&2; exit 45
fi
[ "$rt_runtime" = -1 ] || [ "$rt_runtime" -gt 0 ] || { echo "sched_rt_runtime_us=$rt_runtime" >&2; exit 46; }
# 3. The run's own thread setup under RLIMIT_RTPRIO 60 and 0.
with=$(prlimit --rtprio=60:60 -- ./bin/gx-app rtprobe) || { echo "rtprobe FAILED with RLIMIT_RTPRIO 60: $with" >&2; exit 47; }
if without=$(prlimit --rtprio=0:0 -- ./bin/gx-app rtprobe); then
  echo "rtprobe claims FIFO with RLIMIT_RTPRIO 0 (negative control failed): $without" >&2; exit 48
fi
echo "joystick=$(readlink -f "$device") groups=$groups drm=rw"
echo "RT: candidate unit LimitRTPRIO=60 LimitNICE=30 (controller $controller_sha); manager DefaultLimitRTPRIO=$default_rt; no RT group sched; sched_rt_runtime_us=$rt_runtime"
echo "RT probe rlimit 60: $with"
echo "RT probe rlimit 0 (negative control, the S3 visible-run condition): $without"
echo "RT NOTE: the transient system unit itself is probed by the launcher (sudo run) before the desktop is touched, and again inside the candidate unit (rtprobe.json)."
if sh ./m1_s4_kms_window.sh preflight >/dev/null 2>&1; then
  echo 'DISPLAY_PREFLIGHT=READY'
else
  echo 'DISPLAY_PREFLIGHT=NOT_READY (deploy remains valid; launcher will refuse before stopping LightDM)'
  sh ./m1_s4_kms_window.sh preflight || true
fi
