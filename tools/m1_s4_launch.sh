#!/bin/sh
# M1 S4 visible-run launcher: one bounded direct-KMS window through the S4
# controller (the accepted M0-001C controller plus LimitRTPRIO=60/LimitNICE=30
# on the candidate unit, ISS-038; hash pinned), after a realtime preflight in
# that exact envelope; then
# one DP-1 off/auto relink after a verified restore (R-12), as the SPIKE-001b
# render-layout launcher. Every command is one quote-free line, safe to paste
# from Windows PowerShell 5.1.
#
# Usage (on the board, from the deployed bundle):
#   sudo m1_s4_launch.sh run RUN_ID [SECONDS] [--no-relink] [--bw-ladder]
#                                                             no SECONDS: fly until the panel's EXIT
#                                                             (double click), 10800 s backstop; else 30..10800
#   m1_s4_launch.sh --dry-run run RUN_ID [SECONDS]            print the plan, touch nothing
#   m1_s4_launch.sh status                                    read-only readiness
#   m1_s4_launch.sh relink                                    as arduino, no sudo: DP-1 off then auto, then verify
#   sudo m1_s4_launch.sh restore                              idempotent desktop restore (R-5/R-6); revokes the mouse ACL
#   sudo m1_s4_launch.sh stop-deadman                         R-11
#   (internal) window RUN_ID SECONDS auto|off                 only inside the m1-s4-window unit
set -u
B=$(cd "$(dirname "$0")" && pwd)
SELF=$B/$(basename "$0")
W=$B/m1_s4_kms_window.sh
R=$B/m1_s4_run.sh
CONTROLLER_SHA256=a6e1bbc252e14c884f5c6a5008700c0fbdbf5f804022c9dee640d1aab270a4f0
RUNROOT=/home/arduino/global-express-m0/m1/s4/runs
DEVICE=/dev/input/by-id/usb-Logitech_Logitech_Extreme_3D-event-joystick
# AP-001 autopilot panel mouse (R3): the node is root:input 0660 and the
# candidate unit has only video/render, so the window grants RUN_USER a read
# ACL on the resolved node for the run and revokes it when the controller
# exits (and on restore). The grant is recorded in MOUSE_ACL ("NODE added").
RUN_USER=arduino
MOUSE_ACL=/run/m1-s4-mouse.acl
# AP-002 W7: no fixed flight time. Without SECONDS the run lasts until the
# panel's EXIT; BACKSTOP still bounds the display takeover (candidate,
# dead-man and outer unit all scale from it as from SECONDS). The log grows
# ~3.2 MB/min (~0.6 GB in 3 h), so a run needs MIN_FREE_KB on RUNROOT's disk.
BACKSTOP=10800
MIN_FREE_KB=1048576
UNIT=m1-s4-window
CAND_UNIT=m0-001c-candidate
# The candidate unit's realtime limits: identical to the controller's
# candidate systemd-run line (tools/test_m1_s4_platform.py checks both).
RT_PROPS='-p LimitRTPRIO=60 -p LimitNICE=30'
die() { echo "S4 launch REFUSED: $*" >&2; exit 2; }
plain() { for a in "$@"; do case $a in *[!A-Za-z0-9._-]*|'') die "argument is not plain: $a";; esac; done; }

# DP-1 off/auto pair on the restored desktop, as arduino with its own cookie:
# the Chief's verified manual recovery (R-12). Fixed words; each call bounded.
relink_pair() {  # $1 = runuser when called as root
  if [ "${1:-}" = runuser ]; then
    /usr/sbin/runuser -u arduino -- timeout 10 env DISPLAY=:0 XAUTHORITY=/home/arduino/.Xauthority xrandr --output DP-1 --off || return 1
    sleep 2
    /usr/sbin/runuser -u arduino -- timeout 10 env DISPLAY=:0 XAUTHORITY=/home/arduino/.Xauthority xrandr --output DP-1 --auto || return 1
  else
    timeout 10 env DISPLAY=:0 XAUTHORITY=/home/arduino/.Xauthority xrandr --output DP-1 --off || return 1
    sleep 2
    timeout 10 env DISPLAY=:0 XAUTHORITY=/home/arduino/.Xauthority xrandr --output DP-1 --auto || return 1
  fi
}
# The first /dev/input/by-id/*-event-mouse (glob order = gx-app's choice).
find_mouse() {
  for m in /dev/input/by-id/*-event-mouse; do
    [ -e "$m" ] && { echo "$m"; return 0; }
  done
  return 1
}
# mouse_acl grant|revoke|show NODE UID: the POSIX access ACL as the kernel
# stores it (system.posix_acl_access); the board has no setfacl. grant adds
# u:UID:r-- (prints "added", or "present" if UID already had an entry, which
# revoke then leaves alone); revoke removes the u:UID entry; the mask is the
# union of the group class, as setfacl computes it.
mouse_acl() {
  python3 -I - "$@" <<'PY'
import os, stat, struct, sys
op, node, uid = sys.argv[1], sys.argv[2], int(sys.argv[3])
K, U = "system.posix_acl_access", 0xFFFFFFFF
USER_OBJ, USER, GROUP_OBJ, GROUP, MASK, OTHER = 1, 2, 4, 8, 0x10, 0x20
def entries():
    try:
        b = os.getxattr(node, K)
    except OSError:
        b = b""
    if not b:
        m = os.stat(node).st_mode
        return [(USER_OBJ, (m >> 6) & 7, U), (GROUP_OBJ, (m >> 3) & 7, U), (OTHER, m & 7, U)]
    assert struct.unpack_from("<I", b)[0] == 2, "unknown ACL version"
    return [struct.unpack_from("<HHI", b, 4 + 8 * i) for i in range((len(b) - 4) // 8)]
def store(e):
    e = [x for x in e if x[0] != MASK]
    if any(x[0] in (USER, GROUP) for x in e):
        mask = 0
        for t, p, i in e:
            if t in (USER, GROUP_OBJ, GROUP):
                mask |= p
        e = sorted(e + [(MASK, mask, U)], key=lambda x: (x[0], x[2]))
        os.setxattr(node, K, struct.pack("<I", 2) + b"".join(struct.pack("<HHI", *x) for x in e))
    else:
        try:
            os.removexattr(node, K)
        except OSError:
            pass
        g = [p for t, p, i in e if t == GROUP_OBJ][0]
        os.chmod(node, (stat.S_IMODE(os.stat(node).st_mode) & ~0o070) | (g << 3))
e = entries()
mine = [x for x in e if x[0] == USER and x[2] == uid]
if op == "show":
    print("read ACL for uid %d: %s" % (uid, "yes" if mine and mine[0][1] & 4 else "no"))
elif op == "grant":
    if mine:
        print("present")
    else:
        store(e + [(USER, 4, uid)])
        print("added")
elif op == "revoke":
    store([x for x in e if not (x[0] == USER and x[2] == uid)])
else:
    sys.exit("mouse_acl: grant|revoke|show")
PY
}
# Revoke a recorded grant (idempotent; the node may be gone after an unplug).
mouse_revoke() {
  [ -r "$MOUSE_ACL" ] || return 0
  read -r node state < "$MOUSE_ACL"
  if [ "$state" = added ] && [ -c "$node" ]; then
    mouse_acl revoke "$node" "$(id -u $RUN_USER)" && echo "mouse: revoked read ACL on $node" || echo "mouse: REVOKE FAILED on $node"
  fi
  rm -f "$MOUSE_ACL"
}
window_busy() {
  for u in $UNIT m0-001c-window m0-001c-candidate; do
    systemctl is-active --quiet "$u" 2>/dev/null && { echo "$u"; return 0; }
  done
  return 1
}
seconds_ok() {
  case $1 in *[!0-9]*|'') die 'SECONDS must be an integer';; esac
  [ "$1" -ge 30 ] && [ "$1" -le $BACKSTOP ] || die "SECONDS must be 30..$BACKSTOP"
}
# KiB available on the filesystem that holds RUNROOT (its nearest existing parent).
free_kb() {
  d=$RUNROOT
  while [ ! -d "$d" ]; do d=$(dirname "$d"); done
  df -Pk "$d" | awk 'NR == 2 { print $4 }'
}

dry=0
[ "${1:-}" = --dry-run ] && { dry=1; shift; }
op=${1:-}; [ $# -gt 0 ] && shift

case $op in
  status)
    printf 'units(window candidate deadman): '
    systemctl is-active $UNIT m0-001c-candidate m0-001c-deadman.timer 2>&1 | paste -sd' '
    [ -d /run/m0-001c-kms-window.lock ] && echo 'lock: held' || echo 'lock: free'
    [ -r "$DEVICE" ] && echo "joystick: $(readlink -f "$DEVICE")" || echo 'joystick: NOT READABLE'
    if m=$(find_mouse); then
      echo "mouse: $m -> $(readlink -f "$m") ($(mouse_acl show "$(readlink -f "$m")" "$(id -u $RUN_USER)" 2>&1))"
    else
      echo 'mouse: none (the panel will show NO MOUSE)'
    fi
    [ -e "$MOUSE_ACL" ] && echo "mouse grant recorded: $(cat "$MOUSE_ACL")"
    echo "disk: $(($(free_kb) / 1024)) MB free for runs (a run needs $((MIN_FREE_KB / 1024)) MB)"
    sh "$W" autologin
    sh "$W" seat0
    sh "$W" preflight >/dev/null 2>&1 && echo 'display preflight: READY' || echo 'display preflight: NOT READY (see m1_s4_kms_window.sh preflight)'
    exit 0 ;;
  relink)
    [ $# -eq 0 ] || die 'relink takes no arguments'
    if [ $dry = 1 ]; then echo 'DRY-RUN: xrandr --output DP-1 --off ; sleep 2 ; xrandr --output DP-1 --auto ; then verify'; exit 0; fi
    [ "$(id -un)" = arduino ] || die 'relink runs as arduino: omit sudo'
    u=$(window_busy) && die "$u is active; wait for the window to finish"
    echo "relink: $(date -u +%FT%TZ) DP-1 off then auto"
    relink_pair || die 'xrandr failed (is LightDM/Xorg up? runbook R-10)'
    LOG=/dev/stdout sh "$W" verify; exit $? ;;
  restore)
    [ $# -eq 0 ] || die 'restore takes no arguments'
    [ $dry = 1 ] && { echo "DRY-RUN: EVROOT=$RUNROOT $W restore manual"; exit 0; }
    [ "$(id -u)" = 0 ] || die 'restore needs sudo'
    mouse_revoke
    LOG=/dev/stdout EVROOT=$RUNROOT exec sh "$W" restore manual ;;
  stop-deadman)
    [ $# -eq 0 ] || die 'stop-deadman takes no arguments'
    [ $dry = 1 ] && { echo 'DRY-RUN: systemctl stop m0-001c-deadman.timer'; exit 0; }
    [ "$(id -u)" = 0 ] || die 'stop-deadman needs sudo'
    exec systemctl stop m0-001c-deadman.timer ;;
  window)
    # Inside the m1-s4-window transient unit (root): the S4 controller,
    # then one relink only after a verified restore.
    [ $# -eq 4 ] || die 'window RUN_ID SECONDS auto|off off|on'
    rid=$1; seconds=$2; mode=$3; ladder=$4
    plain "$rid" "$seconds" "$ladder"; seconds_ok "$seconds"
    case $mode in auto|off) ;; *) die 'relink must be auto|off' ;; esac
    case $ladder in on|off) ;; *) die 'bandwidth ladder must be on|off' ;; esac
    [ "$(id -u)" = 0 ] || die 'window mode needs root'
    grep -q "$UNIT\.service" /proc/self/cgroup 2>/dev/null || die "window mode runs only inside the $UNIT unit"
    # The bundle names its aircraft (hash-checked via files.sha256);
    # bundles without the file fly the M1 global5000.
    aircraft=global5000
    [ -r "$B/aircraft" ] && aircraft=$(head -n 1 "$B/aircraft")
    case $aircraft in global5000|787-8|c172p) ;; *) die "bundle names an unknown aircraft: $aircraft" ;; esac
    # AP-001: the panel mouse for this run only (R3). No mouse, or a failed
    # grant, runs without it (--mouse none; the panel shows NO MOUSE).
    mouse_revoke >/dev/null
    mouse=none; mouse_note='no mouse'
    if m=$(find_mouse); then
      node=$(readlink -f "$m")
      if [ -c "$node" ] && state=$(mouse_acl grant "$node" "$(id -u $RUN_USER)"); then
        echo "$node $state" > "$MOUSE_ACL"
        mouse=$m; mouse_note="$m -> $node read ACL $state"
      else
        mouse_note="read ACL grant FAILED on $m; running without the mouse"
      fi
    fi
    "$W" run "$rid" -- "$R" "$B" "$seconds" "$DEVICE" "$aircraft" "$ladder" "$mouse" &
    c=$!
    signalled=0
    # A handler (not an ignore) so the controller keeps its own traps; this
    # shell only waits for it to finish restoring.
    trap 'signalled=1' INT TERM HUP
    while :; do wait "$c"; rc=$?; kill -0 "$c" 2>/dev/null || break; done
    revoked=$(mouse_revoke)
    ev=$RUNROOT/$rid; log=$ev/relink.log
    [ -d "$ev" ] || exit "$rc"
    note() { printf '%s %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$*" >> "$log"; }
    note "mouse: $mouse_note; after the controller: ${revoked:-nothing to revoke}"
    note "controller exit=$rc relink=$mode signalled=$signalled"
    if [ "$signalled" = 1 ]; then note 'SKIP: window was signalled; manual relink if the monitor is blank'; exit "$rc"; fi
    if [ "$mode" = off ]; then note 'SKIP: --no-relink'; exit "$rc"; fi
    # 0 = restored; 10+N = candidate exit N, desktop restored. 2/3/4 are
    # refusals or incidents (R-5/R-8/R-10): never touch the display then.
    if [ "$rc" -ne 0 ] && [ "$rc" -lt 10 ]; then note "SKIP: controller exit $rc is not a verified restore"; exit "$rc"; fi
    sleep 3
    note 'RELINK begin: DP-1 off, 2 s, auto (as arduino)'
    if relink_pair runuser >> "$log" 2>&1; then note 'RELINK xrandr ok'
    else note 'RELINK xrandr FAILED: run the relink command by hand'; exit 3; fi
    LOG=$log sh "$W" verify || { note 'RELINK verify FAILED: INCIDENT, see runbook R-5/R-10'; exit 3; }
    note 'RELINK done'
    exit "$rc" ;;
  run)
    [ $# -ge 1 ] && [ $# -le 4 ] || die 'run RUN_ID [SECONDS] [--no-relink] [--bw-ladder]'
    rid=$1; shift
    seconds=$BACKSTOP; timer=exit; relink=auto; ladder=off
    for a in "$@"; do
      case $a in --no-relink) relink=off ;; --bw-ladder) ladder=on ;; *) seconds=$a; timer=fixed ;; esac
    done
    plain "$rid" "$seconds"; seconds_ok "$seconds"
    ;;
  *) sed -n '2,19p' "$0"; exit 2 ;;
esac

[ "$ladder" = off ] || [ "$seconds" = 300 ] || die '--bw-ladder requires a 300 second run'

(cd "$B" && sha256sum --quiet -c files.sha256) || die "bundle hash failure: $B"
[ "$(sha256sum "$W" | cut -d' ' -f1)" = "$CONTROLLER_SHA256" ] || die 'S4 controller differs from the reviewed one (M0-001C + LimitRTPRIO)'
[ -x "$W" ] && [ -x "$R" ] && [ -x "$B/bin/gx-app" ] || die 'controller, runner or gx-app not executable'
[ "$ladder" = off ] || [ -x "$B/bin/global-express-m0-budget" ] || die 'bandwidth ladder helper is absent'
[ -r "$DEVICE" ] || die "joystick is not readable: $DEVICE"
[ -e "$RUNROOT/$rid" ] && die "evidence dir exists: $RUNROOT/$rid"
# Candidate: flight + 60 s. Dead-man (controller): STOP_WAIT 30 + candidate
# + 30 + 120 after its start. Outer unit: dead-man + 240 s for that restore
# and the relink. Each is an independent cap.
tmo=$((seconds + 60))
rmax=$((tmo + 420))
full="systemd-run --unit=$UNIT --collect -p RuntimeMaxSec=$rmax -E EVROOT=$RUNROOT -E CAND_TIMEOUT=$tmo $SELF window $rid $seconds $relink $ladder"
fkb=$(free_kb)
if [ "$timer" = exit ]; then flight="until EXIT (backstop ${seconds}s)"; else flight="${seconds}s"; fi
echo "disk: $((fkb / 1024)) MB free for runs (needs $((MIN_FREE_KB / 1024)) MB; ~3.2 MB/min of log)"
[ "$fkb" -ge "$MIN_FREE_KB" ] || die "less than $((MIN_FREE_KB / 1024)) MB free under $RUNROOT; desktop untouched"
echo "run_id=$rid evidence=$RUNROOT/$rid flight=$flight candidate_timeout=${tmo}s deadman=+$((30 + tmo + 30 + 120))s outer=${rmax}s relink=$relink bw_ladder=$ladder"
[ $dry = 1 ] && { echo "DRY-RUN: $full"; echo "controller: $W run $rid -- $R $B $seconds $DEVICE global5000 $ladder MOUSE"; echo "mouse: $(find_mouse || echo none) (the window grants $RUN_USER a read ACL for the run and revokes it after)"; echo "rt preflight: systemd-run --unit=m1-s4-rtprobe --uid=arduino --gid=arduino $RT_PROPS --wait --collect --pipe --quiet -- $B/bin/gx-app rtprobe"; exit 0; }
[ "$(id -u)" = 0 ] || die 'run needs sudo'
u=$(window_busy) && die "$u is already active"
sh "$W" preflight >/dev/null 2>&1 || { sh "$W" preflight; die 'display preflight failed; desktop untouched'; }
# ISS-038: prove SCHED_FIFO the way the candidate will get it: a transient
# unit as arduino with exactly the controller's candidate limits ($RT_PROPS),
# before the desktop is touched. gx-app rtprobe exits 0 only with sim FIFO 50
# and I/O FIFO 40.
# shellcheck disable=SC2086
probe=$(systemd-run --unit=m1-s4-rtprobe --uid=arduino --gid=arduino $RT_PROPS --wait --collect --pipe --quiet -- "$B/bin/gx-app" rtprobe 2>&1); prc=$?
echo "realtime preflight (candidate envelope: $RT_PROPS): $probe"
[ "$prc" = 0 ] || die 'realtime preflight FAILED: the sim/I-O threads would not run SCHED_FIFO; desktop untouched'
# SPIKE-002 only: arm the root-owned Qualcomm BWMON trace helper before the
# display controller starts. It refuses to disturb an existing trace session
# and writes an explicit unavailable reason when the platform lacks the event.
if [ -x "$B/spike002_bwtrace.sh" ]; then
  bwready=/run/m1-s4-bwtrace.ready
  rm -f "$bwready"
  systemctl is-active --quiet m1-s4-bwtrace && die 'm1-s4-bwtrace is already active'
  systemd-run --unit=m1-s4-bwtrace --collect -p RuntimeMaxSec=$((seconds + 120)) \
    "$B/spike002_bwtrace.sh" "$RUNROOT" "$rid" "$seconds" "$bwready" "$CAND_UNIT" >/dev/null \
    || die 'could not start bandwidth trace helper; desktop untouched'
  i=0
  while [ ! -s "$bwready" ]; do
    i=$((i + 1)); [ "$i" -le 50 ] || die 'bandwidth trace helper did not arm; desktop untouched'
    sleep .1
  done
  echo "bandwidth instrumentation: $(cat "$bwready")"
fi
# Every word is a fixed path or a plain-validated [A-Za-z0-9._-] value.
# shellcheck disable=SC2086
$full || die 'systemd-run failed; desktop untouched'
if [ "$timer" = exit ]; then
  echo "STARTED: the monitor goes dark (LightDM stops), then the S4 SFO world until you click EXIT twice on the panel (or ${seconds} s at most); the desktop returns by itself."
else
  echo "STARTED: the monitor goes dark (LightDM stops), then the S4 SFO world for ${seconds} s (EXIT ends it sooner); the desktop returns by itself."
fi
echo "progress: systemctl status $UNIT   abort: sudo systemctl kill -s TERM $UNIT"
