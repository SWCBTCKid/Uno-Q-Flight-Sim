#!/bin/sh
# M0-001C exclusive direct-KMS window: stop desktop, run one candidate under a
# hard timeout, ALWAYS restore LightDM + DP-1 1920x1080@60.
#
# NOT APPROVED FOR EXECUTION until the Chief approves
# docs/platform/m0-001c-kms-runbook.md. Only `preflight`, `verify` and
# DRY_RUN=1 are non-disruptive.
#
# Usage (on the board):
#   m0_001c_kms_window.sh preflight              read-only, any user
#   m0_001c_kms_window.sh verify                 read-only restore check
#   sudo m0_001c_kms_window.sh run RUN_ID -- CMD [ARGS...]
#   sudo m0_001c_kms_window.sh restore [REASON]  idempotent recovery
# Env: DRY_RUN=1 (echo mutations; candidate still runs locally under timeout),
#      CAND_TIMEOUT (s, default 660), STOP_WAIT (30), START_WAIT (90),
#      DEADMAN_MARGIN (120), EVROOT, RUN_USER (arduino).
# The candidate command gets env M0C_RUN_ID, M0C_EV (evidence dir, also its
# cwd) and M0C_CAND_TIMEOUT; see m0_001c_candidate_run.sh.
# Exit: 0 ok; 2 usage/precondition refused; 3 restore FAILED (incident);
#       4 GREETER-ONLY: LightDM/Xorg back but no autologin session on seat0
#         (incident; log in at the monitor or restart lightdm, runbook R-10);
#       10+N candidate exit N (desktop restored).
set -u

RUN_USER=${RUN_USER:-arduino}
RUN_HOME=/home/$RUN_USER
EVROOT=${EVROOT:-$RUN_HOME/global-express-m0/m0-001c/runs}
CAND_TIMEOUT=${CAND_TIMEOUT:-660}
STOP_WAIT=${STOP_WAIT:-30}
START_WAIT=${START_WAIT:-90}
DEADMAN_MARGIN=${DEADMAN_MARGIN:-120}
DRY_RUN=${DRY_RUN:-0}
DM=lightdm
CONNECTOR=DP-1
ORIG_MODE=1920x1080
ORIG_RATE=60
CAND_UNIT=m0-001c-candidate
DEADMAN_UNIT=m0-001c-deadman
LOCK=/run/m0-001c-kms-window.lock
SELF=$(readlink -f "$0" 2>/dev/null || echo "$0")
LOG=${LOG:-/dev/stderr}

log() { printf '%s %s\n' "$(date -u +%Y-%m-%dT%H:%M:%S.%NZ 2>/dev/null || date)" "$*" >>"$LOG"; }
# mut: every state-changing command goes through here.
mut() { log "MUT: $*"; [ "$DRY_RUN" = 1 ] && return 0; "$@"; }
die() { log "REFUSED: $*"; exit 2; }

AUTOLOGIN_CONF=${AUTOLOGIN_CONF:-/etc/lightdm/lightdm.conf.d/50-autologin.conf}

xauth_file() {  # the X server's own auth file, from the Xorg command line (TEST-015)
  pgrep -a -x Xorg 2>/dev/null | sed -n 's/.* -auth \([^ ]*\).*/\1/p' | head -1
}

xr() {  # read-only xrandr on :0
  if [ "$(id -u)" = 0 ]; then
    # root: use the server's auth, never the user's cookie (a stale user cookie
    # made window A's verify pass without a desktop session, TEST-015)
    a=$(xauth_file)
    [ -n "$a" ] && [ -r "$a" ] || { echo "no readable Xorg -auth file" >&2; return 1; }
    env DISPLAY=:0 XAUTHORITY="$a" xrandr --current
  else
    # unprivileged read-only checks only (preflight/verify by arduino)
    env DISPLAY=:0 XAUTHORITY="$RUN_HOME/.Xauthority" xrandr --current
  fi
}

mode_ok() {  # original desktop mode active on DP-1?
  out=$(xr 2>/dev/null) || return 1
  printf '%s\n' "$out" | grep -q "^$CONNECTOR connected.* $ORIG_MODE+0+0" || return 1
  printf '%s\n' "$out" | grep -Eq "^ +$ORIG_MODE +$ORIG_RATE\.[0-9]+\*" || return 1
}

seat0() {  # one line per seat0 session: "user class active state type"
  loginctl list-sessions --no-legend 2>/dev/null | while read -r sid _uid user seat _rest; do
    [ "$seat" = seat0 ] || continue
    echo "$user $(loginctl show-session "$sid" -p Class --value 2>/dev/null)" \
      "$(loginctl show-session "$sid" -p Active --value 2>/dev/null)" \
      "$(loginctl show-session "$sid" -p State --value 2>/dev/null)" \
      "$(loginctl show-session "$sid" -p Type --value 2>/dev/null)"
  done
}
user_session_ok() { seat0 | grep -q "^$RUN_USER user yes active x11\$"; }
greeter_only() { ! user_session_ok && seat0 | grep -q ' greeter '; }

autologin_ok() {  # Chief-installed TEST-015 config must be present and exact
  [ -r "$AUTOLOGIN_CONF" ] || return 1
  grep -qx '\[Seat:\*\]' "$AUTOLOGIN_CONF" &&
  grep -qx "autologin-user=$RUN_USER" "$AUTOLOGIN_CONF" &&
  grep -qx 'autologin-user-timeout=0' "$AUTOLOGIN_CONF" &&
  [ "$(grep -cv '^[[:space:]]*\(#.*\)\{0,1\}$' "$AUTOLOGIN_CONF")" = 3 ]
}

snapshot() {  # read-only state dump to stdout
  echo "## $(date -u +%FT%TZ) $1"
  echo "dm=$(systemctl is-active $DM 2>&1) enabled=$(systemctl is-enabled $DM 2>&1)"
  echo "xorg_pids=$(pgrep -x Xorg | tr '\n' ' ')"
  for f in /sys/class/drm/card*-$CONNECTOR/status /sys/class/drm/card*-$CONNECTOR/enabled /sys/class/drm/card*-$CONNECTOR/dpms; do
    [ -r "$f" ] && echo "$f=$(cat "$f")"
  done
  echo "candidate_unit=$(systemctl is-active $CAND_UNIT 2>&1) deadman=$(systemctl is-active $DEADMAN_UNIT.timer 2>&1)"
  echo "xorg_auth=$(xauth_file) autologin_conf_sha256=$(sha256sum "$AUTOLOGIN_CONF" 2>&1 | cut -d' ' -f1)"
  echo "seat0_sessions:"; seat0
  if [ "$(id -u)" = 0 ]; then echo "card0_holders:"; fuser -v /dev/dri/card0 2>&1; fi
  loginctl list-sessions --no-legend 2>&1
  xr 2>&1 | head -3
}

cmd_preflight() {
  snapshot preflight
  rc=0
  [ "$(systemctl is-active $DM)" = active ] || { echo "PRECONDITION: $DM not active"; rc=2; }
  mode_ok || { echo "PRECONDITION: $CONNECTOR not at ${ORIG_MODE}@${ORIG_RATE}"; rc=2; }
  autologin_ok || { echo "PRECONDITION: $AUTOLOGIN_CONF missing or not exactly [Seat:*] autologin-user=$RUN_USER autologin-user-timeout=0"; rc=2; }
  user_session_ok || { echo "PRECONDITION: no active $RUN_USER x11 user session on seat0"; rc=2; }
  systemctl is-active --quiet $CAND_UNIT && { echo "PRECONDITION: $CAND_UNIT already running"; rc=2; }
  echo "preflight rc=$rc"; return $rc
}

cmd_verify() {  # bounded wait for full desktop recovery; 0 ok, 3 failed, 4 greeter-only
  # test hook: DRY_RUN with DRY_VERIFY_RC simulates a verify outcome (3 or 4)
  [ "$DRY_RUN" = 1 ] && [ -n "${DRY_VERIFY_RC:-}" ] && { log "VERIFY (dry-run): simulated rc=$DRY_VERIFY_RC"; return "$DRY_VERIFY_RC"; }
  auth=user-cookie; [ "$(id -u)" = 0 ] && auth=server-auth
  i=0
  while [ $i -lt "$START_WAIT" ]; do
    if [ "$(systemctl is-active $DM)" = active ] && pgrep -x Xorg >/dev/null && mode_ok && user_session_ok; then
      log "VERIFY OK: $DM active, Xorg up, $CONNECTOR ${ORIG_MODE}@${ORIG_RATE} ($auth), $RUN_USER session active on seat0"
      return 0
    fi
    [ "$DRY_RUN" = 1 ] && { log "VERIFY (dry-run): skipping wait"; return 0; }
    sleep 1; i=$((i + 1))
  done
  if [ "$(systemctl is-active $DM)" = active ] && pgrep -x Xorg >/dev/null && greeter_only; then
    log "VERIFY GREETER-ONLY after ${START_WAIT}s: $DM and Xorg up, greeter on seat0, no $RUN_USER session"
    return 4
  fi
  log "VERIFY FAILED after ${START_WAIT}s: dm=$(systemctl is-active $DM) xorg=$(pgrep -x Xorg | tr '\n' ' ') seat0=[$(seat0 | tr '\n' ';')]"
  return 3
}

greeter_incident() {
  log "RESTORE GREETER-ONLY: INCIDENT. Not restarting again (restarts do not create the session)."
  log "  Action: log in as $RUN_USER at the monitor, or: sudo systemctl restart $DM ; then: $SELF verify"
  log "  Check $AUTOLOGIN_CONF. Dead-man disarmed (runbook R-10)."
  mut systemctl stop $DEADMAN_UNIT.timer 2>/dev/null || true
}

cmd_restore() {  # idempotent; safe to call any number of times, from any path
  reason=${1:-manual}
  log "RESTORE begin reason=$reason"
  # 1. end the candidate (cgroup-wide; SIGTERM then SIGKILL via unit stop)
  if [ "$DRY_RUN" = 1 ] || systemctl is-active --quiet $CAND_UNIT; then
    mut systemctl stop $CAND_UNIT
  fi
  mut systemctl kill -s KILL $CAND_UNIT 2>/dev/null || true
  # 2. bring the desktop back
  mut systemctl reset-failed $DM 2>/dev/null || true
  mut systemctl start $DM
  cmd_verify; rc=$?
  if [ $rc = 3 ]; then
    # 3. one corrective mode set (server auth), then one display-manager restart
    log "RESTORE fallback: explicit mode set"
    a=$(xauth_file)
    [ -n "$a" ] && mut env DISPLAY=:0 XAUTHORITY="$a" xrandr --output $CONNECTOR --mode $ORIG_MODE --rate $ORIG_RATE
    cmd_verify; rc=$?
    if [ $rc = 3 ]; then
      log "RESTORE fallback: restart $DM"
      mut systemctl restart $DM
      cmd_verify; rc=$?
    fi
  fi
  # 4. disarm the dead-man only after a verified restore (or a greeter-only state,
  #    which a further automatic restart cannot fix)
  case $rc in
    0) mut systemctl stop $DEADMAN_UNIT.timer 2>/dev/null || true; log "RESTORE OK" ;;
    4) greeter_incident ;;
    *) log "RESTORE FAILED: INCIDENT. Dead-man left armed. See runbook R-5/R-8." ;;
  esac
  return $rc
}

cmd_run() {
  [ $# -ge 3 ] && [ "$2" = "--" ] || die "usage: run RUN_ID -- CMD [ARGS...]"
  run_id=$1; shift 2
  case $run_id in *[!A-Za-z0-9._-]*|'') die "bad RUN_ID";; esac
  [ "$DRY_RUN" = 1 ] || [ "$(id -u)" = 0 ] || die "run needs root (sudo)"
  [ -x "$1" ] || die "candidate not executable: $1"
  ev=$EVROOT/$run_id
  [ -e "$ev" ] && die "evidence dir exists: $ev"
  # runs/ (EVROOT) must be arduino-owned too: the packer writes nothing beside
  # a run dir any more (packs/), but a root-owned runs/ also blocks arduino tools
  [ -d "$EVROOT" ] || { mkdir -p "$EVROOT" && { [ "$DRY_RUN" = 1 ] || chown "$RUN_USER:$RUN_USER" "$EVROOT"; }; }
  [ "$DRY_RUN" = 1 ] || [ "$(stat -c %U "$EVROOT")" = "$RUN_USER" ] || chown "$RUN_USER:$RUN_USER" "$EVROOT"
  mkdir -p "$ev" || die "cannot create $ev"
  [ "$DRY_RUN" = 1 ] || chown "$RUN_USER:$RUN_USER" "$ev"
  LOG=$ev/runctl.log; export LOG
  lockdir=$LOCK; [ "$DRY_RUN" = 1 ] && lockdir=$ev/.lock
  mkdir "$lockdir" 2>/dev/null || die "another window holds $lockdir"
  log "RUN $run_id self_sha256=$(sha256sum "$SELF" | cut -d' ' -f1) cand=$* timeout=$CAND_TIMEOUT dry=$DRY_RUN"
  cmd_preflight >"$ev/state-before.txt" 2>&1 || { [ "$DRY_RUN" = 1 ] || { rmdir "$lockdir"; die "preflight failed (see state-before.txt)"; }; }

  restored=0; cpid=
  finish() {
    trap '' INT TERM HUP
    [ -n "${cpid:-}" ] && kill -TERM "$cpid" 2>/dev/null
    if [ $restored = 0 ]; then restored=1; cmd_restore "${1:-exit}"; rrc=$?; else rrc=0; fi
    snapshot after >"$ev/state-after.txt" 2>&1
    rmdir "$lockdir" 2>/dev/null
    return $rrc
  }
  trap 'finish signal; exit 3' INT TERM HUP

  # External dead-man: a root systemd timer that runs `restore` even if this
  # shell is SIGKILLed, the SSH link drops, or the candidate wedges.
  deadman=$((STOP_WAIT + CAND_TIMEOUT + 30 + DEADMAN_MARGIN))
  mut systemd-run --unit=$DEADMAN_UNIT --on-active=${deadman}s \
    --timer-property=AccuracySec=1s --collect \
    -E LOG="$ev/deadman.log" "$SELF" restore deadman \
    || { rmdir "$lockdir"; die "cannot arm dead-man; desktop untouched"; }
  log "dead-man armed: +${deadman}s"

  # Stop the desktop and wait (bounded) for Xorg to release the card.
  mut systemctl stop $DM
  i=0
  while [ "$DRY_RUN" != 1 ] && pgrep -x Xorg >/dev/null; do
    i=$((i + 1)); [ $i -gt "$STOP_WAIT" ] && { log "Xorg did not exit in ${STOP_WAIT}s"; finish xorg-stuck; exit 3; }
    sleep 1
  done
  [ "$DRY_RUN" = 1 ] || fuser -v /dev/dri/card0 >"$ev/card0-holders.txt" 2>&1

  # Candidate: unprivileged, own cgroup, two independent hard limits.
  hard=$((CAND_TIMEOUT + 15))
  log "candidate start"
  if [ "$DRY_RUN" = 1 ]; then
    log "MUT: systemd-run --unit=$CAND_UNIT --uid=$RUN_USER --wait -p RuntimeMaxSec=$hard ... timeout -k 5 $CAND_TIMEOUT $*"
    (cd "$ev" && exec env M0C_RUN_ID="$run_id" M0C_EV="$ev" M0C_CAND_TIMEOUT="$CAND_TIMEOUT" \
      timeout -k 5 "$CAND_TIMEOUT" "$@" >"$ev/candidate.out" 2>&1) &
  else
    systemd-run --unit=$CAND_UNIT --uid="$RUN_USER" --gid="$RUN_USER" \
      -p SupplementaryGroups="video render" -p RuntimeMaxSec=$hard \
      -p TimeoutStopSec=5 -p KillMode=control-group \
      -p WorkingDirectory="$ev" \
      -E M0C_RUN_ID="$run_id" -E M0C_EV="$ev" -E M0C_CAND_TIMEOUT="$CAND_TIMEOUT" \
      -p StandardOutput=file:"$ev/candidate.out" -p StandardError=file:"$ev/candidate.err" \
      --wait --collect -- timeout -k 5 "$CAND_TIMEOUT" "$@" &
  fi
  # Background + wait so INT/TERM/HUP reach the trap immediately (sh defers
  # traps while a foreground child runs).
  cpid=$!; wait $cpid; crc=$?
  log "candidate exit=$crc (124=timeout, 137=killed)"
  finish normal; rrc=$?
  trap - INT TERM HUP
  [ $rrc = 0 ] || exit $rrc
  [ $crc = 0 ] && exit 0
  exit $((10 + (crc > 200 ? 200 : crc)))
}

sub=${1:-}; [ $# -gt 0 ] && shift
case $sub in
  preflight) cmd_preflight ;;
  verify)    cmd_verify ;;
  restore)   [ "$DRY_RUN" = 1 ] || [ "$(id -u)" = 0 ] || die "restore needs root"; cmd_restore "${1:-manual}" ;;
  autologin) autologin_ok && echo "autologin OK: $AUTOLOGIN_CONF" || { echo "autologin NOT OK: $AUTOLOGIN_CONF"; exit 2; } ;;
  seat0)     seat0; user_session_ok && echo "seat0: $RUN_USER session active" || { greeter_only && echo "seat0: GREETER-ONLY" || echo "seat0: no session"; exit 4; } ;;
  run)       cmd_run "$@" ;;
  *) sed -n '2,20p' "$0"; exit 2 ;;
esac
