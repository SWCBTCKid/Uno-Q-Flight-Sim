#!/usr/bin/env python3
"""Static tests of the S4 visible-run envelope (no board needed):

* the accepted M0-001C controller is untouched, and the S4 controller differs
  from it by exactly the reviewed lines (LimitRTPRIO/LimitNICE on the
  candidate unit, ISS-038/BOARD-002) and is hash-pinned consistently;
* the launcher's realtime preflight uses the controller's exact properties,
  runs before anything is touched, and keeps the S3 bounds;
* the runner probes realtime inside the candidate unit before gx-app s4;
* the deploy preflight never treats the ssh session's `ulimit -r` as
  evidence, and deploy never invokes sudo or the visible run.

    py -I tools/test_m1_s4_platform.py
"""
import difflib
import hashlib
import os
import re
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
M0 = "c4c270059a22592f63195f3f58cef372bfb3598e1137042ebfa56fe18ad5febc"
RT = "-p LimitRTPRIO=60 -p LimitNICE=30"


def read(name):
    with open(os.path.join(HERE, name), "rb") as f:
        return f.read().decode("utf-8")


class Controller(unittest.TestCase):
    def test_m0_controller_unchanged(self):
        self.assertEqual(hashlib.sha256(read("m0_001c_kms_window.sh").encode()).hexdigest(), M0)

    def test_s4_controller_differs_only_by_the_rt_limits(self):
        old, new = read("m0_001c_kms_window.sh").splitlines(), read("m1_s4_kms_window.sh").splitlines()
        added = [l[1:] for l in difflib.unified_diff(old, new, lineterm="", n=0) if l.startswith("+") and not l.startswith("+++")]
        removed = [l[1:] for l in difflib.unified_diff(old, new, lineterm="", n=0) if l.startswith("-") and not l.startswith("---")]
        code_added = [l for l in added if not l.lstrip().startswith("#")]
        self.assertEqual(code_added, [
            '    log "MUT: systemd-run --unit=$CAND_UNIT --uid=$RUN_USER --wait -p RuntimeMaxSec=$hard ' + RT + ' ... timeout -k 5 $CAND_TIMEOUT $*"',
            "      " + RT + " \\",
        ])
        self.assertEqual(removed, ['    log "MUT: systemd-run --unit=$CAND_UNIT --uid=$RUN_USER --wait -p RuntimeMaxSec=$hard ... timeout -k 5 $CAND_TIMEOUT $*"'])
        # The property line sits inside the real candidate systemd-run.
        text = read("m1_s4_kms_window.sh")
        block = text[text.index("    systemd-run --unit=$CAND_UNIT --uid="):text.index("--wait --collect -- timeout")]
        self.assertIn(RT, block)

    def test_hash_pinned_in_build_and_launcher(self):
        h = hashlib.sha256(read("m1_s4_kms_window.sh").encode()).hexdigest()
        self.assertIn(f"$s4Controller='{h}'", read("build_m1_s4.ps1"))
        self.assertIn(f"$m0Controller='{M0}'", read("build_m1_s4.ps1"))
        self.assertIn(f"CONTROLLER_SHA256={h}", read("m1_s4_launch.sh"))


class Launcher(unittest.TestCase):
    def test_rt_preflight_matches_the_candidate_and_runs_before_the_takeover(self):
        text = read("m1_s4_launch.sh")
        self.assertIn(f"RT_PROPS='{RT}'", text)
        probe = text.index("systemd-run --unit=m1-s4-rtprobe --uid=arduino --gid=arduino $RT_PROPS --wait --collect --pipe --quiet -- \"$B/bin/gx-app\" rtprobe")
        refuse = text.index("die 'realtime preflight FAILED")
        display = text.index("sh \"$W\" preflight >/dev/null 2>&1 || {")
        takeover = text.index("$full || die")
        self.assertLess(display, probe)
        self.assertLess(probe, refuse)
        self.assertLess(refuse, takeover)

    def test_bounds_lock_and_paths(self):
        text = read("m1_s4_launch.sh")
        for s in ("tmo=$((seconds + 60))", "rmax=$((tmo + 420))", "relink_pair runuser", 'grep -q "$UNIT\\.service" /proc/self/cgroup',
                  "W=$B/m1_s4_kms_window.sh", "R=$B/m1_s4_run.sh", "UNIT=m1-s4-window", "CAND_UNIT=m0-001c-candidate",
                  "RUNROOT=/home/arduino/global-express-m0/m1/s4/runs"):
            self.assertIn(s, text)
        self.assertNotIn("systemctl stop lightdm", text)
        self.assertNotIn("m0_001c_kms_window.sh", text)

    def test_runner_probes_inside_the_candidate_unit_first(self):
        text = read("m1_s4_run.sh")
        self.assertLess(text.index("./bin/gx-app rtprobe > \"$ev/rtprobe.json\""), text.index("./bin/gx-app s4 --display kms"))
        self.assertIn("exit 23", text)

    def test_spike_runner_samples_the_real_candidate_pid(self):
        text = read("m1_s4_run.sh")
        self.assertIn('app_pid=$!', text)
        self.assertIn('sh ./spike002_sampler.sh "$app_pid" "$helper_pid" > "$ev/sampler.jsonl"', text)
        self.assertIn('wait "$app_pid"; app_rc=$?', text)

    def test_bandwidth_ladder_is_opt_in_gated_and_never_uses_render_core(self):
        launch = read("m1_s4_launch.sh")
        runner = read("m1_s4_run.sh")
        sampler = read("spike002_sampler.sh")
        self.assertIn("--bw-ladder", launch)
        self.assertIn("[ \"$ladder\" = off ] || [ \"$seconds\" = 300 ]", launch)
        self.assertEqual(runner.count('bw_ladder=${5:-off}'), 1)
        self.assertIn('--mode stream_ladder', runner)
        self.assertIn('[ ! -s "$gate.ready" ]', runner)
        self.assertLess(runner.index('[ ! -s "$gate.ready" ]'), runner.index('./bin/gx-app s4 --display kms'))
        self.assertGreater(runner.index('> "$gate.go"'), runner.index('app_pid=$!'))
        self.assertIn('sh ./spike002_sampler.sh "$app_pid" "$helper_pid"', runner)
        self.assertIn('helper=${2:-}', sampler)
        # The public AP-002 bundle intentionally omits the internal synthetic
        # bandwidth-load helper. The legacy option therefore remains gated
        # and fails closed if somebody tries to select it.
        self.assertIn('[ "$ladder" = off ] || [ -x "$B/bin/global-express-m0-budget" ] || die', launch)

    def test_ladder_evidence_is_collectable(self):
        remote = read("m1_s4_remote_collectable.sh")
        collector = read("collect_m1_s4.ps1")
        for name in ("stream-ladder.jsonl", "stream-ladder.ready", "stream-ladder.go", "stream-ladder.exit"):
            self.assertIn(name, remote)
            self.assertIn(name, collector)
        self.assertIn('"test":"stream_ladder_summary".*"full_validation":true', remote)


class Mouse(unittest.TestCase):
    """AP-001 W3 (R3): the launcher grants the run user a read ACL on the
    resolved mouse node for the run only, revokes it on exit and restore,
    and the runner hands the mouse to gx-app."""

    def test_grant_before_the_controller_and_revoke_after_it_exits(self):
        text = read("m1_s4_launch.sh")
        window = text[text.index("  window)"):text.index("  run)")]
        stale = window.index("mouse_revoke >/dev/null")
        grant = window.index('state=$(mouse_acl grant "$node" "$(id -u $RUN_USER)")')
        record = window.index('echo "$node $state" > "$MOUSE_ACL"')
        controller = window.index('"$W" run "$rid" -- "$R" "$B" "$seconds" "$DEVICE" "$aircraft" "$ladder" "$mouse" &')
        wait = window.index('while :; do wait "$c"; rc=$?; kill -0 "$c" 2>/dev/null || break; done')
        revoke = window.index("revoked=$(mouse_revoke)")
        first_exit = window.index('[ -d "$ev" ] || exit "$rc"')
        self.assertLess(stale, grant)
        self.assertLess(grant, record)
        self.assertLess(record, controller)
        self.assertLess(controller, wait)
        self.assertLess(wait, revoke)
        self.assertLess(revoke, first_exit, "revoked before any exit of the window")
        self.assertIn("mouse=none", window[:controller], "no mouse or a failed grant runs without it")

    def test_restore_revokes_and_status_shows_the_mouse(self):
        text = read("m1_s4_launch.sh")
        restore = text[text.index("  restore)"):text.index("  stop-deadman)")]
        self.assertLess(restore.index("mouse_revoke"), restore.index('exec sh "$W" restore manual'))
        status = text[text.index("  status)"):text.index("  relink)")]
        self.assertIn('echo "mouse: $m -> $(readlink -f "$m")', status)
        self.assertIn("mouse: none", status)
        self.assertIn("RUN_USER=arduino", text)
        self.assertIn("os.setxattr(node, K,", text, "the board has no setfacl")
        self.assertIn('if [ "$state" = added ]', text, "an entry the launcher did not add is left alone")

    def test_runner_passes_the_mouse_and_an_unreadable_one_is_not_fatal(self):
        runner = read("m1_s4_run.sh")
        self.assertIn("mouse=${6:-none}", runner)
        self.assertIn('[ "$mouse" = none ] || [ -r "$mouse" ] || {', runner)
        self.assertIn('--mouse "$mouse"', runner)
        self.assertLess(runner.index('[ -r "$mouse" ]'), runner.index("./bin/gx-app s4 --display kms"))


class NoTimer(unittest.TestCase):
    """AP-002 W7: no fixed flight time. Without SECONDS a run lasts until the
    panel's EXIT with a 10800 s backstop that every watchdog bound scales
    from; explicit SECONDS stay allowed in 30..10800; the launcher refuses a
    run with less than 1 GB free; the dry run shows the backstop."""

    def test_bounds_default_to_the_backstop_and_explicit_seconds_stay_bounded(self):
        launch = read("m1_s4_launch.sh")
        self.assertIn("BACKSTOP=10800\n", launch)
        self.assertIn('[ "$1" -ge 30 ] && [ "$1" -le $BACKSTOP ] || die "SECONDS must be 30..$BACKSTOP"', launch)
        self.assertNotIn("-le 300 ]", launch)
        run = launch[launch.index("  run)"):launch.index("  *) sed -n")]
        self.assertIn("seconds=$BACKSTOP; timer=exit;", run)
        self.assertIn("*) seconds=$a; timer=fixed ;;", run)
        self.assertIn('seconds_ok "$seconds"', run)
        # The candidate, dead-man and outer unit all derive from $seconds
        # (the backstop when no time is given), unchanged formulas.
        for s in ("tmo=$((seconds + 60))", "rmax=$((tmo + 420))", "-E CAND_TIMEOUT=$tmo $SELF window $rid $seconds"):
            self.assertIn(s, launch)
        runner = read("m1_s4_run.sh")
        self.assertIn('[ "$seconds" -ge 30 ] && [ "$seconds" -le 10800 ] ||', runner)
        self.assertNotIn("-le 300 ]", runner)

    def test_disk_check_refuses_before_anything_is_touched_and_the_dry_run_shows_the_backstop(self):
        launch = read("m1_s4_launch.sh")
        self.assertIn("MIN_FREE_KB=1048576\n", launch)
        refuse = launch.index('[ "$fkb" -ge "$MIN_FREE_KB" ] || die')
        dry = launch.index('[ $dry = 1 ] && { echo "DRY-RUN: $full"')
        display = launch.index('sh "$W" preflight >/dev/null 2>&1 || {')
        takeover = launch.index("$full || die")
        self.assertLess(refuse, dry, "the dry run applies the disk check too")
        self.assertLess(dry, display)
        self.assertLess(refuse, takeover)
        self.assertIn('flight="until EXIT (backstop ${seconds}s)"', launch)
        self.assertLess(launch.index('flight="until EXIT'), dry)
        self.assertIn("flight=$flight", launch)

    def test_no_message_promises_a_fixed_flight_time(self):
        launch = read("m1_s4_launch.sh")
        started = [l for l in launch.splitlines() if "STARTED:" in l]
        self.assertEqual(len(started), 2)
        self.assertIn("until you click EXIT twice", started[0])
        self.assertIn("EXIT ends it sooner", started[1])
        for text in (launch, read("m1_s4_run.sh")):
            self.assertNotRegex(text.lower(), r"\b(5|five)[ -]minute")


class Deploy(unittest.TestCase):
    def test_preflight_tests_the_candidate_envelope_not_the_ssh_session(self):
        text = read("m1_s4_remote_verify.sh")
        code = "\n".join(l for l in text.splitlines() if not l.lstrip().startswith("#"))
        self.assertNotIn("ulimit -r", code)
        self.assertIn("prlimit --rtprio=60:60 -- ./bin/gx-app rtprobe", code)
        self.assertIn("prlimit --rtprio=0:0 -- ./bin/gx-app rtprobe", code)
        self.assertIn("LimitRTPRIO=60 -p LimitNICE=30", code)
        self.assertTrue(re.search(r"DefaultLimitRTPRIO", code))

    def test_deploy_never_invokes_sudo_or_visible_run(self):
        text = read("deploy_m1_s4.ps1")
        before = "\n".join(l for l in text.split("$runCommand=", 1)[0].lower().splitlines() if not l.lstrip().startswith("#"))
        self.assertNotIn("sudo", before)
        self.assertNotIn("m1_s4_launch.sh run", before)

    def test_collect_pulls_the_in_unit_probe_with_hashes(self):
        text = read("collect_m1_s4.ps1")
        self.assertIn("'rtprobe.json'", text)
        self.assertIn("Get-FileHash -Algorithm SHA256", text)
        self.assertIn("'sampler.jsonl'", text)
        self.assertIn("'bwmon-trace.txt'", text)

    def test_bandwidth_trace_is_scoped_and_non_destructive(self):
        text = read("spike002_bwtrace.sh")
        self.assertIn("CPU interconnect", text)
        self.assertIn("not continuous whole-system DRAM bandwidth", text)
        self.assertIn("tracing_already_active", text)
        self.assertIn("non_nop_tracer_configured", text)
        self.assertIn("other_events_configured", text)
        self.assertIn(': > "$trace/trace"', text)
        self.assertIn("trap 'finalize' EXIT", text)
        controller = read("m1_s4_kms_window.sh")
        candidate = re.search(r"^CAND_UNIT=(\S+)$", controller, re.MULTILINE).group(1)
        self.assertIn(f"CAND_UNIT={candidate}", read("m1_s4_launch.sh"))
        self.assertIn('systemctl is-active --quiet "$cand_unit"', text)

    def test_remote_helpers_are_executable_and_collection_waits(self):
        verify = read("m1_s4_remote_verify.sh")
        collectable = read("m1_s4_remote_collectable.sh")
        self.assertIn("spike002_sampler.sh spike002_bwtrace.sh", verify)
        self.assertIn('chmod 700 "$helper"', verify)
        self.assertIn("systemctl is-active m1-s4-bwtrace", collectable)
        self.assertIn('[ -s "$run/sampler.jsonl" ]', collectable)
        self.assertIn('[ -s "$run/bwmon-meta.txt" ]', collectable)
        collector = read("collect_m1_s4.ps1")
        self.assertIn("$isSpike002=($null -ne $buildManifest.spike002)", collector)
        self.assertIn("if(!$isSpike002)", collector)


if __name__ == "__main__":
    unittest.main()
