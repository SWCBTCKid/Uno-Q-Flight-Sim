#!/bin/sh
set -u

# Fed to `ssh BOARD sh -s -- RUN_DIR` by deploy_m1_s2.ps1: run every
# cross-built workspace test executable on the board (Linux tests and the
# JSBSim FFI tests the Windows host cannot link). Exit 0 only if all pass.
run_dir=${1:?usage: m1_s2_board_tests.sh RUN_DIR}
cd "$run_dir/tests"
GX_JSBSIM_ROOT="$run_dir/jsbsim"
export GX_JSBSIM_ROOT
failed=0
total=0
for t in *; do
    case "$t" in *.so|*.so.*) continue ;; esac
    total=$((total + 1))
    if ./"$t" --test-threads=1 --nocapture >"../test-$t.log" 2>&1; then
        echo "PASS $t: $(grep -E '^test result' "../test-$t.log" | tail -1)"
    else
        failed=$((failed + 1))
        echo "FAIL $t"
        tail -30 "../test-$t.log"
    fi
done
echo "board tests: $((total - failed))/$total executables passed"
[ "$failed" -eq 0 ]
