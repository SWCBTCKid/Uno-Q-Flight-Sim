#!/bin/sh
set -u

# Fed to `ssh BOARD sh -s -- BUNDLE` by deploy_m1_s4.ps1 (or run in the
# bundle): every cross-built workspace test executable on the board,
# including the JSBSim FFI tests and the GPU offscreen render tests
# (gx-render tests/offscreen.rs, S3 frame and S4 world, on the render node: no DRM master, the
# desktop is untouched). Exit 0 only if all pass.
run_dir=${1:?usage: m1_s4_board_tests.sh BUNDLE}
cd "$run_dir/tests"
GX_JSBSIM_ROOT="$run_dir/jsbsim"
GX_RENDER_OFFSCREEN_TEST=1
export GX_JSBSIM_ROOT GX_RENDER_OFFSCREEN_TEST
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
grep -h 'GPU time over\|^renderer \|^S4 world\|probes compared' ../test-offscreen-test.log 2>/dev/null
echo "board tests: $((total - failed))/$total executables passed"
[ "$failed" -eq 0 ]
