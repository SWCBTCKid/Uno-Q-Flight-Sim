#!/usr/bin/env python3
"""Apply the audited M1 timing hook to JSBSim's standalone driver only.

The fetched library/model source is untouched.  Exact anchor checks make this
fail closed if the pinned v1.3.2 driver is not the expected source.
"""
from pathlib import Path
import sys

p = Path(sys.argv[1]) / "src" / "JSBSim.cpp"
s = p.read_text(encoding="utf-8")
if "--m1-timing" in s:
    raise SystemExit(0)

def once(old, new):
    global s
    if s.count(old) != 1:
        raise SystemExit(f"JSBSim timing patch anchor count {s.count(old)} (expected 1): {old[:60]!r}")
    s = s.replace(old, new)

once("#include <iostream>\n#include <cstdlib>\n", "#include <iostream>\n#include <cstdlib>\n#include <errno.h>\n#include <fstream>\n#include <iomanip>\n#include <stdint.h>\n")
once("double sleep_period=0.01;\n", "double sleep_period=0.01;\nstring m1_timing_path;\nvector<uint64_t> m1_slot, m1_start, m1_end;\n")
once("  result = FDMExec->Run();  // MAKE AN INITIAL RUN\n", "  result = FDMExec->Run();  // MAKE AN INITIAL RUN (load warm-up, excluded from M1 timing)\n")
once("  frame_duration = FDMExec->GetDeltaT();\n", "  frame_duration = FDMExec->GetDeltaT();\n  uint64_t m1_epoch_ns = 0, m1_step = 0;\n  if (!m1_timing_path.empty()) {\n    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);\n    m1_epoch_ns = uint64_t(ts.tv_sec)*1000000000ULL + uint64_t(ts.tv_nsec) + 2000000ULL;\n  }\n")
once("        result = FDMExec->Run();\n\n        if (play_nice)", "        if (!m1_timing_path.empty()) {\n          uint64_t slot = m1_epoch_ns + uint64_t((long double)m1_step * 1000000000.0L * frame_duration);\n          struct timespec ts{time_t(slot/1000000000ULL), long(slot%1000000000ULL)};\n          while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {}\n          struct timespec q; clock_gettime(CLOCK_MONOTONIC, &q);\n          uint64_t a=uint64_t(q.tv_sec)*1000000000ULL+uint64_t(q.tv_nsec);\n          result = FDMExec->Run();\n          clock_gettime(CLOCK_MONOTONIC, &q);\n          uint64_t b=uint64_t(q.tv_sec)*1000000000ULL+uint64_t(q.tv_nsec);\n          m1_slot.push_back(slot); m1_start.push_back(a); m1_end.push_back(b); ++m1_step;\n        } else {\n          result = FDMExec->Run();\n        }\n\n        if (play_nice)")
once("  // PRINT ENDING CLOCK TIME\n", "  if (!m1_timing_path.empty()) {\n    ofstream tf(m1_timing_path);\n    tf << \"step\\tslot_ns\\tstart_ns\\tend_ns\\tdt\\n\" << setprecision(17);\n    for (size_t i=0; i<m1_slot.size(); ++i)\n      tf << i+1 << '\\t' << m1_slot[i] << '\\t' << m1_start[i] << '\\t' << m1_end[i] << '\\t' << frame_duration << '\\n';\n  }\n\n  // PRINT ENDING CLOCK TIME\n")
once("    } else if (keyword == \"--realtime\") {\n", "    } else if (keyword == \"--m1-timing\") {\n      if (n != string::npos) m1_timing_path = value; else { gripe; result = false; }\n    } else if (keyword == \"--realtime\") {\n")
once("    cout << \"    --realtime  specifies to run in actual real world time\" << endl;\n", "    cout << \"    --realtime  specifies to run in actual real world time\" << endl;\n    cout << \"    --m1-timing=<file>  M1 paced per-Run timing TSV (standalone driver only)\" << endl;\n")
p.write_text(s, encoding="utf-8", newline="\n")
