#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Must-fire mutations for the meson-gxl-ddr-pub-mon KUnit suite.

Each mutation breaks one SC-reader contract in meson-gxl-ddr-pub-mon.c; the
suite is rebuilt and run with kunit.py, and the case named for that contract
must FAIL. A mutation that does not apply, or a run that does not produce a
result for that case, is VOID -- never a pass. The source is restored from
its in-memory original afterwards.

Usage (from the kernel tree root, on a host with qemu-system-x86_64):
    python3 drivers/soc/amlogic/meson-gxl-ddr-pub-mon-must-fire.py BUILD_DIR
Exit 0 only when every mutation fired its named case.
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TREE = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
SRC = os.path.join(HERE, "meson-gxl-ddr-pub-mon.c")

MUTATIONS = [
    ("K1 copy accepts an odd (mid-write) seq",
     "\t\tif (!seq || (seq & 1))\n",
     "\t\tif (!seq)\n",
     "pubmon_copy_odd_seq_is_busy"),
    ("K2 copy skips the magic re-check",
     "\t\tif (readl(sc + SC_OFF_MAGIC) != MESON_DDR_PUB_SC_MAGIC)\n"
     "\t\t\tgoto busy;\t/* block lost/rewritten mid-pass */\n",
     "",
     "pubmon_copy_lost_magic_is_busy"),
    # The odd-seq refusal happens before any field is read, so only a
    # refusal that comes AFTER the fields (the magic re-check) can show a
    # copy writing into the kept one.
    ("K3 copy fills the kept copy directly (no scratch)",
     "\tstruct pubmon_sc_data *d = pub->sc_scratch;\n",
     "\tstruct pubmon_sc_data *d = pub->sc_live;\n",
     "pubmon_copy_lost_magic_is_busy"),
    ("K4 gate ignores a pending reset",
     "\tif (pub->reset_pending)\n\t\treturn -EBUSY;\n",
     "",
     "pubmon_gate_reset_pending_is_busy"),
    ("K5 gate never dates liveness from movement",
     "\t\t\tpub->live_at = pub->last_obs;\n",
     "\t\t\tpub->live_at = ktime_set(0, 0);\n",  # an empty body is -Werror
     "pubmon_gate_movement_proves_liveness"),
    ("K6 SC-mode resume is not counted (L3)",
     "\t\tn = ++pub->resumes;\n",
     "\t\tn = pub->resumes;\n",
     "pubmon_sc_resume_is_counted"),
    ("K7 SC-mode header reports the SC's sleep count as resumes (L3)",
     "\t\t   samples, elapsed_ms, period_ms, resumes);\n",
     "\t\t   samples, elapsed_ms, period_ms, (u64)d->sleeps + 0 * resumes);\n",
     "pubmon_sc_stats_resumes_is_the_kernel_count"),
]


def run(build_dir):
    r = subprocess.run(
        [sys.executable, "tools/testing/kunit/kunit.py", "run",
         "--kunitconfig=drivers/soc/amlogic", "--arch=x86_64",
         "--build_dir=" + build_dir, "--jobs=32"],
        cwd=TREE, capture_output=True, text=True)
    return r.stdout + r.stderr


def result(out, case):
    m = re.search(r"\[(PASSED|FAILED|CRASHED|SKIPPED)\] " + re.escape(case) + r"\b", out)
    return m.group(1) if m else None


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    build_dir = sys.argv[1]
    orig = open(SRC).read()
    fired = 0
    try:
        for name, old, new, case in MUTATIONS:
            if old not in orig:
                print("%s: MUTATION DID NOT APPLY -- VOID" % name)
                continue
            open(SRC, "w").write(orig.replace(old, new, 1))
            res = result(run(build_dir), case)
            if res is None:
                print("%s: no result for %s (build/run failed?) -- VOID" % (name, case))
                continue
            hit = res in ("FAILED", "CRASHED")
            print("%s: %s -> %s (%s)" % (name, case, "FIRED" if hit else "MISSED", res))
            fired += hit
    finally:
        open(SRC, "w").write(orig)
    out = run(build_dir)
    tail = [l for l in out.splitlines() if "Testing complete" in l]
    print("restored source: %s" % (tail[-1].split("] ", 1)[-1] if tail else "NO RESULT"))
    print("MUST-FIRE %d/%d" % (fired, len(MUTATIONS)))
    return 0 if fired == len(MUTATIONS) else 1


if __name__ == "__main__":
    sys.exit(main())
