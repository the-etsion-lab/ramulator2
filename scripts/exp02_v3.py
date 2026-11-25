#!/usr/bin/env python3
"""EXP-02 v3 sweep (WI-013 item 4): writes the configs and prints the run plan. It never launches runs itself.

usage: exp02_v3.py gen   [--table-csv EXP-01/<run>/summary-P2-100M.csv]   -> configs/exp02/v3-*.yaml (commit them)
       exp02_v3.py plan  [--regions R0-100M,R100M-300M]                    -> shell commands (newrun.sh + run.sh) to stdout
Design (D-31/D-32, PI 2026-11-25): ratios {1,2,4,8,16,32,64}, CoWs assoc 8, carve-out lines, lookup 6 cycles,
table latency per benchmark = EXP-01 P2 mean non-hit read (variant: p95), placement random (source pool = 1G footprint / 4).
Sensitivity variant same_bank is generated too; the H-D policy variant needs the Q-03 code and is not part of this branch.
Each CoWs point is compared with the P2 baseline run on the same region (scripts/region_ci.py). Results are labelled
"C4/C5 open (D-31)".
"""
import os, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RATIOS = [1, 2, 4, 8, 16, 32, 64]
VARIANTS = [("mean", "lat_nonhit_mean", "random"), ("p95", "lat_nonhit_p95", "random"), ("mean", "lat_nonhit_mean", "same_bank")]


def names():
    for r in RATIOS:
        for tag, col, pl in VARIANTS:
            yield f"v3-r{r}-{tag}-{pl}", r, col, pl


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "plan"
    opt = lambda k, d: sys.argv[sys.argv.index(k) + 1] if k in sys.argv else d
    if cmd == "gen":
        csv = opt("--table-csv", "EXP-01/20261124-2314-b651ed4/summary-P2-100M.csv")
        for name, r, col, pl in names():
            subprocess.run([sys.executable, os.path.join(ROOT, "scripts/exp02_config.py"), "--name", name, "--ratio", str(r),
                            "--assoc", "8", "--table-lat", "perbench", "--table-lat-run", csv, "--table-lat-stat", col,
                            "--lookup-lat", "6", "--carveout", "lines", "--placement", pl, "--pool", "footprint4"], check=True)
    elif cmd == "plan":
        regions = opt("--regions", "R0-100M,R100M-300M").split(",")
        print("# EXP-02 v3 plan - DO NOT RUN before the PI confirms the professor's approval. C4/C5 open (D-31).")
        print('R=$(results/EXP-01/newrun.sh EXP-02 "EXP-02 v3 sweep")')
        for reg in regions:
            print(f"results/EXP-01/run.sh $R {reg} P2 &")
            for name, *_ in names():
                print(f"results/EXP-01/run.sh $R {reg} {name} &")
        print("wait")
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
