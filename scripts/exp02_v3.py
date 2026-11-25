#!/usr/bin/env python3
"""EXP-02 v3 sweep (WI-013 item 4, amended by D-33/D-34): writes configs, prints the run plan, reports pass/stop. Never launches runs.

usage: exp02_v3.py gen    [--table-csv EXP-01/<run>/summary-P2-100M.csv]  -> configs/exp02/v3-*.yaml (commit them)
       exp02_v3.py plan   <ratio> [--variant mean-random]                 -> run.sh commands (100M, P2 + that CoWs point)
       exp02_v3.py status <run-dir> [--variant mean-random]               -> per-ratio pass table and the next action
Design: 100M runs only (D-34; no 1G without professor approval), 10-interval CI over the 100M window (region_ci.py, same instruction
intervals as the P2 baseline). Ratios tried in order 64, 32, ..., 1; stop at the first ratio that passes on every benchmark
(CI lower bound of IPC_cows / IPC_P2 >= 0.98) plus one more (the next smaller ratio). CoWs assoc 8, carve-out lines, lookup 6 cycles,
table latency per benchmark = EXP-01 P2 non-hit read mean (variant p95). Placement random, pool = footprint (no sharing, headline,
PI ruling); sensitivities: pool footprint/4 (pool4), same_bank. Every touched page renamed and tracked (worst case). C4/C5 open (D-31).
"""
import csv, glob, os, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RATIOS = [64, 32, 16, 8, 4, 2, 1]
VARIANTS = {   # name: (table-latency column, placement, pool)
    "mean-random": ("lat_nonhit_mean", "random", "footprint"),
    "p95-random": ("lat_nonhit_p95", "random", "footprint"),
    "mean-random-pool4": ("lat_nonhit_mean", "random", "footprint4"),
    "mean-same_bank": ("lat_nonhit_mean", "same_bank", "footprint"),
}
BENCHS = ["data-analytics", "data-caching", "data-serving", "graph-analytics", "in-memory-analytics", "media-streaming",
          "web-search", "web-serving"]


def opt(k, d):
    return sys.argv[sys.argv.index(k) + 1] if k in sys.argv else d


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    var = opt("--variant", "mean-random")
    if cmd == "gen":
        tcsv = opt("--table-csv", "EXP-01/20261124-2314-b651ed4/summary-P2-100M.csv")
        for f in glob.glob(os.path.join(ROOT, "configs/exp02/v3-*.yaml")):
            os.remove(f)
        for r in RATIOS:
            for name, (col, pl, pool) in VARIANTS.items():
                subprocess.run([sys.executable, os.path.join(ROOT, "scripts/exp02_config.py"), "--name", f"v3-r{r}-{name}",
                                "--ratio", str(r), "--assoc", "8", "--table-lat", "perbench", "--table-lat-run", tcsv,
                                "--table-lat-stat", col, "--lookup-lat", "6", "--carveout", "lines", "--placement", pl,
                                "--pool", pool], check=True, stdout=subprocess.DEVNULL)
    elif cmd == "plan":
        r = int(sys.argv[2])
        print("# EXP-02 v3 - only after the PI confirms the professor's approval. 100M only (D-34). C4/C5 open (D-31).")
        print('# R=$(results/EXP-01/newrun.sh EXP-02 "EXP-02 v3")   (once; P2 baseline once per run dir)')
        print("results/EXP-01/run.sh $R 100M P2 &")
        print(f"results/EXP-01/run.sh $R 100M v3-r{r}-{var} &")
        print("wait")
    elif cmd == "status":
        rd = sys.argv[2]
        ci = os.path.join(ROOT, "scripts/region_ci.py")
        first_pass = None
        print("ratio,benchmarks_done,all_pass,worst_ci_low")
        for r in RATIOS:
            d = os.path.join(rd, f"v3-r{r}-{var}-100M")
            if not os.path.isdir(d):
                continue
            lows, passes, n = [], [], 0
            for b in BENCHS:
                if not os.path.isdir(os.path.join(d, b)):
                    continue
                out = subprocess.run([sys.executable, ci, os.path.join(d, b), os.path.join(rd, "P2-100M", b)],
                                     capture_output=True, text=True).stdout.strip().split(",")
                if len(out) < 13:
                    continue
                n += 1; lows.append(float(out[9])); passes.append(out[11] == "True")
            allp = n == len(BENCHS) and all(passes)
            print(f"{r},{n},{allp},{min(lows) if lows else ''}")
            if allp and first_pass is None:
                first_pass = r
        if first_pass is None:
            print("# next: the next smaller ratio (no ratio passes yet)")
        else:
            nxt = [x for x in RATIOS if x < first_pass][:1]
            print(f"# first passing ratio: {first_pass}; run one more: {nxt[0] if nxt else 'none'}; then stop (D-34)")
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
