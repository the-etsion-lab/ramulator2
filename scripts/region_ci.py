#!/usr/bin/env python3
"""WI-013 region analysis: per-region IPC with a 95% CI from 10 intervals, and the paired IPC ratio vs a baseline run.

usage: region_ci.py <run-bench-dir> [<baseline-bench-dir>] [--intervals 10] [--threshold 0.98]
Each dir is one benchmark run of results/EXP-01/run.sh (config + coreN_ipc_stats + out).
Method:
  - The measured window of a run is instructions [warmup, num_expected_insts) of every core (warmup = warmup_insts, or
    warmup_percent x num_expected_insts if warmup_insts is 0), i.e. the region [a, b) of run.sh's R<a>-<b>.
  - coreN_ipc_stats holds one (cycle, IPC) sample per ~1M retired instructions; sample k covers instructions (k, k+1] x 1M.
    The samples of the window are split into 10 equal groups (fewer, >= 5, if the window has < 10 samples); interval IPC of a core = sum(insts) / sum(cycles); interval IPC of
    the run = mean over cores (the per-core-mean IPC of EXP-01).
  - IPC: mean over intervals and 95% CI half-width t(0.975, n-1) x sd / sqrt(n).
  - Ratio vs baseline (same benchmark, same region, same instruction intervals): per-interval ratio, mean, 95% CI.
    PASS iff the CI lower bound >= threshold (0.98 = within 2%).
  - A run whose cores were zeroed by the global warmup reset (TRUNC, see NOTES) is reported as TRUNC and never passes.
Output: one CSV line on stdout (header with --header).
"""
import math, os, re, statistics as S, sys

T975 = {4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228, 11: 2.201, 12: 2.179, 14: 2.145, 19: 2.093}


def cfg(d):
    c = {}
    for line in open(os.path.join(d, "config")):
        m = re.match(r"^\s+(num_expected_insts|warmup_insts|warmup_percent|skip_insts):\s*(\d+)", line)
        if m:
            c[m.group(1)] = int(m.group(2))
    w = c.get("warmup_insts", 0) or c["warmup_percent"] * c["num_expected_insts"] // 100
    return c["num_expected_insts"], w, c.get("skip_insts", 0)


def truncated(d):
    t = open(os.path.join(d, "out"), errors="replace").read()
    z = re.findall(r"^\s*cycles_recorded_post_warmup_core_\d+:\s*0\s*$", t, re.M)
    return len(z) > 0 or "MemorySystem:" not in t


def interval_ipc(d, n_iv):
    N, W, _ = cfg(d)
    per_core = []
    c = 0
    while os.path.exists(os.path.join(d, f"core{c}_ipc_stats")):
        smp = [tuple(map(float, l.split()[:2])) for l in open(os.path.join(d, f"core{c}_ipc_stats"))
               if l.strip() and not l.startswith("#")]
        rec, prev = [], 0.0
        for cyc, ipc in smp:
            rec.append((ipc * (cyc - prev), cyc - prev)); prev = cyc
        lo, hi = math.ceil(W / 1e6), int(N // 1e6)
        win = rec[lo:hi]
        n_iv = min(n_iv, len(win))   # short windows (smoke tests): one 1M-instruction sample per interval
        if n_iv < 5:
            raise SystemExit(f"{d}: core {c} has {len(win)} 1M-instruction samples in the window; at least 5 are needed")
        g = len(win) // n_iv
        per_core.append([sum(i for i, _ in win[k*g:(k+1)*g]) / sum(cy for _, cy in win[k*g:(k+1)*g]) for k in range(n_iv)])
        c += 1
    n = min(len(core) for core in per_core)
    return [S.mean(core[k] for core in per_core) for k in range(n)]


def ci(v):
    m = S.mean(v); h = T975.get(len(v) - 1, 1.96) * S.stdev(v) / math.sqrt(len(v))
    return m, h


def main():
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    opt = lambda k, dflt: type(dflt)(sys.argv[sys.argv.index(k) + 1]) if k in sys.argv else dflt
    n_iv, thr = opt("--intervals", 10), opt("--threshold", 0.98)
    run, base = a[0], (a[1] if len(a) > 1 else None)
    N, W, skip = cfg(run)
    cols = ["run", "region_start", "region_end", "trunc", "n_intervals", "ipc_mean", "ipc_ci95", "base_ipc_mean", "ratio_mean", "ratio_ci_low",
            "ratio_ci_high", "pass"]
    if "--header" in sys.argv:
        print(",".join(cols))
    r = {"run": run, "region_start": skip + W, "region_end": skip + N, "trunc": bool(truncated(run) or (base and truncated(base)))}
    iv = interval_ipc(run, n_iv)
    r["ipc_mean"], r["ipc_ci95"] = ci(iv)
    r["n_intervals"] = len(iv)
    if base:
        if cfg(base) != (N, W, skip):
            raise SystemExit(f"baseline region {cfg(base)} != run region {(N, W, skip)}")
        bv = interval_ipc(base, n_iv)
        n = min(len(iv), len(bv))
        m, h = ci([x / y for x, y in zip(iv[:n], bv[:n])])
        r.update(base_ipc_mean=S.mean(bv), ratio_mean=m, ratio_ci_low=m - h, ratio_ci_high=m + h,
                 **{"pass": (not r["trunc"]) and (m - h >= thr)})
    f = lambda v: f"{v:.5g}" if isinstance(v, float) else str(v)
    print(",".join(f(r.get(k, "")) for k in cols))


if __name__ == "__main__":
    main()
