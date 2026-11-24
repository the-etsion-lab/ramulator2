#!/usr/bin/env python3
"""Compare a Frontend statistic between a baseline run and a test run.

Usage:
  compare-stat.py <stat> <test-dir> -t <base-dir> [-b bench] [-n v1,v2,...] [-r resdir]

<test-dir> may contain "=N"; with -n each value is substituted in turn.

Besides the raw Frontend stats, the following derived stats are available:
  ipc_total            sum(post-warmup insts) / sum(post-warmup cycles) over cores
  total_insts          instructions retired in the stats window (after the
                       global warmup reset; LLC/CoWs counters cover the same window)
  llc_mpki             llc_total_misses per kilo-instruction
  cows_mpki            cows_misses (all paths) per kilo-instruction
  cows_llcmiss_mpki    cows_misses_on_llc_miss per kilo-instruction
  cows_evict_pki       cows_evicts per kilo-instruction
  llc_total_hits, llc_write_access_ratio,
  cows_misses_on_llc_hit_ratio, cows_misses_on_llc_miss_ratio

Runs whose 'out' file has no stats (e.g. killed by a signal) are reported as
INCOMPLETE with the reason, instead of being silently skipped.
"""

import argparse
import difflib
import os
import re
import sys
from pathlib import Path

import yaml

BASEDIR = "/scratch/yetsion/cows/tools/ramulator2/res"


class RunIncomplete(Exception):
    pass


def _ratio(num, den):
    return num / den if den else float("nan")


def load_ramulator_yaml(path: str) -> dict:
    """Parse the stats YAML printed at the end of a ramulator run.

    Raises RunIncomplete if the file is missing or contains no stats block.
    """
    file_path = Path(path)
    if not file_path.is_file():
        raise RunIncomplete(f"missing file {path}")
    lines = file_path.read_text().splitlines()

    start_idx = None
    end_idx = len(lines)
    for idx, line in enumerate(lines):
        if start_idx is None and line.startswith("Frontend:"):
            start_idx = idx
        elif start_idx is not None and "Command being timed" in line:
            end_idx = idx
            break

    if start_idx is None:
        m = re.search(r"Command terminated by signal (\d+)", "\n".join(lines))
        why = f"killed by signal {m.group(1)}" if m else "no stats block"
        raise RunIncomplete(f"{why} ({path})")

    # The stats block is a YAML document whose top-level keys start in column 0.
    # Non-YAML lines (e.g. '# ...' progress prints, '[Ramulator...]' log lines)
    # may be interleaved; drop lines that start with '[' (logger output).
    block = [l for l in lines[start_idx:end_idx] if not l.startswith("[")]
    ret = yaml.safe_load("\n".join(block))
    if not isinstance(ret, dict) or "Frontend" not in ret:
        raise RunIncomplete(f"unparsable stats block ({path})")

    add_derived_stats(ret["Frontend"])
    return ret


def add_derived_stats(fe: dict) -> None:
    # IPC as computed by the simulator (sum insts / sum cycles, post warmup).
    if "ipc_total" not in fe:
        cyc = sum(v for k, v in fe.items() if k.startswith("cycles_recorded_post_warmup_core_"))
        ins = sum(v for k, v in fe.items() if k.startswith("insts_retired_post_warmup_core_"))
        fe["ipc_total"] = _ratio(ins, cyc)

    # Instructions retired in the stats window: all counters (LLC, CoWs) are
    # reset when *all* cores finish warmup, and total_insts_retired_post_warmup
    # counts from that same reset until the end of simulation.
    total_insts = sum(v for k, v in fe.items()
                      if k.startswith("total_insts_retired_post_warmup_core_"))
    fe["total_insts"] = total_insts
    kinst = total_insts / 1000.0

    llc_acc = fe.get("llc_total_access", 0)
    llc_miss = fe.get("llc_total_misses", 0)
    fe["llc_total_hits"] = llc_acc - llc_miss
    fe["llc_write_access_ratio"] = _ratio(fe.get("llc_write_access", 0), llc_acc)
    fe["llc_mpki"] = _ratio(llc_miss, kinst)

    if "cows_accesses" in fe:
        fe["cows_misses_on_llc_hit_ratio"] = _ratio(fe.get("cows_misses_on_llc_hit", 0), fe["llc_total_hits"])
        fe["cows_misses_on_llc_miss_ratio"] = _ratio(fe.get("cows_misses_on_llc_miss", 0), llc_miss)
        fe["cows_mpki"] = _ratio(fe.get("cows_misses", 0), kinst)
        fe["cows_llcmiss_mpki"] = _ratio(fe.get("cows_misses_on_llc_miss", 0), kinst)
        fe["cows_evict_pki"] = _ratio(fe.get("cows_evicts", 0), kinst)


def get_stat(data: dict, statname: str, path: str):
    fe = data["Frontend"]
    if statname not in fe:
        close = difflib.get_close_matches(statname, fe.keys(), n=5)
        raise RunIncomplete(f"stat '{statname}' not found in {path}"
                       + (f"; did you mean: {', '.join(close)}" if close else ""))
    return fe[statname]


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Compare stats between baseline and test runs")
    parser.add_argument("stat_name", help="Stat name to compare (Frontend section, or a derived stat)")
    parser.add_argument("test_name", help="Test directory name (relative to the results dir)")
    parser.add_argument("-t", "--base", required=True,
                        help="Baseline directory name (e.g. tst-base-nocows-100M-8c-...). Required: "
                             "it must match the test's instruction count and core count.")
    parser.add_argument("-b", "--bench", help="Only this benchmark")
    parser.add_argument("-n", "--nvals", help="Values substituted for '=N' in test_name, comma-separated")
    parser.add_argument("-r", "--resdir", default=BASEDIR, help=f"Results directory (default {BASEDIR})")
    args = parser.parse_args(argv)

    statname = args.stat_name.strip()
    if not statname:
        print("Error: stat name cannot be empty.", file=sys.stderr)
        return 1

    resdir = Path(args.resdir)
    nvals = [v.strip() for v in args.nvals.split(",") if v.strip()] if args.nvals else [None]
    test_dirs = {n: resdir / (args.test_name.replace("=N", f"={n}") if n is not None else args.test_name)
                 for n in nvals}
    base_dir = resdir / args.base

    print(f" statname: {statname}\n base: {base_dir}\n test: {args.test_name}\n N={nvals}")

    for d in [base_dir, *test_dirs.values()]:
        if not d.is_dir():
            print(f"Error: no such run directory {d}", file=sys.stderr)
            return 1

    first_test = next(iter(test_dirs.values()))
    benchs = sorted(f.name for f in first_test.iterdir() if f.is_dir())
    if args.bench:
        benchs = [b for b in benchs if b == args.bench]

    errors = 0
    for b in benchs:
        if args.nvals:
            print(f"\nbenchmark {b}:")
        base_file = str(base_dir / b / "out")
        try:
            base_dict = load_ramulator_yaml(base_file)
            base_stat = get_stat(base_dict, statname, base_file)
        except RunIncomplete as e:
            print(f"{b:<30} BASE ERROR: {e}")
            errors += 1
            continue

        for n, tdir in test_dirs.items():
            label = n if args.nvals else b
            bench_file = str(tdir / b / "out")
            try:
                bench_dict = load_ramulator_yaml(bench_file)
                bench_stat = get_stat(bench_dict, statname, bench_file)
            except RunIncomplete as e:
                print(f"{label:<30} TEST ERROR: {e}")
                errors += 1
                continue

            bi = base_dict["Frontend"].get("num_expected_insts")
            ti = bench_dict["Frontend"].get("num_expected_insts")
            if bi != ti:
                print(f"{label:<30} WARNING: instruction count differs (base {bi}, test {ti})")

            ratio = bench_stat / base_stat if base_stat != 0 else float("inf")
            print(f"{label:<30} bench/base: {ratio:12.4f} (base: {base_stat}, bench: {bench_stat})")

    return 2 if errors else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
