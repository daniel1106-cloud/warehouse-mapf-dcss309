"""How far can an optimal solver get beyond Joint A*?

Runs the independent reference CBS on the experiment's own instances, robot count by
robot count, with a per-instance time limit. For every instance CBS solves, the
fixed-order PP cost from results.csv is compared with the CBS optimum.

Usage:  python cbs_reach.py ../outputs [instances_per_size] [time_limit]
"""
import csv
import json
import statistics
import sys
import time
from pathlib import Path

from check_results import reference_optimum


def main(out, per_size=5, limit=10):
    out = Path(out)
    instances = json.loads((out / "instances.json").read_text())
    pp = {r["instance_id"]: r for r in csv.DictReader((out / "results.csv").open(encoding="utf-8"))
          if r["solver"] == "pp"}
    print("layout,robots,cbs_solved,tried,median_cbs_seconds,pp_success_and_paired,median_gap_pct,max_gap_pct")
    for layout in ("aisle", "warehouse"):
        for k in range(2, 21, 2):
            group = [d for d in instances if d["id"].startswith(f"{layout}-k{k}-")][:per_size]
            solved, times, gaps = 0, [], []
            for d in group:
                t0 = time.perf_counter()
                opt = reference_optimum(d, limit)
                times.append(time.perf_counter() - t0)
                if opt is None:
                    continue
                solved += 1
                r = pp[d["id"]]
                if r["success"] == "True" and opt > 0:
                    gaps.append(100 * (int(r["sum_of_costs"]) / opt - 1))
            g_med = f"{statistics.median(gaps):.2f}" if gaps else ""
            g_max = f"{max(gaps):.2f}" if gaps else ""
            print(f"{layout},{k},{solved},{len(group)},{statistics.median(times):.3f},{len(gaps)},{g_med},{g_max}", flush=True)
            if solved == 0:
                break


if __name__ == "__main__":
    a = sys.argv
    main(a[1] if len(a) > 1 else "../outputs", int(a[2]) if len(a) > 2 else 5, float(a[3]) if len(a) > 3 else 10)
