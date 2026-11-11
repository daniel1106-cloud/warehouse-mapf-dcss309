"""Detect failure early, then reorder: does re-prioritization work at scale?

On the benchmark map, failed-robot-first PP rescued only 2 of 18 failures: most
failures were timeouts, and the failed attempt used up the whole 60 s budget. Here the
same 60 instances (scale/instances.json) are planned with a cheap extra test before
each robot's search: if the robot is cut off by parked earlier robots, it is reported
at once ("cut_off") instead of after a long search (fast_pp.cut_off).

  pp_cutoff          fixed order + cut-off test (shows how often the test fires)
  pp_promote_cutoff  failed robot first + cut-off test

Compared with pp and pp_promote in scale/results.csv.
Outputs: scale/cutoff_results.csv
Usage:   python scale/run_cutoff.py
"""
import csv
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
import fast_pp  # noqa: E402
from maps import load_map  # noqa: E402
from run_scale import MAP, check_plan, lower_bound  # noqa: E402

FIELDS = ["num_robots", "instance_id", "seed", "solver", "success", "status", "runtime_sec", "attempts",
          "sum_of_costs", "lower_bound", "gap_upper_pct"]


def main(limit=60):
    grid = load_map(MAP)
    graph = fast_pp.Graph(grid)
    data = json.loads((HERE / "instances.json").read_text())
    assert data["map"] == MAP
    with (HERE / "cutoff_results.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for d in data["instances"]:
            inst = dict(grid=grid, starts=[tuple(p) for p in d["starts"]], goals=[tuple(p) for p in d["goals"]])
            k = len(inst["starts"])
            lb = lower_bound(inst, graph)
            out = []
            for solver in ("pp_cutoff", "pp_promote_cutoff"):
                res = fast_pp.solve(inst, limit, solver, d["seed"], 32)
                row = dict(num_robots=k, instance_id=d["id"], seed=d["seed"], solver=solver, status=res["status"],
                           runtime_sec=round(res["runtime"], 4), attempts=res["attempts"], lower_bound=lb,
                           success=False, sum_of_costs="", gap_upper_pct="")
                if res["paths"] is not None:
                    legal, v, s, cost = check_plan(inst, res["paths"])
                    ok = legal and v == 0 and s == 0 and res["runtime"] <= limit
                    row.update(success=ok, sum_of_costs=cost)
                    if ok:
                        row["gap_upper_pct"] = round(100 * (cost / lb - 1), 3)
                    else:
                        row["status"] = "timeout" if res["runtime"] > limit else "invalid"
                w.writerow(row); f.flush(); out.append(row)
            print(d["id"] + ": " + ", ".join(f"{r['solver']} {r['status']} {r['runtime_sec']:.1f}s a={r['attempts']}"
                                             for r in out), flush=True)


if __name__ == "__main__":
    main()
