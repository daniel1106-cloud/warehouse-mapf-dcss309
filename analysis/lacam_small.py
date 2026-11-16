"""LaCAM (scale/lacam.py) on the main experiment's 400 instances (12 x 20 map, 10 s limit).

Shows two things: whether every instance is solvable (any valid plan proves it), and how
LaCAM's cost compares with PP and with the proven optima from CBS. Two variants:
  lacam       a configuration reached again goes back on the search stack (default)
  lacam_skip  it is skipped, as in the AAAI pseudocode (long plans, but often found fast)

Outputs: analysis/lacam_small.csv, printed summary
Usage:   python analysis/lacam_small.py
"""
import csv
import json
import statistics
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scale"))
import lacam  # noqa: E402
from run_scale import check_plan  # noqa: E402


def main(limit=10):
    instances = json.loads((ROOT / "outputs" / "instances.json").read_text())
    optimum = {r["instance_id"]: int(float(r["cbs_optimum"]))
               for r in csv.DictReader((ROOT / "analysis" / "cbs_gap.csv").open()) if r["cbs_optimum"]}
    pp_cost = {r["instance_id"]: int(r["sum_of_costs"]) for r in csv.DictReader((ROOT / "outputs" / "results.csv").open())
               if r["solver"] == "pp" and r["success"] == "True"}
    rows = []
    with (ROOT / "analysis" / "lacam_small.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["layout", "num_robots", "instance_id", "solver", "success", "status",
                                          "runtime_sec", "sum_of_costs", "optimum", "pp_cost"])
        w.writeheader()
        for d in instances:
            inst = dict(grid=d["grid"], starts=[tuple(p) for p in d["starts"]], goals=[tuple(p) for p in d["goals"]])
            for solver in ("lacam", "lacam_skip"):
                r = lacam.solve(inst, limit, d["seed"], reinsert=solver == "lacam")
                row = dict(layout=d["id"].split("-")[0], num_robots=len(d["starts"]), instance_id=d["id"],
                           solver=solver, success=False, status=r["status"], runtime_sec=round(r["runtime"], 4),
                           sum_of_costs="", optimum=optimum.get(d["id"], ""), pp_cost=pp_cost.get(d["id"], ""))
                if r["paths"] is not None:
                    legal, v, s, cost = check_plan(inst, r["paths"])
                    row.update(success=legal and v == 0 and s == 0, sum_of_costs=cost)
                    if not row["success"]:
                        row["status"] = "invalid"
                w.writerow(row); rows.append(row)
    print(Counter((r["solver"], r["layout"], r["status"]) for r in rows))
    either = {r["instance_id"] for r in rows if r["success"]}
    print("instances with a valid plan from either variant:", len(either), "of", len(instances))
    rows = [r for r in rows if r["solver"] == "lacam"]
    ok = [r for r in rows if r["success"]]
    below = [r for r in ok if r["optimum"] != "" and r["sum_of_costs"] < r["optimum"]]
    print("plans below a proven optimum (would be a bug):", len(below))
    for layout in ("aisle", "warehouse"):
        g = [100 * (r["sum_of_costs"] / r["optimum"] - 1) for r in ok if r["layout"] == layout and r["optimum"] != ""]
        h = [100 * (r["sum_of_costs"] / r["pp_cost"] - 1) for r in ok if r["layout"] == layout and r["pp_cost"] != ""]
        print(f"{layout}: gap to optimum median {statistics.median(g):.1f}% max {max(g):.1f}% (n={len(g)}); "
              f"cost vs fixed-order PP median {statistics.median(h):+.1f}% (n={len(h)})")


if __name__ == "__main__":
    main()
