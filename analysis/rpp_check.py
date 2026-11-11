"""Revised prioritized planning (RPP) on the main experiment's 400 instances.

Our PP (as in mapf.py) lets a robot drive through the start cell of a robot that has not
been planned yet, although that robot is still standing there. Revised PP (Cap et al.,
2015) forbids this: each robot also stays off the start cells of all robots planned after
it. Cap et al. show that RPP always finds a plan when the layout is "well-formed" (every
start and goal is off the paths that connect the others). This script measures how much
that single rule changes success on our two layouts.

Methods (scale/fast_pp.py): pp, rpp (fixed order); pp_promote, rpp_promote (failed robot first).
Plans are re-checked with scale/run_scale.check_plan.
Outputs: analysis/rpp_check.csv, printed summary (including how many instances are well-formed)
Usage:   python analysis/rpp_check.py
"""
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scale"))
import fast_pp  # noqa: E402
from run_scale import check_plan  # noqa: E402

METHODS = ("pp", "rpp", "pp_promote", "rpp_promote")
FIELDS = ["layout", "num_robots", "instance_id", "solver", "success", "status", "runtime_sec", "attempts",
          "sum_of_costs"]


def well_formed(inst):
    """Cap et al.'s condition: between any two endpoints (starts and goals) there is a path
    that avoids all other endpoints. A BFS from each endpoint may end at, but not pass
    through, another endpoint."""
    graph = fast_pp.Graph(inst["grid"])
    ends = set(inst["starts"]) | set(inst["goals"])
    for a in ends:
        seen = {a}; queue = [a]; head = 0
        while head < len(queue):
            p = queue[head]; head += 1
            if p != a and p in ends:
                continue
            for q in graph.steps[p]:
                if q not in seen:
                    seen.add(q); queue.append(q)
        if not ends <= seen:
            return False
    return True


def main(limit=10):
    instances = json.loads((ROOT / "outputs" / "instances.json").read_text())
    rows = []
    with (ROOT / "analysis" / "rpp_check.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for d in instances:
            inst = dict(grid=d["grid"], starts=[tuple(p) for p in d["starts"]], goals=[tuple(p) for p in d["goals"]])
            for m in METHODS:
                r = fast_pp.solve(inst, limit, m, d["seed"], 32)
                row = dict(layout=d["id"].split("-")[0], num_robots=len(d["starts"]), instance_id=d["id"], solver=m,
                           status=r["status"], runtime_sec=round(r["runtime"], 5), attempts=r["attempts"],
                           success=False, sum_of_costs="")
                if r["paths"] is not None:
                    legal, v, s, cost = check_plan(inst, r["paths"])
                    row.update(success=legal and v == 0 and s == 0, sum_of_costs=cost)
                    if not row["success"]:
                        row["status"] = "invalid"
                w.writerow(row); rows.append(row)
    formed = defaultdict(int)
    for d in instances:
        inst = dict(grid=d["grid"], starts=[tuple(p) for p in d["starts"]], goals=[tuple(p) for p in d["goals"]])
        formed[d["id"].split("-")[0]] += well_formed(inst)
    print("well-formed instances: " + ", ".join(f"{k} {v}/200" for k, v in sorted(formed.items())))
    total = defaultdict(int)
    for r in rows:
        total[(r["layout"], r["solver"])] += r["success"]
    for layout in ("aisle", "warehouse"):
        print(layout + ": " + ", ".join(f"{m} {total[(layout, m)]}/200" for m in METHODS))


if __name__ == "__main__":
    main()
