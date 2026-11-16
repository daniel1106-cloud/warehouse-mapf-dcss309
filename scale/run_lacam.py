"""LaCAM (our Python implementation, scale/lacam.py) on the benchmark-map instances.

Two sets, the same instances as before, so the rows can be compared with PP directly:
  scale      the 60 instances of run_scale.py (random starts and goals, 50-300 robots)
  endpoints  the 60 instances of run_endpoints.py (goals in shelf aisles or open areas)
60 s limit per run, plans re-checked with run_scale.check_plan.

Outputs: scale/lacam_scale.csv, scale/lacam_endpoints.csv
Usage:   python scale/run_lacam.py scale        (or: endpoints; add --resume to continue)
"""
import csv
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
import fast_pp  # noqa: E402
import lacam  # noqa: E402
import run_endpoints  # noqa: E402
from maps import load_map  # noqa: E402
from run_scale import MAP, check_plan, lower_bound  # noqa: E402

FIELDS = ["set", "goal_area", "num_robots", "instance_id", "seed", "solver", "success", "status", "runtime_sec",
          "nodes", "sum_of_costs", "makespan", "lower_bound", "gap_upper_pct"]


def instances(which):
    grid = load_map(MAP)
    if which == "scale":
        for d in json.loads((HERE / "instances.json").read_text())["instances"]:
            yield "", d["id"], d["seed"], dict(grid=grid, starts=[tuple(p) for p in d["starts"]],
                                               goals=[tuple(p) for p in d["goals"]])
    else:
        for area, k, j, seed, inst in run_endpoints.make_instances():
            yield area, f"{area}-k{k}-{j:03d}", seed, inst


def main(which, limit=60):
    graph = fast_pp.Graph(load_map(MAP))
    target = HERE / f"lacam_{which}.csv"
    done = set()
    if "--resume" in sys.argv and target.exists():  # continue an interrupted run
        done = {r["instance_id"] for r in csv.DictReader(target.open(encoding="utf-8"))}
    with target.open("a" if done else "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if not done:
            w.writeheader()
        for area, iid, seed, inst in instances(which):
            if iid in done:
                continue
            lb = lower_bound(inst, graph)
            r = lacam.solve(inst, limit, seed, graph)
            row = dict(set=which, goal_area=area, num_robots=len(inst["starts"]), instance_id=iid, seed=seed,
                       solver="lacam", status=r["status"], runtime_sec=round(r["runtime"], 4), nodes=r["nodes"],
                       success=False, sum_of_costs="", makespan="", lower_bound=lb, gap_upper_pct="")
            if r["paths"] is not None:
                legal, v, s, cost = check_plan(inst, r["paths"])
                ok = legal and v == 0 and s == 0 and r["runtime"] <= limit
                row.update(success=ok, sum_of_costs=cost, makespan=max(map(len, r["paths"])) - 1)
                if ok:
                    row["gap_upper_pct"] = round(100 * (cost / lb - 1), 3)
                else:
                    row["status"] = "invalid"
            w.writerow(row); f.flush()
            print(f"{iid}: {row['status']} {row['runtime_sec']:.1f}s gap={row['gap_upper_pct']}", flush=True)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "scale")
