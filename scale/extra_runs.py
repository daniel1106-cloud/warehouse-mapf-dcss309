"""Extra runs that tighten three earlier comparisons.

  density   Aisle width at equal crowding. The two-cell-aisle map has 9,776 free cells,
            the one-cell-aisle map 5,699, so the same robot count is less crowded on the
            first. Here both maps are run at the same robots per free cell: the one-cell map
            with 100, 200, 300 robots (the instances of run_scale.py) and the two-cell map with
            172, 343, 515 robots. Fixed-order PP, random endpoints, 10 instances per size,
            120 s limit on both maps (larger plans take longer in Python).
  oneway    One-way aisles with goals in the open areas: 20 more instances per size (200 and
            300 robots, instances 10-29 of run_endpoints.py), two-way and one-way PP, 60 s.
  cutoff    Early cut-off detection: 20 more random-endpoint instances per size (100, 150,
            200, 250 robots, instances 10-29 with run_scale.py's seed scheme), failed-robot-
            first with and without the cut-off test, 60 s.

Outputs: scale/extra_density.csv, scale/extra_oneway.csv, scale/extra_cutoff.csv
Usage:   python scale/extra_runs.py density|oneway|cutoff
"""
import csv
import random
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
import fast_pp  # noqa: E402
import mapf  # noqa: E402
import oneway  # noqa: E402
import run_endpoints  # noqa: E402
from maps import load_map  # noqa: E402
from run_scale import check_plan, lower_bound  # noqa: E402

FIELDS = ["experiment", "map", "setting", "num_robots", "instance_id", "seed", "solver", "success", "status",
          "runtime_sec", "attempts", "sum_of_costs", "lower_bound", "gap_upper_pct"]
MAP1, MAP2 = "warehouse-10-20-10-2-1", "warehouse-10-20-10-2-2"


def random_instance(map_name, grid, k, j, seed0=20261111):
    """Same seed scheme as run_scale.run: distinct random starts and goals."""
    seed = seed0 + 1000 * k + j
    sel = random.Random(seed).sample(mapf.cells(grid), 2 * k)
    return f"{map_name}-k{k}-{j:03d}", seed, dict(grid=grid, starts=sel[:k], goals=sel[k:])


def jobs(which):
    if which == "density":
        for map_name, sizes in ((MAP1, (100, 200, 300)), (MAP2, (172, 343, 515))):
            grid = load_map(map_name)
            for k in sizes:
                for j in range(10):
                    iid, seed, inst = random_instance(map_name, grid, k, j)
                    yield map_name, "random", iid, seed, inst, (("pp", None),), 120
    elif which == "oneway":
        graph = oneway.OneWayGraph(load_map(MAP1))
        for area, k, j, seed, inst in run_endpoints.make_instances(sizes=(200, 300), per_size=30):
            if area == "open_area" and j >= 10:
                yield MAP1, area, f"{area}-k{k}-{j:03d}", seed, inst, (("pp", None), ("pp_oneway", graph)), 60
    elif which == "cutoff":
        grid = load_map(MAP1)
        for k in (100, 150, 200, 250):
            for j in range(10, 30):
                iid, seed, inst = random_instance(MAP1, grid, k, j)
                yield MAP1, "random", iid, seed, inst, (("pp_promote", None), ("pp_promote_cutoff", None)), 60
    else:
        raise SystemExit("usage: python scale/extra_runs.py density|oneway|cutoff")


def main(which):
    graphs = {}
    with (HERE / f"extra_{which}.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for map_name, setting, iid, seed, inst, solvers, limit in jobs(which):
            if map_name not in graphs:
                graphs[map_name] = fast_pp.Graph(inst["grid"])
            lb = lower_bound(inst, graphs[map_name])
            for name, graph in solvers:
                method = "pp" if name == "pp_oneway" else name
                r = fast_pp.solve(inst, limit, method, seed, 32, graph=graph or graphs[map_name])
                row = dict(experiment=which, map=map_name, setting=setting, num_robots=len(inst["starts"]),
                           instance_id=iid, seed=seed, solver=name, status=r["status"],
                           runtime_sec=round(r["runtime"], 4), attempts=r["attempts"], lower_bound=lb,
                           success=False, sum_of_costs="", gap_upper_pct="")
                if r["paths"] is not None:
                    legal, v, s, cost = check_plan(inst, r["paths"])
                    if name == "pp_oneway":
                        legal = legal and all(a == b or oneway.allowed(a, b)
                                              for p in r["paths"] for a, b in zip(p, p[1:]))
                    ok = legal and v == 0 and s == 0 and r["runtime"] <= limit
                    row.update(success=ok, sum_of_costs=cost)
                    if ok:
                        row["gap_upper_pct"] = round(100 * (cost / lb - 1), 3)
                    else:
                        row["status"] = "timeout" if r["runtime"] > limit else "invalid"
                w.writerow(row); f.flush()
                print(f"{iid} {name}: {row['status']} {row['runtime_sec']:.1f}s", flush=True)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "")
