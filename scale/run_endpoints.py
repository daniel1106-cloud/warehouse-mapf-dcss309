"""Does it matter where robots stop? Benchmark-map version of aisle vs. warehouse.

Same map as run_scale.py. Starts are random free cells anywhere. Only the goals,
where robots stop and stay, are restricted:
  shelf_area: goals in the shelf area (columns 26-134), whose aisles are one cell wide
  open_area:  goals in the open areas on the left and right (columns 1-25 and 135-159)
A robot parked in a one-cell aisle can block it; one parked in an open area usually
cannot. Trip lengths differ between the two settings, so the lower bound per robot
is reported too.

Outputs: scale/endpoints_results.csv, scale/endpoints.png
Usage:   python scale/run_endpoints.py
"""
import csv
import random
import statistics
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
import mapf  # noqa: E402
import fast_pp  # noqa: E402
from maps import load_map  # noqa: E402
from run_scale import MAP, check_plan, lower_bound  # noqa: E402

SHELF_COLS = (26, 134)  # first and last column containing shelves
FIELDS = ["goal_area", "num_robots", "instance_id", "seed", "solver", "success", "status", "runtime_sec",
          "attempts", "sum_of_costs", "lower_bound", "gap_upper_pct"]


def run(sizes=(100, 200, 300), per_size=10, limit=60, seed0=20261112):
    grid = load_map(MAP)
    free = mapf.cells(grid)
    graph = fast_pp.Graph(grid)
    areas = {
        "shelf_area": [p for p in free if SHELF_COLS[0] <= p[1] <= SHELF_COLS[1]],
        "open_area": [p for p in free if not SHELF_COLS[0] <= p[1] <= SHELF_COLS[1]],
    }
    rows = []
    with (HERE / "endpoints_results.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for k in sizes:
            for j in range(per_size):
                for a, (area, pool) in enumerate(areas.items()):
                    seed = seed0 + 1000 * k + 100 * a + j
                    rng = random.Random(seed)
                    goals = rng.sample(pool, k)
                    taken = set(goals)
                    starts = rng.sample([p for p in free if p not in taken], k)
                    inst = dict(grid=grid, starts=starts, goals=goals)
                    lb = lower_bound(inst, graph)
                    for solver in ("pp", "pp_promote"):
                        res = fast_pp.solve(inst, limit, solver, seed, 32)
                        row = dict(goal_area=area, num_robots=k, instance_id=f"{area}-k{k}-{j:03d}", seed=seed,
                                   solver=solver, status=res["status"], runtime_sec=round(res["runtime"], 4),
                                   attempts=res["attempts"], lower_bound=lb, success=False,
                                   sum_of_costs="", gap_upper_pct="")
                        if res["paths"] is not None:
                            legal, v, s, cost = check_plan(inst, res["paths"])
                            ok = legal and v == 0 and s == 0 and res["runtime"] <= limit
                            row.update(success=ok, sum_of_costs=cost)
                            if ok:
                                row["gap_upper_pct"] = round(100 * (cost / lb - 1), 3)
                            else:
                                row["status"] = "timeout" if res["runtime"] > limit else "invalid"
                        w.writerow(row); f.flush(); rows.append(row)
                    print(f"k={k} #{j} {area}: " + ", ".join(
                        f"{r['solver']} {r['status']} {r['runtime_sec']:.1f}s" for r in rows[-2:]), flush=True)
    return rows


def plot(rows, target):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    sizes = sorted({int(r["num_robots"]) for r in rows})
    style = {"shelf_area": ("Goals in shelf aisles", "#dc2626", "o"),
             "open_area": ("Goals in open areas", "#16a34a", "s")}
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    for area, (label, color, marker) in style.items():
        for solver, ls in (("pp", "-"), ("pp_promote", "--")):
            g = lambda k: [r for r in rows if r["goal_area"] == area and r["solver"] == solver and int(r["num_robots"]) == k]
            rate = [100 * sum(str(r["success"]) == "True" for r in g(k)) / len(g(k)) for k in sizes]
            name = "fixed order" if solver == "pp" else "failed robot first"
            axes[0].plot(sizes, rate, marker=marker, color=color, linestyle=ls, label=f"{label}, {name}")
        g = lambda k: [r for r in rows if r["goal_area"] == area and r["solver"] == "pp" and int(r["num_robots"]) == k]
        med = [statistics.median(float(r["runtime_sec"]) for r in g(k)) for k in sizes]
        axes[1].plot(sizes, med, marker=marker, color=color, label=f"{label}, fixed order")
    axes[0].set(title="Plans found", ylabel="Success rate (%)", ylim=(-5, 105))
    axes[1].set(title="Median planning time (Python; a timeout counts as 60 s)", ylabel="Seconds")
    for ax in axes:
        ax.set_xlabel("Robots"); ax.set_xticks(sizes); ax.grid(alpha=.2); ax.legend(fontsize=8)
    fig.suptitle(f"Where robots stop: {MAP}, starts random, 10 instances per point")
    fig.tight_layout()
    fig.savefig(target, dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--plot-only":
        rows = list(csv.DictReader((HERE / "endpoints_results.csv").open(encoding="utf-8")))
    else:
        rows = run()
    plot(rows, HERE / "endpoints.png")
