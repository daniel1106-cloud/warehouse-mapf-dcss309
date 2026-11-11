"""Scale experiment: prioritized planning on a standard warehouse benchmark map.

Map: warehouse-10-20-10-2-1 from the MovingAI MAPF benchmark (161 x 63, 5,699 free
cells), about 40 times larger than the main experiment's map. Starts and goals are
distinct random free cells, like the benchmark's "random" scenarios.

Methods: independent BFS (collision count only), fixed-order PP and failed-robot-first
PP, using scale/fast_pp.py (same algorithm as mapf.py, faster data structures).
The optimum cannot be computed at this size. Instead each PP cost is compared with a
lower bound on the optimum: the sum of every robot's own shortest-path length.
Because optimum >= lower bound, cost / lower bound - 1 is an upper bound on PP's gap.

Outputs: scale/results.csv, scale/instances.json, scale/scale.png
Usage:   python scale/run_scale.py [--quick]
         python scale/run_scale.py --map warehouse-10-20-10-2-2   (second map: two-cell aisles)
"""
import argparse
import csv
import json
import platform
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

MAP = "warehouse-10-20-10-2-1"
FIELDS = ["map", "num_robots", "instance_id", "seed", "solver", "success", "status", "runtime_sec",
          "attempts", "sum_of_costs", "lower_bound", "gap_upper_pct", "vertex_conflicts", "swap_conflicts"]


def check_plan(instance, paths):
    """Independent plan check in O(time x robots): legality, conflicts, cost.

    Same rules as the main experiment: a robot stays on its last cell after its path
    ends; vertex conflicts are counted per robot pair and time step, swap conflicts per
    robot pair and transition; cost = sum of final arrival times.
    """
    grid = instance["grid"]
    legal = True
    for p, s, g in zip(paths, instance["starts"], instance["goals"]):
        if p[0] != s or p[-1] != g:
            legal = False
        for a, b in zip(p, p[1:]):
            if abs(a[0] - b[0]) + abs(a[1] - b[1]) > 1:
                legal = False
        if any(grid[r][c] == "#" for r, c in p):
            legal = False
    T = max(map(len, paths))
    vertex = swap = 0
    for t in range(T):
        here = {}
        for i, p in enumerate(paths):
            here.setdefault(p[min(t, len(p) - 1)], []).append(i)
        vertex += sum(len(v) * (len(v) - 1) // 2 for v in here.values())
        if t:
            moves = {}
            for i, p in enumerate(paths):
                a, b = p[min(t - 1, len(p) - 1)], p[min(t, len(p) - 1)]
                if a != b:
                    moves[(a, b)] = moves.get((a, b), 0) + 1
            swap += sum(n * moves.get((b, a), 0) for (a, b), n in moves.items()) // 2
    cost = 0
    for p, g in zip(paths, instance["goals"]):
        t = len(p) - 1
        while t > 0 and p[t - 1] == g:
            t -= 1
        cost += t
    return legal, vertex, swap, cost


def lower_bound(instance, graph):
    return sum(graph.distances(g)[s] for s, g in zip(instance["starts"], instance["goals"]))


def run(sizes, per_size, limit, seed0, out):
    grid = load_map(MAP)
    free = mapf.cells(grid)
    graph = fast_pp.Graph(grid)
    rows, instances = [], []
    with (out / "results.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for k in sizes:
            for j in range(per_size):
                seed = seed0 + 1000 * k + j
                sel = random.Random(seed).sample(free, 2 * k)
                inst = dict(grid=grid, starts=sel[:k], goals=sel[k:])
                iid = f"{MAP}-k{k}-{j:03d}"
                instances.append(dict(id=iid, seed=seed, starts=inst["starts"], goals=inst["goals"]))
                lb = lower_bound(inst, graph)
                for solver in ("independent", "pp", "pp_promote"):
                    if solver == "independent":
                        t0 = time.perf_counter()
                        paths, status = mapf.independent(inst, t0 + limit)
                        res = dict(paths=paths, status=status, runtime=time.perf_counter() - t0, attempts=1)
                    else:
                        res = fast_pp.solve(inst, limit, solver, seed, 32)
                    row = dict(map=MAP, num_robots=k, instance_id=iid, seed=seed, solver=solver,
                               status=res["status"], runtime_sec=round(res["runtime"], 4),
                               attempts=res["attempts"], lower_bound=lb, success=False,
                               sum_of_costs="", gap_upper_pct="", vertex_conflicts="", swap_conflicts="")
                    if res["paths"] is not None:
                        legal, v, s, cost = check_plan(inst, res["paths"])
                        row.update(sum_of_costs=cost, vertex_conflicts=v, swap_conflicts=s)
                        ok = legal and v == 0 and s == 0 and res["runtime"] <= limit
                        row["success"] = ok
                        if ok:
                            row["gap_upper_pct"] = round(100 * (cost / lb - 1), 3)
                        elif res["runtime"] > limit:
                            row["status"] = "timeout"
                        elif not legal:
                            row["status"] = "invalid_path"
                        else:
                            row["status"] = "collision"
                    w.writerow(row); f.flush(); rows.append(row)
                print(f"k={k} #{j}: " + ", ".join(
                    f"{r['solver']} {r['status']} {r['runtime_sec']:.1f}s" for r in rows[-3:]), flush=True)
    (out / "instances.json").write_text(json.dumps(dict(map=MAP, instances=instances)))
    (out / "metadata.json").write_text(json.dumps(dict(
        map=MAP, sizes=sizes, instances_per_size=per_size, time_limit_sec=limit, base_seed=seed0,
        python=platform.python_version(), platform=platform.platform()), indent=2))
    return rows


MIN_SUCCESSES = 3  # robot counts with fewer successful runs are left out of the cost panel


def plot(rows, target):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    sizes = sorted({int(r["num_robots"]) for r in rows})
    style = {"pp": ("Fixed-order PP", "#2563eb", "o"), "pp_promote": ("PP, failed robot first", "#ea580c", "s")}
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.3))
    for solver, (label, color, marker) in style.items():
        sel = lambda k: [r for r in rows if r["solver"] == solver and int(r["num_robots"]) == k]
        ok = lambda r: str(r["success"]) == "True"
        rate = [100 * sum(map(ok, sel(k))) / len(sel(k)) for k in sizes]
        med_t = [statistics.median(float(r["runtime_sec"]) for r in sel(k)) for k in sizes]
        axes[0].plot(sizes, rate, marker=marker, color=color, label=label)
        axes[1].plot(sizes, med_t, marker=marker, color=color, label=label)
        if solver == "pp":  # the two PP variants have nearly identical costs; show one
            xs, med, mx = [], [], []
            for k in sizes:
                g = [float(r["gap_upper_pct"]) for r in sel(k) if ok(r)]
                if len(g) >= MIN_SUCCESSES:
                    xs.append(k); med.append(statistics.median(g)); mx.append(max(g))
            axes[2].plot(xs, med, marker=marker, color=color, label="median")
            axes[2].plot(xs, mx, marker="^", color=color, linestyle="--", alpha=.45, label="maximum")
    axes[0].set(title="Plans found", ylabel="Success rate (%)", ylim=(-5, 105))
    axes[1].set(title="Median planning time (Python; a timeout counts as 60 s)", ylabel="Seconds")
    axes[2].set(title="Fixed-order PP cost above a lower bound on the optimum", ylabel="Upper bound on gap (%)")
    for ax in axes:
        ax.set_xlabel("Robots"); ax.set_xticks(sizes); ax.grid(alpha=.2); ax.legend(fontsize=8)
    fig.suptitle(f"Prioritized planning on the {MAP} benchmark map, 10 instances per size")
    fig.tight_layout()
    fig.savefig(target, dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--plot-only", action="store_true")
    ap.add_argument("--map", help="another map in scale/maps; results go to scale/<map>/ (100, 200, 300 robots)")
    a = ap.parse_args()
    if a.map:
        MAP = a.map
        out = HERE / MAP; out.mkdir(exist_ok=True)
        rows = run([100, 200, 300], 10, 60, 20261111, out)
        plot(rows, out / "scale.png"); sys.exit()
    if a.plot_only:
        rows = list(csv.DictReader((HERE / "results.csv").open(encoding="utf-8")))
    elif a.quick:
        out = HERE / "quick"; out.mkdir(exist_ok=True)
        rows = run([20, 40], 2, 10, 20261111, out)
        plot(rows, out / "scale.png"); sys.exit()
    else:
        rows = run([50, 100, 150, 200, 250, 300], 10, 60, 20261111, HERE)
    plot(rows, HERE / "scale.png")
