"""How much cost does prioritized planning lose, beyond the range of Joint A*?

Joint A* proves optimal costs only up to 4 (aisle) or 6 (warehouse) robots. This script
uses the independent reference CBS (verification/reference_cbs.py) on the same
instances of the main experiment, with the same 10-second limit, and compares the
costs of fixed-order PP and failed-robot-first PP with the CBS optimum.

Outputs: analysis/cbs_gap.csv (one row per instance) and analysis/cbs_gap.png.
Usage:   python analysis/cbs_gap.py [outputs_folder] [time_limit]
"""
import csv
import json
import statistics
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "verification"))
from check_results import reference_optimum  # noqa: E402


def main(out=ROOT / "outputs", limit=10.0):
    out = Path(out)
    instances = json.loads((out / "instances.json").read_text())
    rows = {(r["instance_id"], r["solver"]): r
            for r in csv.DictReader((out / "results.csv").open(encoding="utf-8"))}
    records = []
    for layout in ("aisle", "warehouse"):
        for k in range(2, 21, 2):
            group = [d for d in instances if d["id"].startswith(f"{layout}-k{k}-")]
            solved = 0
            for d in group:
                t0 = time.perf_counter()
                opt = reference_optimum(d, limit)
                sec = time.perf_counter() - t0
                rec = dict(layout=layout, num_robots=k, instance_id=d["id"],
                           cbs_optimum="" if opt is None else opt, cbs_seconds=round(sec, 4),
                           joint_astar_optimum=rows[(d["id"], "joint_astar")]["sum_of_costs"]
                           if rows[(d["id"], "joint_astar")]["status"] == "optimal" else "")
                for m in ("pp", "pp_promote"):
                    r = rows[(d["id"], m)]
                    ok = r["success"] == "True"
                    rec[f"{m}_cost"] = r["sum_of_costs"] if ok else ""
                    rec[f"{m}_gap_pct"] = (round(100 * (int(r["sum_of_costs"]) / opt - 1), 3)
                                           if ok and opt else "")
                records.append(rec)
                solved += opt is not None
            print(f"{layout} k={k}: CBS solved {solved}/{len(group)}", flush=True)
            if solved == 0:
                break
    path = Path(__file__).resolve().parent
    with (path / "cbs_gap.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(records[0].keys()))
        w.writeheader()
        w.writerows(records)
    plot(records, path / "cbs_gap.png")


MIN_SAMPLES = 5  # robot counts with fewer proven optima are not plotted


def plot(records, target):
    """Median and maximum gap of fixed-order PP; only sizes with >= MIN_SAMPLES optima."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.2), sharey=True)
    for ax, layout in zip(axes, ("aisle", "warehouse")):
        xs, med, mx, ticks = [], [], [], []
        for k in sorted({int(r["num_robots"]) for r in records if r["layout"] == layout}):
            group = [r for r in records if r["layout"] == layout and int(r["num_robots"]) == k]
            proven = [r for r in group if r["cbs_optimum"] != ""]
            gaps = [float(r["pp_gap_pct"]) for r in proven if r["pp_gap_pct"] != ""]
            if len(proven) < MIN_SAMPLES or not gaps:
                continue
            xs.append(k); med.append(statistics.median(gaps)); mx.append(max(gaps))
            ticks.append(f"{k}\n{len(proven)}/{len(group)} proven")
        ax.plot(xs, med, marker="o", color="#2563eb", label="median")
        ax.plot(xs, mx, marker="^", linestyle="--", color="#2563eb", alpha=.45, label="maximum")
        ax.set_xticks(xs)
        ax.set_xticklabels(ticks, fontsize=8)
        ax.set(title=layout, xlabel="Robots", ylabel="Fixed-order PP cost above optimum (%)")
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    fig.suptitle("Cost lost by prioritized planning, measured against CBS optima (10 s limit)")
    fig.tight_layout()
    fig.savefig(target, dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    a = sys.argv
    if len(a) > 1 and a[1] == "--plot-only":
        here = Path(__file__).resolve().parent
        plot(list(csv.DictReader((here / "cbs_gap.csv").open(encoding="utf-8"))), here / "cbs_gap.png")
    else:
        main(a[1] if len(a) > 1 else ROOT / "outputs", float(a[2]) if len(a) > 2 else 10.0)
