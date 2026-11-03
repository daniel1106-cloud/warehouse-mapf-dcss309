"""Side-by-side animation for the presentation.

Left:  every robot follows its own shortest path, ignoring the others (uncoordinated BFS).
Right: prioritized planning (PP) on the same instance.
Both use the paths saved by the main experiment (outputs/plans.jsonl), so the GIF shows
exactly what was measured. Conflicts are found with the project's validator.

Usage: python presentation/side_by_side.py [instance_id]
Outputs: presentation/side_by_side.gif and presentation/side_by_side_frame.png
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from mapf import validate, at  # noqa: E402


def load(instance_id):
    inst = next(d for d in json.loads((ROOT / "outputs/instances.json").read_text()) if d["id"] == instance_id)
    inst["starts"] = [tuple(p) for p in inst["starts"]]
    inst["goals"] = [tuple(p) for p in inst["goals"]]
    paths = {}
    for line in (ROOT / "outputs/plans.jsonl").open(encoding="utf-8"):
        r = json.loads(line)
        if r["instance_id"] == instance_id and r["solver"] in ("independent", "pp"):
            paths[r["solver"]] = [[tuple(c) for c in p] for p in r["paths"]]
    return inst, paths["independent"], paths["pp"]


def main(instance_id="warehouse-k10-010"):
    inst, left, right = load(instance_id)
    rep_l, rep_r = validate(inst, left), validate(inst, right)
    assert rep_r["valid"], "PP plan must be conflict-free"
    grid = inst["grid"]
    shade = [[0.25 if c == "#" else 0.85 if c == "S" else 1.0 for c in row] for row in grid]
    colors = plt.get_cmap("tab10")
    T = max(max(map(len, left)), max(map(len, right))) + 3
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.6))

    def draw(ax, paths, report, t, title):
        ax.clear()
        ax.imshow(shade, cmap="gray", vmin=0, vmax=1)
        for i, (p, g) in enumerate(zip(paths, inst["goals"])):
            r, c = at(p, t)
            ax.scatter(g[1], g[0], marker="x", s=60, color=colors(i % 10), linewidths=2)
            ax.scatter(c, r, s=170, color=colors(i % 10), edgecolor="black", zorder=3)
            ax.text(c, r, str(i), ha="center", va="center", fontsize=7, color="white", zorder=4)
        now = [e for e in report["events"] if e[0] == t]
        so_far = sum(1 for e in report["events"] if e[0] <= t)
        for e in now:
            for rr, cc in e[3:]:
                ax.scatter(cc, rr, s=420, facecolor="none", edgecolor="red", linewidth=3, zorder=5)
        ax.set_title(f"{title}\nconflicts so far: {so_far}", fontsize=10,
                     color="red" if so_far else "black")
        ax.set_xticks([]); ax.set_yticks([])

    def frame(t):
        draw(axes[0], left, rep_l, t, "No coordination (each robot's own shortest path)")
        draw(axes[1], right, rep_r, t, "Prioritized planning (one robot at a time)")
        fig.suptitle(f"Same warehouse, same {len(left)} robots  |  t = {t}", fontsize=12)
        fig.tight_layout()

    anim = FuncAnimation(fig, frame, frames=T, interval=350)
    out = Path(__file__).resolve().parent
    anim.save(out / "side_by_side.gif", writer=PillowWriter(fps=3))
    # A still frame for slides: the time step with the most simultaneous conflicts on the left
    # (the later one if tied, so the running total is larger).
    counts = {}
    for e in rep_l["events"]:
        counts[e[0]] = counts.get(e[0], 0) + 1
    first = max(counts, key=lambda t: (counts[t], t)) if counts else 0
    frame(first)
    fig.savefig(out / "side_by_side_frame.png", dpi=160)
    plt.close(fig)
    print(f"{instance_id}: left conflicts {len(rep_l['events'])}, right valid {rep_r['valid']}, frames {T}, still at t={first}")


if __name__ == "__main__":
    main(*sys.argv[1:2])
