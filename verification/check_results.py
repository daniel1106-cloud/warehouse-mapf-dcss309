"""Independent cross-check of an experiment output folder.

This script does NOT import the project's mapf.py. It re-checks saved results with
code written separately:

A. Its own validator re-checks every returned path in plans.jsonl (legal moves,
   no shelves, correct start/goal, vertex/swap conflicts including parked robots,
   sum of final arrival times) and compares with results.csv.
B. No prioritized-planning cost may be below the proven Joint A* optimum.
C. Every proven Joint A* optimum is recomputed with an independently written
   CBS solver (reference_cbs.py) and compared.

Usage:  python check_results.py ../outputs
"""
import csv
import collections
import json
import sys
from pathlib import Path

import reference_cbs as ref


def my_validate(instance, paths):
    """Return (legal, vertex_conflicts, swap_conflicts, sum_of_costs)."""
    grid = instance["grid"]
    H, W = len(grid), len(grid[0])
    starts = [tuple(x) for x in instance["starts"]]
    goals = [tuple(x) for x in instance["goals"]]
    P = [[tuple(c) for c in p] for p in paths]
    legal = True
    for i, p in enumerate(P):
        if p[0] != starts[i] or p[-1] != goals[i]:
            legal = False
        for (r, c) in p:
            if not (0 <= r < H and 0 <= c < W) or grid[r][c] == "#":
                legal = False
        for a, b in zip(p, p[1:]):
            if abs(a[0] - b[0]) + abs(a[1] - b[1]) > 1:
                legal = False
    T = max(len(p) for p in P)

    def at(p, t):  # a robot stays on its last cell after its path ends
        return p[t] if t < len(p) else p[-1]

    vertex = swap = 0
    for t in range(T):
        for i in range(len(P)):
            for j in range(i + 1, len(P)):
                if at(P[i], t) == at(P[j], t):
                    vertex += 1
                if (t + 1 < T and at(P[i], t) != at(P[i], t + 1)
                        and at(P[i], t) == at(P[j], t + 1) and at(P[j], t) == at(P[i], t + 1)):
                    swap += 1
    cost = 0
    for p, g in zip(P, goals):
        t = len(p) - 1
        while t > 0 and p[t - 1] == g:  # time of the final arrival
            t -= 1
        cost += t
    return legal, vertex, swap, cost


class _Grid:
    """Adapter: the reference solver expects cell ids and a neighbour table."""

    def __init__(self, grid):
        self.H, self.W = len(grid), len(grid[0])
        self.nbrs = {}
        for y in range(self.H):
            for x in range(self.W):
                self.nbrs[y * self.W + x] = [ny * self.W + nx
                                             for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
                                             for nx, ny in [(x + dx, y + dy)]
                                             if 0 <= nx < self.W and 0 <= ny < self.H]


def reference_optimum(instance, time_limit=20):
    grid = instance["grid"]
    g = _Grid(grid)
    free = {y * g.W + x for y in range(g.H) for x in range(g.W) if grid[y][x] != "#"}
    I = ref.Instance(g, [r * g.W + c for r, c in instance["starts"]],
                     [r * g.W + c for r, c in instance["goals"]], [free] * len(instance["starts"]))
    paths, _ = ref.cbs(I, time_limit)
    return None if paths is None else ref.soc(paths)


def main(out):
    out = Path(out)
    instances = {d["id"]: d for d in json.loads((out / "instances.json").read_text())}
    rows = list(csv.DictReader((out / "results.csv").open(encoding="utf-8")))
    plans = {}
    for line in (out / "plans.jsonl").open(encoding="utf-8"):
        r = json.loads(line)
        plans[(r["instance_id"], r["solver"])] = r["paths"]

    mismatch = collections.Counter()
    checked = 0
    for r in rows:
        if r["status"].startswith("skipped"):
            continue
        p = plans.get((r["instance_id"], r["solver"]))
        if p is None:
            if r["success"] == "True":
                mismatch["success without paths"] += 1
            continue
        checked += 1
        legal, v, s, cost = my_validate(instances[r["instance_id"]], p)
        ok = legal and v == 0 and s == 0
        if str(ok) != r["success"]:
            mismatch["success flag"] += 1
        if r["vertex_conflicts"] != "" and int(r["vertex_conflicts"]) != v:
            mismatch["vertex conflicts"] += 1
        if r["swap_conflicts"] != "" and int(r["swap_conflicts"]) != s:
            mismatch["swap conflicts"] += 1
        if r["sum_of_costs"] != "" and int(r["sum_of_costs"]) != cost:
            mismatch["cost"] += 1
        if not legal:
            mismatch["illegal path"] += 1
    print(f"A. Paths re-checked: {checked}; mismatches with results.csv: {dict(mismatch) or 'none'}")

    optimum = {r["instance_id"]: int(r["sum_of_costs"]) for r in rows
               if r["solver"] == "joint_astar" and r["status"] == "optimal"}
    below = [r["instance_id"] for r in rows
             if r["solver"].startswith("pp") and r["success"] == "True"
             and r["instance_id"] in optimum and int(r["sum_of_costs"]) < optimum[r["instance_id"]]]
    print(f"B. Proven optima: {len(optimum)}; PP results below the optimum: {len(below)}")

    same = diff = unsolved = 0
    for iid, o in optimum.items():
        c = reference_optimum(instances[iid])
        if c is None:
            unsolved += 1
        elif c == o:
            same += 1
        else:
            diff += 1
            print("   differs:", iid, "joint A*", o, "reference CBS", c)
    print(f"C. Optima recomputed with reference CBS: {same} equal, {diff} different, {unsolved} not solved in time")
    return not mismatch and not below and diff == 0


if __name__ == "__main__":
    sys.exit(0 if main(sys.argv[1] if len(sys.argv) > 1 else "../outputs") else 1)
