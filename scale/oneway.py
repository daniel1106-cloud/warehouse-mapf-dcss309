"""One-way aisles: does a traffic rule help prioritized planning?

Amazon describes "social rules" for its robots, such as virtual streets with a fixed
direction. Here the benchmark map's shelf area gets such rules:
  - horizontal aisles (rows 1, 4, 7, ...) alternate: eastbound, westbound, eastbound, ...
  - vertical aisles between shelf blocks (columns 36, 47, ..., 124) alternate: southbound,
    northbound, ...
  - the open areas left and right (and the columns 25 and 135 bordering them) stay two-way.
Waiting is always allowed. The directed graph is strongly connected (checked below), so
every goal stays reachable. Planning uses the same PP code (fast_pp) with this graph;
the heuristic distances are directed shortest paths to the goal.

Instances: the random-endpoint instances with 200-300 robots (scale/instances.json) and
the shelf-aisle-goal instances with 200 and 300 robots (run_endpoints.py), the settings
where two-way PP failed most. Same 60 s limit. Cost is reported against the same two-way
lower bound as before, so it includes any detour the rules force.

Outputs: scale/oneway_results.csv
Usage:   python scale/oneway.py
"""
import csv
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
import fast_pp  # noqa: E402
import run_endpoints  # noqa: E402
from maps import load_map  # noqa: E402
from run_scale import MAP, check_plan, lower_bound  # noqa: E402

SHELF = (26, 134)
H_AISLES = list(range(1, 62, 3))           # rows 1, 4, ..., 61
V_AISLES = list(range(36, 125, 11))        # columns 36, 47, ..., 124


def allowed(p, q):
    """May a robot step from p to q under the one-way rules? (waiting is always allowed)"""
    (r1, c1), (r2, c2) = p, q
    if r1 == r2:  # horizontal move
        if r1 in H_AISLES and SHELF[0] <= c1 <= SHELF[1] and SHELF[0] <= c2 <= SHELF[1]:
            east = H_AISLES.index(r1) % 2 == 0
            return (c2 > c1) == east
        return True
    if c1 in V_AISLES:  # vertical move inside a shelf-area aisle
        south = V_AISLES.index(c1) % 2 == 0
        return (r2 > r1) == south
    return True


class OneWayGraph(fast_pp.Graph):
    def __init__(self, grid):
        super().__init__(grid)
        self.moves = {p: [q for q in m if q == p or allowed(p, q)] for p, m in self.moves.items()}
        self.steps = {p: [q for q in m if q != p] for p, m in self.moves.items()}
        self.reverse = {p: [] for p in self.steps}
        for p, qs in self.steps.items():
            for q in qs:
                self.reverse[q].append(p)

    def distances(self, goal):
        """Directed shortest-path length from every cell to the goal (BFS on reversed edges)."""
        d = {goal: 0}; queue = [goal]; head = 0
        while head < len(queue):
            p = queue[head]; head += 1
            for q in self.reverse[p]:
                if q not in d:
                    d[q] = d[p] + 1; queue.append(q)
        return d


def strongly_connected(graph):
    any_cell = next(iter(graph.steps))
    forward = graph.distances(any_cell)          # cells that can reach any_cell
    back = {any_cell}; queue = [any_cell]; head = 0
    while head < len(queue):
        p = queue[head]; head += 1
        for q in graph.steps[p]:
            if q not in back:
                back.add(q); queue.append(q)
    return len(forward) == len(back) == len(graph.steps)


def instances():
    grid = load_map(MAP)
    for d in json.loads((HERE / "instances.json").read_text())["instances"]:
        if len(d["starts"]) >= 200:
            yield "random", d["id"], d["seed"], dict(grid=grid, starts=[tuple(p) for p in d["starts"]],
                                                     goals=[tuple(p) for p in d["goals"]])
    for area, k, j, seed, inst in run_endpoints.make_instances():
        if area == "shelf_area" and k >= 200:
            yield "shelf_goals", f"{area}-k{k}-{j:03d}", seed, inst


def main(limit=60):
    grid = load_map(MAP)
    two_way, one_way = fast_pp.Graph(grid), OneWayGraph(grid)
    assert strongly_connected(one_way), "one-way rules disconnect the map"
    fields = ["set", "num_robots", "instance_id", "seed", "solver", "success", "status", "runtime_sec",
              "attempts", "sum_of_costs", "lower_bound", "gap_upper_pct"]
    with (HERE / "oneway_results.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for which, iid, seed, inst in instances():
            lb = lower_bound(inst, two_way)
            r = fast_pp.solve(inst, limit, "pp", seed, 32, graph=one_way)
            row = dict(set=which, num_robots=len(inst["starts"]), instance_id=iid, seed=seed, solver="pp_oneway",
                       status=r["status"], runtime_sec=round(r["runtime"], 4), attempts=r["attempts"],
                       lower_bound=lb, success=False, sum_of_costs="", gap_upper_pct="")
            if r["paths"] is not None:
                legal, v, s, cost = check_plan(inst, r["paths"])
                legal = legal and all(a == b or allowed(a, b) for p in r["paths"] for a, b in zip(p, p[1:]))
                ok = legal and v == 0 and s == 0 and r["runtime"] <= limit
                row.update(success=ok, sum_of_costs=cost)
                if ok:
                    row["gap_upper_pct"] = round(100 * (cost / lb - 1), 3)
                else:
                    row["status"] = "timeout" if r["runtime"] > limit else "invalid"
            w.writerow(row); f.flush()
            print(f"{iid}: {row['status']} {row['runtime_sec']:.1f}s gap={row['gap_upper_pct']}", flush=True)


if __name__ == "__main__":
    main()
