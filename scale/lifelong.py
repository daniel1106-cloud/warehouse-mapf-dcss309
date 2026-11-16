"""Lifelong planning: tasks keep arriving, as in a real fulfilment centre.

Every other experiment plans one batch: each robot gets one goal and stays there. Real
warehouses never stop: when a robot reaches its goal it gets the next task at once. This
script simulates that with windowed prioritized planning, a simplified version of the
"rolling-horizon collision resolution" (RHCR) framework of Li et al. (AAAI 2021):

  - every H = 5 steps all robots are replanned with PP;
  - each robot's path only has to avoid the others for the next W = 10 steps; beyond that
    the robot is assumed to continue along its shortest path (A* uses it as the estimate);
  - a robot that reaches its goal inside the window waits there until the next replanning,
    then gets a new goal;
  - if PP cannot plan every robot, the failed robot is moved to the front and PP retried
    (up to 10 orders); if all fail, every robot waits for H steps (a "stall").

Every executed step is checked for vertex and swap conflicts. Measures over T = 500 steps:
  throughput  goals reached per time step (higher is better)
  efficiency  sum of the shortest-path lengths of the completed tasks / (robots x steps):
              the share of robot-steps that turned into useful progress. Unlike throughput
              it does not depend on how long the tasks are, so settings with near and far
              goals can be compared.
Timing (mean_plan_sec) is Python time per replanning; throughput and efficiency do not
depend on the machine (no time limit is used).

Settings (benchmark map warehouse-10-20-10-2-1, random distinct start cells):
  goals    shelf      new goals are cells in the shelf area (one-cell aisles)
           open       new goals are cells in the open areas left and right
           alternate  goals alternate between a shelf-area cell and an open-area cell,
                      like fetching a shelf and bringing it to a station
  traffic  two-way, or one-way aisles in the shelf area (scale/oneway.py)
  robots   100, 200, 300; 5 runs (seeds) per setting

Outputs: scale/lifelong_results.csv
Usage:   python scale/lifelong.py            (python scale/lifelong.py --quick for a short check)
"""
import csv
import heapq
import random
import sys
import time
from itertools import count
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
import fast_pp  # noqa: E402
import mapf  # noqa: E402
import oneway  # noqa: E402
from maps import load_map  # noqa: E402

MAP = "warehouse-10-20-10-2-1"
SHELF = (26, 134)
W, H, T = 10, 5, 500
MAX_ORDERS = 10


class Dist:
    """Shortest-path distances to each goal, computed once and cached."""

    def __init__(self, graph):
        self.graph, self.cache = graph, {}

    def __call__(self, goal):
        d = self.cache.get(goal)
        if d is None:
            d = self.cache[goal] = self.graph.distances(goal)
        return d


def plan_one(graph, start, goal, dist, vertex, edge, later):
    """Space-time A* over the next W steps. Returns a path of W+1 cells or None.

    Cost = steps until the robot stands on its goal for the rest of the window, plus the
    shortest-path distance still left at the end of the window (so a robot that cannot
    reach its goal inside the window still moves towards it). `later` holds the cells of
    robots not planned yet, which may not be entered in the first step.
    """
    d = dist(goal)
    if start not in d:
        return None
    tie = count()
    open_ = [(d[start], next(tie), 0, 0, start, None)]
    parent = {}
    while open_:
        f, _, g, t, p, prev = heapq.heappop(open_)
        if (p, t) in parent:
            continue
        parent[(p, t)] = prev
        if t == W:
            path = []
            node = (p, t)
            while node is not None:
                path.append(node[0]); node = parent[node]
            return path[::-1]
        nt = t + 1
        for q in graph.moves[p]:
            if q not in d or (q, nt) in parent or (q, nt) in vertex or (q, p, t) in edge:
                continue
            if nt == 1 and q in later:
                continue
            ng = g if (p == goal and q == goal) else g + 1  # waiting on the goal is free
            heapq.heappush(open_, (ng + d[q], next(tie), ng, nt, q, (p, t)))
    return None


def plan_window(graph, positions, goals, dist, order):
    """PP over the window. Returns (paths, None) or (None, index of the failed robot)."""
    vertex, edge = set(), set()
    later = {positions[i] for i in order}
    paths = [None] * len(positions)
    for i in order:
        later.discard(positions[i])
        path = plan_one(graph, positions[i], goals[i], dist, vertex, edge, later)
        if path is None:
            return None, i
        paths[i] = path
        for t, c in enumerate(path):
            vertex.add((c, t))
        for t in range(W):
            edge.add((path[t], path[t + 1], t))
    return paths, None


def check_step(before, after):
    """No two robots on one cell, no swaps, moves of at most one cell."""
    assert len(set(after)) == len(after), "vertex conflict"
    where = {c: i for i, c in enumerate(before)}
    for i, (a, b) in enumerate(zip(before, after)):
        assert abs(a[0] - b[0]) + abs(a[1] - b[1]) <= 1, "jump"
        j = where.get(b)
        if j is not None and j != i and after[j] == a:
            raise AssertionError("swap conflict")


def simulate(graph, grid, k, goal_mode, seed, steps=T):
    rng = random.Random(seed)
    free = mapf.cells(grid)
    shelf = [p for p in free if SHELF[0] <= p[1] <= SHELF[1]]
    open_ = [p for p in free if not SHELF[0] <= p[1] <= SHELF[1]]
    dist = Dist(graph)
    positions = rng.sample(free, k)
    phase = [rng.random() < 0.5 for _ in range(k)]  # alternate mode: next goal in shelf area?

    def new_goal(i, taken):
        if goal_mode == "shelf":
            pool = shelf
        elif goal_mode == "open":
            pool = open_
        else:
            pool = shelf if phase[i] else open_
            phase[i] = not phase[i]
        while True:
            g = rng.choice(pool)
            if g not in taken and g != positions[i]:
                return g

    goals = []
    for i in range(k):
        goals.append(new_goal(i, set(goals)))
    task_len = [dist(goals[i])[positions[i]] for i in range(k)]  # shortest distance when assigned
    done = stalls = replans = 0
    useful = 0  # sum of the shortest distances of completed tasks
    plan_time = 0.0
    t = 0
    while t < steps:
        # robots standing on their goal get the next task
        taken = set(goals)
        for i in range(k):
            if positions[i] == goals[i]:
                done += 1
                useful += task_len[i]
                taken.discard(goals[i])
                goals[i] = new_goal(i, taken)
                taken.add(goals[i])
                task_len[i] = dist(goals[i])[positions[i]]
        started = time.perf_counter()
        order = list(range(k))
        rng.shuffle(order)
        paths = None
        for _ in range(MAX_ORDERS):
            paths, failed = plan_window(graph, positions, goals, dist, order)
            if paths is not None:
                break
            order.remove(failed); order.insert(0, failed)
        plan_time += time.perf_counter() - started
        replans += 1
        if paths is None:
            stalls += 1
            t += H  # everyone waits
            continue
        for s in range(1, H + 1):
            after = [p[s] for p in paths]
            check_step(positions, after)
            positions = after
        t += H
    # efficiency: share of robot-steps that made shortest-path progress on completed tasks;
    # unlike throughput it does not depend on how long the tasks are
    return dict(throughput=done / steps, goals_reached=done, mean_task_length=useful / done if done else 0,
                efficiency=useful / (k * steps), stalls=stalls, replans=replans,
                mean_plan_sec=plan_time / replans)


def main(quick=False):
    grid = load_map(MAP)
    graphs = {"two_way": fast_pp.Graph(grid), "one_way": oneway.OneWayGraph(grid)}
    sizes = (100,) if quick else (100, 200, 300)
    runs = 1 if quick else 5
    steps = 50 if quick else T
    fields = ["goals", "traffic", "num_robots", "run", "seed", "throughput", "goals_reached", "mean_task_length",
              "efficiency", "stalls", "replans", "mean_plan_sec"]
    target = HERE / ("lifelong_quick.csv" if quick else "lifelong_results.csv")
    with target.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for k in sizes:
            for goal_mode in ("shelf", "open", "alternate"):
                for traffic, graph in graphs.items():
                    for r in range(runs):
                        seed = 20261116 + 1000 * k + r
                        res = simulate(graph, grid, k, goal_mode, seed, steps)
                        row = dict(goals=goal_mode, traffic=traffic, num_robots=k, run=r, seed=seed,
                                   **{key: round(v, 4) if isinstance(v, float) else v for key, v in res.items()})
                        w.writerow(row); f.flush()
                        print(f"k={k} {goal_mode} {traffic} run {r}: throughput {row['throughput']}, "
                              f"stalls {row['stalls']}, {row['mean_plan_sec']} s/replan", flush=True)


if __name__ == "__main__":
    main(quick="--quick" in sys.argv)
