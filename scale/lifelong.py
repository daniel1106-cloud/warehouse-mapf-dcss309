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
  - robots are planned in a random order; a robot that cannot be planned waits in place
    for the window (it is then planned first and planning restarts), so a stuck robot
    stops only itself, never the whole warehouse.

Every executed step is checked for vertex and swap conflicts. Measures over T = 500 steps:
  throughput  goals reached per time step (higher is better)
  efficiency  sum of the shortest-path lengths of the completed tasks (on the two-way map)
              / (robots x steps): the share of robot-steps that turned into useful
              progress. Unlike throughput it does not depend on how long the tasks are, so
              settings with near and far goals can be compared.
  also        share of robot-steps spent moving, robots that never finished a task, the
              longest time a task stayed open, robots forced to wait per replanning.
Simplifications against RHCR: PP instead of PBS/ECBS (no completeness); nothing is
reserved beyond the window; a goal counts only when the robot stands on it at a
replanning (every H steps), so robots may idle on a reached goal for up to H steps;
goals reached at the last step are not counted. Each robot has its own random task
stream, so two-way and one-way runs give robots the same sequence of task areas.
Timing (mean_plan_sec) is Python time per replanning; throughput and efficiency do not
depend on the machine (no time limit is used).

Settings (benchmark map warehouse-10-20-10-2-1, random distinct start cells):
  goals    shelf      new goals are cells in the shelf area (one-cell aisles)
           open       new goals are cells in the open areas left and right
           alternate  goals alternate between a shelf-area cell and an open-area cell,
                      like fetching a shelf and bringing it to a station
  traffic  two-way, or one-way aisles in the shelf area (scale/oneway.py)
  robots   100, 200, 300; 5 runs (seeds) per setting

Outputs: scale/lifelong_results.csv, scale/lifelong.png
Usage:   python scale/lifelong.py            (--quick for a short check, --plot-only to redraw)
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


def check_step(graph, before, after):
    """Every move is a legal edge of the graph (or a wait); no shared cells; no swaps."""
    if len(set(after)) != len(after):
        raise RuntimeError("vertex conflict")
    where = {c: i for i, c in enumerate(before)}
    for i, (a, b) in enumerate(zip(before, after)):
        if b not in graph.moves[a]:
            raise RuntimeError(f"illegal move {a} -> {b}")
        j = where.get(b)
        if j is not None and j != i and after[j] == a:
            raise RuntimeError("swap conflict")


def plan_with_waits(graph, positions, goals, dist, rng):
    """Plan every robot; a robot that cannot be planned waits in place for the window.

    Waiting robots are planned first (their cells reserved for the whole window), the
    others in a random order. When a robot fails, it joins the waiting robots and planning
    restarts. This always ends: at worst every robot waits, which is always legal.
    """
    k = len(positions)
    waiting = []
    while True:
        rest = [i for i in range(k) if i not in waiting]
        rng.shuffle(rest)
        vertex, edge = set(), set()
        paths = [None] * k
        for i in waiting:
            paths[i] = [positions[i]] * (W + 1)
            for t in range(W + 1):
                vertex.add((positions[i], t))
        later = {positions[i] for i in rest}
        failed = None
        for i in rest:
            later.discard(positions[i])
            path = plan_one(graph, positions[i], goals[i], dist, vertex, edge, later)
            if path is None:
                failed = i
                break
            paths[i] = path
            for t, c in enumerate(path):
                vertex.add((c, t))
            for t in range(W):
                edge.add((path[t], path[t + 1], t))
        if failed is None:
            return paths, len(waiting)
        waiting.append(failed)


def simulate(graph, grid, k, goal_mode, seed, steps=T, base_graph=None):
    rng = random.Random(seed)                      # starts and planning orders
    goal_rng = [random.Random(seed * 1000 + i) for i in range(k)]  # each robot's own task stream
    free = mapf.cells(grid)
    shelf = [p for p in free if SHELF[0] <= p[1] <= SHELF[1]]
    open_ = [p for p in free if not SHELF[0] <= p[1] <= SHELF[1]]
    dist = Dist(graph)
    # task length is measured on the two-way map, so one-way rules get no credit for detours
    base_dist = Dist(base_graph) if base_graph is not None else dist
    positions = rng.sample(free, k)
    phase = [goal_rng[i].random() < 0.5 for i in range(k)]  # alternate mode: next goal in shelf area?

    def new_goal(i, taken):
        if goal_mode == "shelf":
            pool = shelf
        elif goal_mode == "open":
            pool = open_
        else:
            pool = shelf if phase[i] else open_
            phase[i] = not phase[i]
        while True:
            g = goal_rng[i].choice(pool)
            if g not in taken and g != positions[i]:
                return g

    goals = []
    for i in range(k):
        goals.append(new_goal(i, set(goals)))
    task_len = [base_dist(goals[i])[positions[i]] for i in range(k)]
    assigned_at = [0] * k
    finished = [0] * k
    done = forced_waits = replans = moves = 0
    useful = 0  # sum of the shortest distances of completed tasks
    max_age = 0
    plan_time = 0.0
    t = 0
    while t < steps:
        # robots standing on their goal get the next task
        taken = set(goals)
        for i in range(k):
            if positions[i] == goals[i]:
                done += 1; finished[i] += 1
                useful += task_len[i]
                max_age = max(max_age, t - assigned_at[i])
                taken.discard(goals[i])
                goals[i] = new_goal(i, taken)
                taken.add(goals[i])
                task_len[i] = base_dist(goals[i])[positions[i]]
                assigned_at[i] = t
        started = time.perf_counter()
        paths, n_wait = plan_with_waits(graph, positions, goals, dist, rng)
        plan_time += time.perf_counter() - started
        replans += 1
        forced_waits += n_wait
        for s in range(1, H + 1):
            after = [p[s] for p in paths]
            check_step(graph, positions, after)
            moves += sum(a != b for a, b in zip(positions, after))
            positions = after
        t += H
    max_age = max([max_age] + [steps - a for a in assigned_at])  # include unfinished tasks
    return dict(throughput=done / steps, goals_reached=done, mean_task_length=useful / done if done else 0,
                efficiency=useful / (k * steps), moving_share=moves / (k * steps),
                robots_without_task_done=sum(f == 0 for f in finished), max_task_age=max_age,
                forced_waits_per_replan=forced_waits / replans, replans=replans,
                mean_plan_sec=plan_time / replans)


def main(quick=False):
    grid = load_map(MAP)
    graphs = {"two_way": fast_pp.Graph(grid), "one_way": oneway.OneWayGraph(grid)}
    sizes = (100,) if quick else (100, 200, 300)
    runs = 1 if quick else 5
    steps = 50 if quick else T
    fields = ["goals", "traffic", "num_robots", "run", "seed", "throughput", "goals_reached", "mean_task_length",
              "efficiency", "moving_share", "robots_without_task_done", "max_task_age", "forced_waits_per_replan",
              "replans", "mean_plan_sec"]
    target = HERE / ("lifelong_quick.csv" if quick else "lifelong_results.csv")
    with target.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for k in sizes:
            for goal_mode in ("shelf", "open", "alternate"):
                for traffic, graph in graphs.items():
                    for r in range(runs):
                        seed = 20261116 + 1000 * k + r
                        res = simulate(graph, grid, k, goal_mode, seed, steps, graphs["two_way"])
                        row = dict(goals=goal_mode, traffic=traffic, num_robots=k, run=r, seed=seed,
                                   **{key: round(v, 4) if isinstance(v, float) else v for key, v in res.items()})
                        w.writerow(row); f.flush()
                        print(f"k={k} {goal_mode} {traffic} run {r}: throughput {row['throughput']}, "
                              f"efficiency {row['efficiency']}, waits/replan {row['forced_waits_per_replan']}, "
                              f"idle robots {row['robots_without_task_done']}, {row['mean_plan_sec']} s/replan",
                              flush=True)


def plot(target=HERE / "lifelong.png"):
    """Throughput against robots, one panel per goal setting, two-way vs one-way."""
    import statistics
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = list(csv.DictReader((HERE / "lifelong_results.csv").open(encoding="utf-8")))
    sizes = sorted({int(r["num_robots"]) for r in rows})
    titles = {"shelf": "Tasks in shelf aisles", "open": "Tasks in open areas",
              "alternate": "Alternating shelf and open area"}
    style = {"two_way": ("Two-way aisles", "#2a78d6", "o", "-"), "one_way": ("One-way aisles", "#eb6834", "s", "--")}
    fig, axes = plt.subplots(1, 3, figsize=(12, 4), sharey=True)
    for ax, (mode, title) in zip(axes, titles.items()):
        for traffic, (label, color, marker, ls) in style.items():
            vals = [[float(r["throughput"]) for r in rows if r["goals"] == mode and r["traffic"] == traffic
                     and int(r["num_robots"]) == k] for k in sizes]
            mean = [statistics.mean(v) for v in vals]
            ax.plot(sizes, mean, color=color, marker=marker, markersize=8, linewidth=2, linestyle=ls, label=label)
            ax.vlines(sizes, [min(v) for v in vals], [max(v) for v in vals], color=color, linewidth=2)
            ax.annotate(label, (sizes[-1], mean[-1]), xytext=(6, 0), textcoords="offset points",
                        va="center", fontsize=8, color="#333333")
        ax.set_title(title, fontsize=11)
        ax.set_xlabel("Robots"); ax.set_xticks(sizes); ax.set_xlim(sizes[0] - 30, sizes[-1] + 110)
        ax.grid(alpha=.2); ax.spines[["top", "right"]].set_visible(False)
    axes[0].set_ylabel("Tasks finished per time step")
    axes[0].legend(fontsize=8, frameon=False, loc="upper left")
    fig.suptitle("Lifelong planning (windowed PP), warehouse-10-20-10-2-1: mean of 5 runs, bars = min to max")
    fig.tight_layout()
    fig.savefig(target, dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    if "--plot-only" in sys.argv:
        plot()
    else:
        main(quick="--quick" in sys.argv)
        if "--quick" not in sys.argv:
            plot()
