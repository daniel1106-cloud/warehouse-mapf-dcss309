"""Prioritized planning with a hash-table reservation table.

Same algorithm as `pp`, `pp_promote` and `pp_shuffle` in mapf.py, step for step:
same space-time A*, same neighbour order, same tie-breaking, same horizon, same
reordering rules. Only the data structures change: neighbour lists are computed once
per map, and reservations are kept in hash tables. mapf.py scans every earlier
path for every candidate move, which costs O(number of robots) per move; here each
lookup is a constant-time hash query. On the same input both return the same paths
(checked by scale/check_equivalence.py).
"""
import heapq
import random
import sys
import time
from itertools import count
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from mapf import Limit, check, cells, neighbors, reconstruct  # noqa: E402

INF = float("inf")


class Graph:
    """Neighbour lists computed once per grid, in the same order as mapf.neighbors."""

    def __init__(self, grid):
        free = cells(grid)
        self.n_cells = len(free)
        self.moves = {p: list(neighbors(grid, p)) for p in free}                 # wait first, then 4 moves
        self.steps = {p: [q for q in self.moves[p] if q != p] for p in free}     # 4 moves only

    def distances(self, goal):
        """Same result as mapf.distances (BFS from the goal)."""
        d = {goal: 0}; queue = [goal]; head = 0
        while head < len(queue):
            p = queue[head]; head += 1
            for q in self.steps[p]:
                if q not in d:
                    d[q] = d[p] + 1; queue.append(q)
        return d


class Reservations:
    """Where earlier robots are, indexed for constant-time queries."""

    def __init__(self, n_cells):
        self.vertex = set()     # (cell, t) while a robot is still on its way
        self.edge = set()       # (from_cell, to_cell, t): a move between t and t+1
        self.parked = {}        # cell -> time from which a robot stays there for good
        self.last_visit = {}    # cell -> last time any robot is there (INF if parked)
        self.starts = set()
        self.max_end = 0        # latest time step at which an earlier path ends
        self.n_cells = n_cells

    def add(self, path):
        end = len(path) - 1
        for t, p in enumerate(path[:-1]):
            self.vertex.add((p, t))
            if path[t + 1] != p:
                self.edge.add((p, path[t + 1], t))
            self.last_visit[p] = max(self.last_visit.get(p, -1), t)
        goal = path[-1]
        self.parked[goal] = end
        self.last_visit[goal] = INF
        self.starts.add(path[0])
        self.max_end = max(self.max_end, end)


def single_reserved(graph, start, goal, res, deadline):
    """Line-for-line counterpart of mapf.single_reserved."""
    d = graph.distances(goal)
    if start not in d:
        return None, "unreachable"
    horizon = res.max_end + res.n_cells
    if start in res.starts:
        return None, "blocked"
    vertex, edge, parked, moves = res.vertex, res.edge, res.parked, graph.moves
    goal_free_after = res.last_visit.get(goal, -1)
    serial = count(); root = (start, 0); parent = {root: None}; heap = [(d[start], next(serial), root)]
    while heap:
        check(deadline); _, _, node = heapq.heappop(heap); p, t = node
        if p == goal and goal_free_after < t:
            return [q for q, _ in reconstruct(parent, node)], "solved"
        if t >= horizon:
            continue
        t1 = t + 1
        for q in moves[p]:
            if q not in d:
                continue
            # vertex conflict, parked robot, or swap with an earlier robot
            if (q, t1) in vertex or parked.get(q, INF) <= t1 or (q, p, t) in edge:
                continue
            nxt = (q, t1)
            if nxt not in parent:
                parent[nxt] = node; heapq.heappush(heap, (t1 + d[q], next(serial), nxt))
    return None, "blocked"


def pp(instance, deadline, order, graph):
    res = Reservations(graph.n_cells); result = [None] * len(order)
    for i in order:
        path, status = single_reserved(graph, instance["starts"][i], instance["goals"][i], res, deadline)
        if path is None:
            return None, status, i
        res.add(path); result[i] = path
    return result, "solved", None


def solve(instance, time_limit=10, method="pp", seed=0, max_attempts=32):
    """Same interface and return value as mapf.solve, for the three PP methods."""
    begun = time.perf_counter(); deadline = begun + time_limit; paths = None; attempts = 1
    order = list(range(len(instance["starts"])))
    graph = Graph(instance["grid"])
    try:
        if method == "pp":
            paths, status, _ = pp(instance, deadline, order, graph)
        elif method in ("pp_promote", "pp_shuffle"):
            rng = random.Random(seed); seen = set(); status = "attempt_limit"
            for attempts in range(1, max_attempts + 1):
                check(deadline)
                key = tuple(order)
                if key in seen:
                    rng.shuffle(order); key = tuple(order)
                seen.add(key)
                paths, status, failed = pp(instance, deadline, order, graph)
                if paths is not None:
                    break
                if method == "pp_promote":
                    order.remove(failed); order.insert(0, failed)
                else:
                    rng.shuffle(order)
            if paths is None:
                status = "attempt_limit"
        else:
            raise ValueError(method)
    except Limit:
        status = "timeout"; paths = None
    elapsed = time.perf_counter() - begun
    return dict(success=paths is not None, paths=paths, runtime=elapsed, status=status, attempts=attempts, order=order)
