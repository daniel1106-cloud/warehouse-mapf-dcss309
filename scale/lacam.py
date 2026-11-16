"""LaCAM: a fast, complete MAPF planner (Okumura, AAAI 2023), written from the paper.

Why it is here: prioritized planning (PP) plans robots one at a time and can fail when a
parked robot blocks the way. LaCAM is a newer fast method that never gives up on a
solvable instance (given enough time). It moves all robots one time step at a time:

  low level  PIBT (Okumura et al., 2022) picks every robot's next cell for one step.
             Each robot tries its neighbours in order of distance to its goal. If a
             neighbour is occupied by a robot that has not moved yet, that robot is
             asked to move out of the way first (priority inheritance), recursively.
  high level a depth-first search over configurations (positions of all robots).
             Each configuration remembers which next-step choices it has already tried
             ("constraints": robot i must go to cell v). When PIBT leads into a dead end
             or a configuration seen before, the search tries another constraint, so in
             the limit every successor configuration can be reached. This makes LaCAM
             complete: it finds a plan whenever one exists, given unlimited time.

This is the basic LaCAM of the paper, not LaCAM* (which keeps improving the cost) and
not the engineered C++ code of the authors (lacam3). It uses the same model as the rest
of the repository: vertex and swap conflicts are forbidden, a robot may follow another
into a cell it is leaving, and cost = sum of final arrival times.

Usage: from scale import lacam; lacam.solve(instance, time_limit, seed)
"""
import random
import sys
import time
from collections import deque

from fast_pp import Graph

sys.setrecursionlimit(10000)  # PIBT recursion can be as deep as the number of robots


class Constraint:
    """A chain of fixed moves: robot `who` goes to `where`, plus all moves of its parents."""
    __slots__ = ("parent", "who", "where", "depth")

    def __init__(self, parent=None, who=None, where=None):
        self.parent, self.who, self.where = parent, who, where
        self.depth = 0 if parent is None else parent.depth + 1

    def moves(self):
        c = self
        while c.parent is not None:
            yield c.who, c.where
            c = c.parent


class Node:
    __slots__ = ("config", "parent", "order", "priority", "tree")

    def __init__(self, config, parent, order, priority):
        self.config, self.parent, self.order, self.priority = config, parent, order, priority
        self.tree = deque([Constraint()])


def solve(instance, time_limit=60.0, seed=0, graph=None):
    """Return dict(paths, status, runtime, nodes). status: solved / timeout / no_solution."""
    start_time = time.perf_counter()
    deadline = start_time + time_limit
    graph = graph or Graph(instance["grid"])
    starts, goals = tuple(instance["starts"]), tuple(instance["goals"])
    k = len(starts)
    rng = random.Random(seed)
    dist = [graph.distances(g) for g in goals]
    if any(starts[i] not in dist[i] for i in range(k)):
        return dict(paths=None, status="no_solution", runtime=time.perf_counter() - start_time, nodes=0)
    # candidate cells per robot and cell, sorted by distance to that robot's goal; ties broken
    # randomly once per robot so that the search is reproducible for a given seed
    tie = [{} for _ in range(k)]

    def candidates(i, v):
        c = tie[i].get(v)
        if c is None:
            c = list(graph.steps[v]) + [v]
            rng.shuffle(c)
            c.sort(key=lambda u: dist[i][u])
            tie[i][v] = c
        return c

    def next_priority(config, priority):
        # PIBT's dynamic priorities (as used by LaCAM): a robot away from its goal gains one
        # unit per step, a robot on its goal drops back to its small fractional part. This
        # lets every robot eventually get the highest priority, which avoids livelock.
        return tuple(p + 1 if config[i] != goals[i] else p - int(p) for i, p in enumerate(priority))

    def get_order(priority):
        return sorted(range(k), key=lambda i: -priority[i])

    def generate(config, order, constraint):
        """PIBT step under a constraint chain. Returns the next configuration or None."""
        nxt = [None] * k
        occ_now = {v: i for i, v in enumerate(config)}
        occ_next = {}
        for i, v in constraint.moves():
            if nxt[i] is not None:
                continue  # a deeper (later) constraint for the same robot cannot happen
            if v in occ_next:
                return None  # two robots fixed to the same cell
            j = occ_now.get(v)
            if j is not None and j != i and nxt[j] == config[i]:
                return None  # fixed swap
            nxt[i] = v; occ_next[v] = i
        # a fixed move into a cell whose occupant is fixed to swap back is caught above;
        # the remaining robots are moved by PIBT in priority order

        def pibt(i):
            here = config[i]
            for v in candidates(i, here):
                if occ_next.get(v) is not None:
                    continue  # someone already goes there
                j = occ_now.get(v)
                if j is not None and j != i and nxt[j] == here:
                    continue  # would swap with j
                nxt[i] = v; occ_next[v] = i
                if j is not None and j != i and nxt[j] is None:
                    if not pibt(j):
                        # j could not move and stays on v; v is now j's, try the next cell
                        continue
                return True
            nxt[i] = here; occ_next[here] = i
            return False

        for i in order:
            if nxt[i] is None and not pibt(i):
                return None
        # final consistency check: no two robots on one cell, no swaps
        if len(set(nxt)) != k:
            return None
        for i in range(k):
            j = occ_now.get(nxt[i])
            if j is not None and j != i and nxt[j] == config[i]:
                return None
        return tuple(nxt)

    longest = max(dist[i][starts[i]] for i in range(k)) + 1
    priority0 = tuple(dist[i][starts[i]] / longest for i in range(k))  # in [0, 1): ties by distance
    root = Node(starts, None, get_order(priority0), priority0)
    open_ = [root]
    explored = {starts: root}
    nodes = 1
    while open_:
        if time.perf_counter() > deadline:
            return dict(paths=None, status="timeout", runtime=time.perf_counter() - start_time, nodes=nodes)
        node = open_[-1]
        if node.config == goals:
            configs = []
            while node is not None:
                configs.append(node.config); node = node.parent
            configs.reverse()
            return dict(paths=to_paths(configs, goals), status="solved",
                        runtime=time.perf_counter() - start_time, nodes=nodes)
        if not node.tree:
            open_.pop()
            continue
        c = node.tree.popleft()
        if c.depth < k:
            i = node.order[c.depth]
            options = list(graph.steps[node.config[i]]) + [node.config[i]]
            rng.shuffle(options)
            for v in options:
                node.tree.append(Constraint(c, i, v))
        new = generate(node.config, node.order, c)
        if new is None or new in explored:
            continue
        priority = next_priority(new, node.priority)
        child = Node(new, node, get_order(priority), priority)
        explored[new] = child
        open_.append(child)
        nodes += 1
    return dict(paths=None, status="no_solution", runtime=time.perf_counter() - start_time, nodes=nodes)


def to_paths(configs, goals):
    """Per-robot paths, each cut after the robot's final arrival (it then stays parked)."""
    paths = []
    for i, g in enumerate(goals):
        p = [c[i] for c in configs]
        end = len(p)
        while end > 1 and p[end - 2] == g:
            end -= 1
        paths.append(p[:end])
    return paths
