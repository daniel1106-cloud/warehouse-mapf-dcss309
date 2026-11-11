"""Stress test on small, crowded grids where robots conflict often.

Compares the project's Joint A* (optimal cost) with the independent reference CBS,
and checks every Joint A* and PP (failed-robot-first) plan with the independent
validator from check_results.py.

Usage:  python stress_test.py ../mapf.py [trials]
"""
import importlib.util
import random
import sys

import reference_cbs as ref
from check_results import my_validate, _Grid

GRIDS = [["....", "....", "...."],
         [".....", ".#.#.", "....."],
         ["......", ".####.", "......"],
         ["...", ".#.", "...", ".#.", "..."]]


def load_project(path):
    spec = importlib.util.spec_from_file_location("project_mapf", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main(project_path, trials=400, seed=7):
    J = load_project(project_path)
    rng = random.Random(seed)
    s = dict(trials=0, equal_optimum=0, different_optimum=0, both_no_plan=0, reference_timeout=0,
             joint_none_reference_found=0,
             joint_timeout=0, joint_invalid=0, pp_invalid=0, pp_below_optimum=0)
    for trial in range(trials):
        grid = rng.choice(GRIDS)
        free = [(r, c) for r in range(len(grid)) for c in range(len(grid[0])) if grid[r][c] != "#"]
        k = rng.choice([2, 3, 4])
        cells = rng.sample(free, 2 * k)
        inst = dict(grid=grid, starts=cells[:k], goals=cells[k:])
        s["trials"] += 1
        joint = J.solve(inst, 5, "joint_astar")
        pp = J.solve(inst, 5, "pp_promote", seed=trial)
        g = _Grid(grid)
        open_cells = {r * g.W + c for r, c in free}
        I = ref.Instance(g, [r * g.W + c for r, c in inst["starts"]],
                         [r * g.W + c for r, c in inst["goals"]], [open_cells] * k)
        ref_paths, _ = ref.cbs(I, 5)
        if joint["status"] == "timeout":
            s["joint_timeout"] += 1
            continue
        if joint["paths"] is None and ref_paths is None:
            # Joint A* searched everything and found no plan; CBS also returned none
            s["both_no_plan"] += 1
            continue
        if joint["paths"] is None:
            s["joint_none_reference_found"] += 1  # would be a disagreement
            print("disagreement: Joint A* no plan, CBS found one:", inst)
            continue
        if ref_paths is None:
            s["reference_timeout"] += 1  # CBS gave up in 5 s; Joint A* plan still validated below
        legal, v, sw, cost = my_validate(inst, joint["paths"])
        if not legal or v or sw:
            s["joint_invalid"] += 1
            continue
        if ref_paths is None:
            pass
        elif cost == ref.soc(ref_paths):
            s["equal_optimum"] += 1
        else:
            s["different_optimum"] += 1
            print("different optimum:", inst, cost, ref.soc(ref_paths))
        if pp["paths"] is not None:
            l2, v2, sw2, c2 = my_validate(inst, pp["paths"])
            if not l2 or v2 or sw2:
                s["pp_invalid"] += 1
            elif c2 < cost:
                s["pp_below_optimum"] += 1
    print(s)
    return (s["different_optimum"] == s["joint_invalid"] == s["pp_invalid"] == s["pp_below_optimum"]
            == s["joint_none_reference_found"] == 0)


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "../mapf.py"
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 400
    sys.exit(0 if main(path, n) else 1)
