# Warehouse Robots: Optimal Planning vs. Prioritized Planning

Reproducible experimental project · DCSS309 Algorithms · Group A4

## Quick Start (Windows / macOS / Linux)

Install Python 3.10 or later. Open a terminal in the folder containing this file:

```bash
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v
python run_project.py
```

On Windows, use `py` if `python` is unavailable. On macOS/Linux, use `python3` if necessary.

Running `python run_project.py` executes the tests, main experiment, and planning-order experiment. It generates `outputs/results.csv`, `runtime.png`, `success.png`, `cost_gap.png`, `demo.gif`, the original instances, and execution metadata. The package includes results from an actual run. The English experiment report is generated from current CSV data after each run; the planning-order study refreshes it again with its own results. A report is saved in the output folder, and the project-root report is also updated when using the default `outputs` folder. Running the project again overwrites the corresponding output files.

For a quick check:

```bash
python run.py --quick --output quick_outputs
```

To run the planning-order experiment separately after the main experiment:

```bash
python order_study.py
```

This generates `outputs/order_sensitivity.csv`. Each instance is tested with five planning-order trials, including the fixed order. The remaining trials are performed even if the first order succeeds. Repeated orders may occur, particularly with small numbers of robots. These are five trials, not necessarily five distinct permutations.

To regenerate the plots from an existing CSV:

```bash
python run.py --plot-only
```

## Project Goals

- **Q1: Why is optimal planning unsuitable for large robot fleets?** Compare the running times of Joint A* and prioritized planning (PP).
- **Q2: How much solution quality is lost by using a faster method?** Compare costs only when a proven optimum is available for the same instance.
- **Q3: Does warehouse endpoint design make PP more reliable?** Compare success rates between aisle and warehouse instances.
- **Q4: Why change the planning priorities?** Compare fixed-order PP, failed-robot promotion, random reordering, and a separate planning-order sensitivity experiment.

Expected outcomes are hypotheses to test. This project does not assume that PP always has a small optimality gap or that warehouse instances always succeed.

## Files

| File | Purpose |
| --- | --- |
| `mapf.py` | Data interface, map generation, BFS, space-time A*, Joint A*, PP, two reordering strategies, and a reference validator |
| `run_project.py` | Runs the tests, main experiment, and planning-order experiment with one command |
| `run.py` | Batch experiments, CSV output, three plots, and GIF generation |
| `order_study.py` | Changes only the PP planning order while keeping each instance fixed |
| `tests/test_mapf.py` | Tests for the five specified examples, edge cases, and optimality |
| `outputs/` | Generated experimental results |
| `AI_USAGE.md` | AI assistance disclosure and concepts that students should understand |
| `report.py` | Generates the English report from current experimental data |
| `EXPERIMENT_REPORT.md` | Automatically updated interpretation of the latest default-output run |

## Data Interface

Coordinates use zero-based `(row, column)` indexing. `#` represents an impassable shelf, `.` an aisle, and `S` a traversable workstation. At each time step, a robot may move one cell up, down, left, or right, or wait in place. The grid is rectangular. Start cells are distinct, and goal cells are distinct.

```python
from mapf import solve, validate

instance = {
    'grid': ['...', '...'],
    'starts': [(0, 0), (0, 2)],
    'goals': [(0, 2), (0, 0)],
}
result = solve(instance, time_limit=10, method='pp', seed=42)
# {'success': ..., 'paths': ..., 'runtime': ..., 'status': ...,
#  'attempts': ..., 'order': ...}
if result['paths'] is not None:
    report = validate(instance, result['paths'])
```

`paths[i][t]` is the position of robot `i` at time `t`. Paths may have different lengths. After its path ends, a robot permanently occupies its final cell.

`solve()['success']` indicates that the solver returned paths. The main experiment validates these paths before recording the final `success` value in the CSV. Independent BFS intentionally serves as an uncoordinated baseline; paths containing collisions do not count as successful solutions.

Team members may replace `validate` with an independently implemented validator for cross-checking. The included validator supports development checks and the experimental pipeline; it does not replace the independent validation assigned to another team member.

## Methods

- **`independent`**: Each robot independently finds a shortest path using BFS, without coordinating with other robots.
- **`joint_astar`**: A state contains all robot positions and flags indicating which robots have permanently finished. The search enumerates joint actions and rejects vertex and swap conflicts. Its admissible heuristic is the sum of the individual static BFS distances to the goals. It minimizes the sum of final arrival times. A result is considered proven optimal only when its status is `optimal`.
- **`pp`**: Robots are planned sequentially in index order using space-time A*. Later robots avoid the vertex occupancies, reverse-edge conflicts, and permanent goal occupancies of previously planned robots. This greedy method does not guarantee global optimality or success on every solvable instance.
- **`pp_promote`**: After a planning failure, the failed robot is moved to the front, and all paths are replanned. Random reordering breaks cycles when an order repeats.
- **`pp_shuffle`**: After a planning failure, the order is randomly shuffled, and all paths are replanned.

Each reordering method allows up to 32 attempts within a shared 10-second budget per solver run. `attempt_limit` means that the retry limit was reached; it does not prove that the instance has no solution. For PP, `blocked` means only that the current planning order could not produce a solution.

Joint A* explicitly handles robots that reach a goal and later leave it. A robot may permanently finish, after which it cannot move, or remain active and pay a cost for each time step. Waiting at a goal cannot always be assigned zero cost: doing so would underestimate the final arrival time if the robot later left the goal.

## Experimental Design

The default map has 12 rows and 20 columns. Four one-cell-wide horizontal aisles (rows 1, 4, 7, and 10) are joined by one-cell-wide vertical aisles at columns 0 and 19. Shelf cells fill the remaining space except for 45 recessed station/pickup bays, including stations on the top and bottom boundaries. Every bay has exactly one adjacent aisle cell. No traversable 2x2 square exists, including station cells.

- **Aisle instances**: Distinct start and goal cells are randomly sampled from aisle cells. A parked robot may obstruct traffic.
- **Warehouse instances**: Start and goal cells are sampled only from recessed station/pickup bays. These model stopping locations at boundary workstations or in shelf areas. Each bay is accessible from one adjacent aisle and does not lie on a through route.
- Robot counts are `k = 2, 4, ..., 20`, with 20 instances per robot count and layout, producing 400 instances.
- Five methods are included for every instance, producing 2,000 CSV records, including explicitly marked skipped runs.
- Random seeds are determined by the base seed, layout, robot count, and instance index. All methods receive exactly the same instances.
- Each solver run has a 10-second limit. The search frequently checks a monotonic clock. This is a cooperative time limit, rather than operating-system-enforced process termination. Initialization or the final operation may cause a small overrun. Results exceeding the solver budget do not count as successful; validation is performed separately.
- **For each layout, Joint A* completes every instance at the current robot count, even after the first timeout. Only larger robot counts are skipped.** Timeouts are recorded as actual timeouts. Runs at larger skipped sizes are marked `skipped_after_timeout`; they are not treated as executed failures or assigned fabricated 10-second runtimes. Skipped runs are excluded from plotted success rates and runtime medians. See `metadata.json` for the stopping instance.
- This is a small-scale implementation of published principles, not a reproduction of Amazon's actual maps, code, or thousand-robot performance.

The space-time A* horizon is the latest end time among previously planned paths plus the number of traversable cells. After all previously planned robots have parked, the remaining problem is a static graph. If the goal is reachable, a simple path requires fewer steps than the number of traversable cells. This horizon is therefore sufficient for the fixed set of previously planned paths.

## Metrics

- **Vertex conflicts**: Two robots occupy the same cell at the same time. Counted per robot pair and time step.
- **Swap conflicts**: Two robots exchange two distinct positions between consecutive time steps. Counted per robot pair and transition.
- **Cost**: The sum of the times at which each robot reaches its goal for the final time and never leaves again. Waiting before final arrival is included. Repeated goal positions at the end of a path do not add cost.
- **Success rate**: The proportion of executed runs that produce legal, collision-free paths within the time limit. Skipped runs are excluded from the denominator.
- **Runtime plot**: Median solver runtime across executed runs, including successful runs, failures, and timeouts.
- **Cost-gap plot**: Uses only matched instances for which Joint A* has established the optimum and the compared method succeeds. The gap is `100 * (cost / optimum - 1)`. Different methods may have different numbers of matched successful instances. Percentage gaps are not calculated when the optimum is zero.
- **Independent baseline costs**: Costs of paths containing collisions are retained only for diagnosis and are not treated as valid solution-quality comparisons against the optimum.

`results.csv` records the layout, robot count, instance ID, seed, method, validated success, status, solver runtime, cost, both conflict counts, number of attempts, final planning order, optimal cost, and percentage cost gap. The original instances are stored in `instances.json`. Coordinate arrays loaded from JSON can be converted to tuples. `plans.jsonl` stores returned paths for each instance/method, so a teammate can independently validate the actual solutions. Join the two files using `instance_id` (the corresponding instance field is `id`).

## Animation

`demo.gif` intentionally demonstrates collisions in the independent BFS baseline. Robots have different colors, crosses indicate their goals, and red circles mark vertex-conflict cells or both endpoints of a swap conflict at the current time step. The title displays the current time. This illustrates a failure of the uncoordinated baseline; it is not evidence of PP failure.

## Configurable Parameters

```bash
python run.py --instances 20 --max-robots 20 --time-limit 10 --seed 20261029 --attempts 32 --output outputs
```

Run the tests before conducting the full experiment. Runtime measurements vary across machines. For a consistent comparison, run all methods on the same machine and minimize other computationally intensive activity.

## Independent Verification

The `verification/` folder re-checks the saved results with separately written code: its own validator for every returned path, an independent CBS solver for every proven optimum, a stress test on small crowded grids, and a full re-run on a second machine. See [`verification/README.md`](verification/README.md).

## References

This implementation was independently written from the method descriptions in the team proposal. It does not copy a paper's implementation or company code.

- Standley (2010), *Finding Optimal Solutions to Cooperative Pathfinding Problems*.
- Silver (2005), *Cooperative Pathfinding*.
- Bennewitz, Burgard & Thrun (2002), *Finding and Optimizing Solvable Priority Schemes for Decoupled Path Planning Techniques for Teams of Mobile Robots*.
- Čáp et al. (2015), *Prioritized Planning Algorithms for Trajectory Coordination of Multiple Mobile Robots*.

The algorithm names correspond to those in the proposal. These are educational implementations and do not claim to reproduce every optimization or guarantee described in the papers.

## Regenerating the English Report

```bash
python report.py --output outputs --include-order
```

Use `--include-order` only when `order_sensitivity.csv` belongs to the current main experiment. `run_project.py` handles this sequence automatically. A custom or quick output folder receives its own report without overwriting the main project report.
