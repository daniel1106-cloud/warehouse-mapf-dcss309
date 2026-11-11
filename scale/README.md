# Scale Experiment: a Standard Warehouse Benchmark Map

The main experiment uses a 12 × 20 map with 137 free cells and at most 20 robots. This folder repeats the prioritized-planning part on a map of realistic size: **`warehouse-10-20-10-2-1`** from the MovingAI MAPF benchmark (Stern et al., 2019), 161 × 63 cells with 5,699 free cells, about 40 times the main map.

![Scale results](scale.png)

## Setup

| Item | Value |
| --- | --- |
| Map | `maps/warehouse-10-20-10-2-1.map` (shelf blocks of 10 × 2 cells, one-cell aisles, open areas left and right) |
| Robots | 50, 100, 150, 200, 250, 300 |
| Instances | 10 per size; starts and goals are distinct random free cells (base seed 20261111) |
| Methods | independent BFS (collisions only), fixed-order PP, PP with the failed robot first |
| Time limit | 60 s per run (Python) |

Endpoints are random free cells, as in the benchmark's "random" scenarios. About half of the free cells (2,649 of 5,699) are in the shelf area with its one-cell aisles; the rest are in the open areas on both sides. A robot parked on a goal in an aisle can block that aisle, as in the main experiment's **aisle** layout.

Joint A* and CBS are not run. At these sizes the optimum cannot be computed, so each PP cost is compared with a **lower bound** on the optimum: the sum of every robot's own shortest-path length. Every robot needs at least that many steps, so optimum ≥ lower bound, and `cost / lower bound − 1` is an **upper bound** on PP's real gap.

## Results

| Robots | Collisions without coordination (median) | Fixed-order PP: plans found | Failed robot first: plans found | Fixed-order PP: median time of successful runs | Fixed-order PP cost above lower bound: median / max |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 50 | 32 | 10/10 | 10/10 | 0.4 s | 3.0% / 5.7% |
| 100 | 127 | 8/10 | 8/10 | 3.3 s | 12.1% / 14.4% |
| 150 | 359 | 10/10 | 10/10 | 6.2 s | 14.1% / 17.4% |
| 200 | 702.5 | 8/10 | 8/10 | 10.4 s | 17.0% / 19.4% |
| 250 | 1,105.5 | 5/10 | 6/10 | 16.5 s | 20.0% / 24.9% |
| 300 | 1,418.5 | 1/10 | 2/10 | 29.1 s | (1 run) |

"Collisions" counts vertex and swap conflicts. Times and costs cover successful runs only; the plot's middle panel instead takes the median over all runs, counting a timeout as 60 s. Where both PP variants succeeded, their costs were identical. Full data: [`results.csv`](results.csv).

1. **Coordination is essential.** Without it, conflicts grow from 32 at 50 robots to over 1,400 at 300.
2. **PP plans hundreds of robots in seconds to tens of seconds**, even in plain Python: 200 robots in about 10 s, 250 in about 16 s (successful runs).
3. **The upper bound on the cost gap grows with crowding.** Its median rises from 3.0% at 50 robots to 20.0% at 250, and the real gap is at most this large. The lower bound ignores robots getting in each other's way, so it probably becomes looser as the floor gets busier; part of the rise may come from the bound rather than from PP.
4. **Timeouts come from robots parked in aisles.** Fixed-order PP failed 18 times: 14 timeouts and 4 `blocked`. In all 14 timeouts, the robot being planned has its goal in the shelf area and is cut off from it once the robots planned before it are parked ([`failure_diagnosis.txt`](failure_diagnosis.txt)), as in the main experiment's aisle layout. A likely reason for the timeout rather than a quick `blocked`: the search horizon is the latest earlier arrival plus 5,699 steps, so the space-time search has a huge number of (cell, time) combinations to rule out first. The 4 `blocked` failures are different: there the robot could still reach its goal around the parked robots, so the cause lies in the earlier robots' moving paths; we did not analyse them further.
5. **Reordering rescued little.** Failed-robot-first rescued 2 of the 18 failures, both `blocked` ones. It cannot help after a timeout: its first attempt is the same as fixed-order PP and uses up the shared 60 s budget. The other 2 `blocked` failures were not rescued either; their second attempt timed out.

## How the speed-up was checked

`mapf.py` checks every earlier robot's path for every candidate move. That is too slow at this size: with 50 robots `mapf.py` took 1.7 to 9.3 s per run where `fast_pp.py` took 0.3 to 1.3 s ([`equivalence_check.txt`](equivalence_check.txt)), and in a trial run (not saved) 200 robots did not finish within 2 minutes. [`fast_pp.py`](fast_pp.py) implements the same algorithm with the same neighbour order, tie-breaking, horizon and reordering rules, but keeps the reservations in hash tables. [`check_equivalence.py`](check_equivalence.py) confirms that both return identical paths and statuses:

- all 1,200 PP runs of the main experiment (400 instances × 3 PP methods), against the saved `outputs/plans.jsonl`;
- 12 runs on this map with 25 and 50 robots (own random instances), run side by side with `mapf.py`.

Output of the run: [`equivalence_check.txt`](equivalence_check.txt).

Plans in this folder are checked with `check_plan` in `run_scale.py`, which gave the same result as `verification/check_results.py`'s validator on 12 test plans (including colliding ones).

## Limits

- **Python runtimes.** Production planners are compiled and heavily optimized; the times show the trend, not achievable speed.
- **10 instances per size.** Success rates move in steps of 10 percentage points; the dip at 100 robots (8/10) next to 150 (10/10) is within that noise.
- **Random endpoints.** The benchmark map has no recessed bays, so this does not repeat the main experiment's warehouse layout.
- **One-shot planning.** Real warehouses assign new tasks continuously (lifelong MAPF); this experiment plans one batch.
- **Map source.** The benchmark site was not reachable from the environment used, so the map was copied from the `lacam3` repository ([github.com/kei18/lacam3](https://github.com/kei18/lacam3), commit `1c3ecf8`, `scripts/map/`). The original is at [movingai.com/benchmarks/mapf](https://movingai.com/benchmarks/mapf/index.html).

## How to run

From the repository root:

```bash
python scale/check_equivalence.py   # fast_pp.py vs mapf.py (several minutes)
python scale/run_scale.py           # full experiment (about 1 hour)
python scale/run_scale.py --quick   # small check (seconds)
python scale/diagnose_failures.py   # why the failed runs failed (about 15 minutes)
```

## Reference

Stern, R. et al. (2019). *Multi-Agent Pathfinding: Definitions, Variants, and Benchmarks.* Symposium on Combinatorial Search (SoCS).

The code in this folder was written with the help of Claude (AI assistant).
