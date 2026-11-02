# Independent Verification

The planners, experiment pipeline and reference validator in the project root were written in one package. This folder checks that package with code written separately, so the code that produces the results is not the only code that judges them.

None of the scripts here import the project's `mapf.py`, except `stress_test.py`, which loads it only to call the planners under test.

## Files

| File | Purpose |
| --- | --- |
| `check_results.py` | Re-checks every saved path with its own validator, compares with `results.csv`, and recomputes every proven optimum with the reference CBS |
| `stress_test.py` | 400 random instances on small crowded grids: Joint A* optimum vs. reference CBS, and validity of Joint A* and PP plans |
| `cbs_reach.py` | Measures how many robots the reference CBS can still solve optimally on the experiment's own instances |
| `reference_cbs.py` | Separately written MAPF code (space-time A*, CBS, own validator) used only as a reference |
| `results/` | Saved outputs of the runs described below |

## How to run

From this folder:

```bash
python check_results.py ../outputs
python stress_test.py ../mapf.py 400
python cbs_reach.py ../outputs 5 10
```

## Results (2026-11-02)

**1. Full re-run on a second machine** (Linux, Python 3, default settings, `python run_project.py`): all 15 tests passed. Compared with the original run in `outputs/`, the 400 instances are identical, and all 2,000 result rows have the same success value and the same cost. Joint A* stopped at the same instances (aisle `aisle-k4-001`, warehouse `warehouse-k6-008`). The planning-order study gave identical success values and costs. Only runtimes differ (total 445 s vs. 381 s). Summary files: `results/rerun/`.

**2. `check_results.py`** (`results/check_results_rerun.txt`)

- 1,484 returned paths re-checked: no mismatch with `results.csv` in success, vertex conflicts, swap conflicts or cost; no illegal path.
- 95 proven Joint A* optima: no PP result below the optimum.
- All 95 optima recomputed with the reference CBS: 95 equal, 0 different.

**3. `stress_test.py`** (`results/stress_test.txt`): in 354 of 400 instances both solvers returned a plan, with the same optimal cost every time. All Joint A* and PP plans were valid, and no PP plan had a cost below the optimum. In the other 46 instances, Joint A* finished its search and reported that no solution exists, and the reference CBS also returned no plan, so the two agree (counted as `reference_unsolved` in the output).

**4. `cbs_reach.py`** (`results/cbs_reach.csv`, 5 instances per size, 10 s per instance)

| Layout | Robots solved optimally by CBS | Joint A* in the main experiment |
| --- | --- | --- |
| aisle | all 5 up to 6 robots, 1 of 5 at 8, none at 10 | stops at 4 robots |
| warehouse | all 5 up to 6 robots, 2–3 of 5 at 8–12, none at 14 | stops at 6 robots |

CBS extends the range of proven optima only moderately, and only with small samples. Where it solved warehouse instances with 8–12 robots, fixed-order PP was a median 0.8–4.5% and at most 9.0% above the optimum.

## Limits

- The checks confirm that the saved results are internally consistent and agree with an independent solver. They do not test whether the experimental design answers the project questions.
- `reference_cbs.py` assumes the same model: 4-neighbour moves and waits, vertex and swap conflicts, parked robots occupy their goal, cost = sum of final arrival times.

## AI assistance

The scripts in this folder and `reference_cbs.py` were written with the help of an AI assistant (Claude). The team member responsible for verification reviewed and ran them.
