# Experimental Results

Automatically generated from the current results.csv and metadata.json. This file is refreshed after each main run and after the planning-order study.

Configuration: 20 instances per robot count and layout; robot counts from 2 to 20 in steps of 2; 10 seconds per solver run; base seed 20261029.

Map: 12 rows by 20 columns, one-cell-wide connecting aisles, and 45 recessed station/pickup bays. No traversable 2x2 square exists.

Distinct instances: 400. Main-experiment records: 2000.

| Layout | Method | Executed | Skipped | Successful | Timeouts |
| --- | --- | ---: | ---: | ---: | ---: |
| aisle | independent | 200 | 0 | 25 | 0 |
| aisle | joint_astar | 40 | 160 | 39 | 1 |
| aisle | pp | 200 | 0 | 78 | 0 |
| aisle | pp_promote | 200 | 0 | 185 | 0 |
| aisle | pp_shuffle | 200 | 0 | 133 | 0 |
| warehouse | independent | 200 | 0 | 31 | 0 |
| warehouse | joint_astar | 60 | 140 | 56 | 4 |
| warehouse | pp | 200 | 0 | 193 | 0 |
| warehouse | pp_promote | 200 | 0 | 200 | 0 |
| warehouse | pp_shuffle | 200 | 0 | 200 | 0 |

## Q1: Runtime and scalability

Joint A* attempts every instance at the current robot count even if one times out. After completing that size, larger sizes are skipped for that layout. Skipped rows are excluded from runtime medians and success-rate denominators.

| Layout | Robots | Joint A* executed | Successful | Timeouts | Median seconds |
| --- | ---: | ---: | ---: | ---: | ---: |
| aisle | 2 | 20 | 20 | 0 | 0.000533 |
| aisle | 4 | 20 | 19 | 1 | 0.079396 |
| warehouse | 2 | 20 | 20 | 0 | 0.000681 |
| warehouse | 4 | 20 | 20 | 0 | 0.008415 |
| warehouse | 6 | 20 | 16 | 4 | 0.677590 |

These measurements characterize this implementation and machine, not every optimal planner or a production warehouse.

## Q2: Cost relative to a proven optimum

Only matched successful runs on instances with a proven Joint A* optimum are compared. A zero median gap does not mean every run is optimal. Sample counts may be smaller than the number of executed instances.

| Layout | Robots | Method | Paired samples | Median gap (%) | Maximum gap (%) |
| --- | ---: | --- | ---: | ---: | ---: |
| aisle | 2 | pp | 20 | 0.00 | 10.81 |
| aisle | 2 | pp_promote | 20 | 0.00 | 10.81 |
| aisle | 2 | pp_shuffle | 20 | 0.00 | 10.81 |
| aisle | 4 | pp | 17 | 0.00 | 35.00 |
| aisle | 4 | pp_promote | 19 | 0.00 | 35.00 |
| aisle | 4 | pp_shuffle | 19 | 0.00 | 35.00 |
| warehouse | 2 | pp | 20 | 0.00 | 3.92 |
| warehouse | 2 | pp_promote | 20 | 0.00 | 3.92 |
| warehouse | 2 | pp_shuffle | 20 | 0.00 | 3.92 |
| warehouse | 4 | pp | 20 | 0.00 | 6.67 |
| warehouse | 4 | pp_promote | 20 | 0.00 | 6.67 |
| warehouse | 4 | pp_shuffle | 20 | 0.00 | 6.67 |
| warehouse | 6 | pp | 16 | 0.00 | 8.22 |
| warehouse | 6 | pp_promote | 16 | 0.00 | 8.22 |
| warehouse | 6 | pp_shuffle | 16 | 0.00 | 8.22 |

## Q3: Aisle versus warehouse endpoints

- Fixed-order PP on aisle: 78/200 successful (39.0%).
- Fixed-order PP on warehouse: 193/200 successful (96.5%).

The layouts use different endpoint sampling distributions. Differences do not prove that a single factor alone caused the result, or guarantee success in every warehouse.

## Q4: Changing the planning order

- pp_promote: solved 114 instances that fixed-order PP failed to solve.
- pp_shuffle: solved 62 instances that fixed-order PP failed to solve.

The separate order study contains 2000 trials across 400 fixed instances.
- Instances whose success changed with the planning order: 90.
- Instances with different costs across successful orders: 222.
- Repeated orders may occur; trials are not necessarily distinct permutations.

## Interpretation and validation

- All numbers above are computed from saved experimental records; no expected results are substituted for measurements.
- Solver runtime excludes validation, plotting, and GIF saving. The time limit is cooperative, and an over-budget result does not count as a success.
- Skipped Joint A* rows have a runtime of zero as a placeholder; they are not zero-second solutions.
- Reordering stops when it finds a feasible plan. It does not improve the cost further when the initial order already succeeds.
- plans.jsonl saves the paths returned by each solver for independent review; join records to instances.json using instance_id. Skipped and unsuccessful runs may have null paths.
- The included validator is a reference implementation. A teammate should still perform independent cross-validation.
- CSVs, plots, instance data, execution metadata, and the AI assistance disclosure accompany the code.
