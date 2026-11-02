# Update Notes

## Single-cell aisle map revision

The former map has been replaced by a 12x20 shelf map with strictly one-cell-wide connecting aisles and 45 recessed station/pickup bays. Each station has a single aisle neighbor, and no traversable 2x2 square exists. Tests check these properties, connectivity, and sufficient unique endpoints for 20 robots. All included results are regenerated for this revised map.

## Earlier changes

1. Joint A* now completes every instance at the current robot count, even if a run times out. Only larger robot counts are skipped after that complete batch. Every executed default-size group therefore has 20 instances.
2. The English EXPERIMENT_REPORT.md is generated from the current CSV files and metadata. It is refreshed after the main experiment and again after the planning-order study. Default runs update both the project-root report and the report in outputs; custom runs update only their own output report.
3. Returned paths are stored in outputs/plans.jsonl. A teammate can join each record's instance_id to the id in instances.json and independently inspect conflicts and cost.
4. The planning-order study inherits the actual main-run time limit, including the shorter quick-mode budget.
5. The project launcher supports both --output folder and --output=folder. It runs the tests before launching experiments.
6. A pipeline regression test verifies that Joint A* completes the timeout-size batch before skipping larger sizes, path records are saved, and report numbers change when the source CSV changes. The suite now contains 15 tests.
7. The README and AI disclosure are in English. The report explains skipped runs, paired optimal-cost comparisons, and sample counts.

## Running the Update

Extract this package into a new folder, open a terminal in warehouse_mapf, and run:

```bash
python -m pip install -r requirements.txt
python run_project.py
```

The default run may take several minutes because every instance in the final Joint A* batch is attempted with its own 10-second budget. The package also includes results from an actual updated run.

Independent teammate validation and uploading to the group repository remain team tasks.
