# AI Assistance Disclosure

ChatGPT / Codex assisted with the design and implementation of this version based on the team proposal and task screenshots. Assistance covered the Python code, map generator, BFS, Joint A*, space-time A*, PP and reordering strategies, reference validator, experiments, plots, GIF, tests, documentation, and automatic English report generation.

The included measurements were produced by actual program execution. No estimated or manually invented results are substituted for measurements. See outputs/metadata.json for configuration and environment details. The team must review the findings rather than presenting the proposal's hypotheses as established facts.

Before submission, students should run the project themselves and be able to explain:

1. The differences between BFS and A*, and why the heuristic does not overestimate remaining cost.
2. Why joint action enumeration grows exponentially with the number of robots.
3. Vertex conflicts, swap conflicts, and permanent goal occupancy.
4. Why PP can fail for one order and succeed after reordering.
5. Why total cost is the sum of final arrival times, including waiting and departure after an earlier goal visit.
6. Why cost gaps require proven optimal solutions for the same instances.
7. The differences between a timeout, an attempt limit, a skipped run, a collision, and proven infeasibility.
8. Why the complete current robot-count batch is finished before Joint A* skips larger sizes, and how reports are computed from current CSV records.

The teammate assigned to independent validation should still perform that work. The reference validator is included so this version can run independently. Actual returned paths are saved in outputs/plans.jsonl to support cross-validation.
