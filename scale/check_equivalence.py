"""Check that scale/fast_pp.py returns exactly what mapf.py returns.

1. Every PP run of the main experiment (400 instances x 3 PP methods): the fast
   version must return the same status and the same paths as saved in
   outputs/plans.jsonl and outputs/results.csv.
2. The real warehouse map at 25 and 50 robots: mapf.solve and fast_pp.solve are run
   side by side on the same instances.

Usage: python scale/check_equivalence.py
"""
import csv
import json
import random
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(HERE))
import mapf  # noqa: E402
import fast_pp  # noqa: E402
from maps import load_map  # noqa: E402

METHODS = ("pp", "pp_promote", "pp_shuffle")


def as_instance(d):
    return dict(grid=d["grid"], starts=[tuple(p) for p in d["starts"]], goals=[tuple(p) for p in d["goals"]])


def main_experiment():
    out = ROOT / "outputs"
    instances = {d["id"]: d for d in json.loads((out / "instances.json").read_text())}
    rows = {(r["instance_id"], r["solver"]): r for r in csv.DictReader((out / "results.csv").open(encoding="utf-8"))}
    saved = {}
    for line in (out / "plans.jsonl").open(encoding="utf-8"):
        r = json.loads(line)
        if r["solver"] in METHODS:
            saved[(r["instance_id"], r["solver"])] = r["paths"]
    compared = same = 0
    for iid, d in instances.items():
        for m in METHODS:
            row = rows[(iid, m)]
            if row["status"] == "timeout":
                continue  # a timeout depends on the machine, not on the algorithm
            r = fast_pp.solve(as_instance(d), 10, m, d["seed"], 32)
            mine = None if r["paths"] is None else [[list(c) for c in p] for p in r["paths"]]
            compared += 1
            if mine == saved.get((iid, m)) and r["status"] == row["status"]:
                same += 1
            else:
                print("DIFFERENT:", iid, m, row["status"], r["status"])
    print(f"main experiment: {same}/{compared} PP runs identical (paths and status)")
    return same == compared


def real_map(map_name="warehouse-10-20-10-2-1", sizes=(25, 50), per_size=3):
    grid = load_map(map_name)
    free = mapf.cells(grid)
    compared = same = 0
    for k in sizes:
        for j in range(per_size):
            rng = random.Random(1000 * k + j)
            sel = rng.sample(free, 2 * k)
            inst = dict(grid=grid, starts=sel[:k], goals=sel[k:])
            for m in ("pp", "pp_promote"):
                a = mapf.solve(inst, 300, m, j)
                b = fast_pp.solve(inst, 300, m, j)
                if a["status"] == "timeout" or b["status"] == "timeout":
                    continue
                compared += 1
                if a["paths"] == b["paths"] and a["status"] == b["status"]:
                    same += 1
                else:
                    print("DIFFERENT:", map_name, k, j, m)
                print(f"  {map_name} k={k} #{j} {m}: mapf.py {a['runtime']:.2f} s, fast {b['runtime']:.2f} s", flush=True)
    print(f"real map: {same}/{compared} runs identical")
    return same == compared


if __name__ == "__main__":
    ok = main_experiment() & real_map()
    sys.exit(0 if ok else 1)
