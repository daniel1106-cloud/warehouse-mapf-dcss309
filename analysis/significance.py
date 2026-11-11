"""Significance tests for the main success-rate comparisons.

Two kinds of test, both exact (no large-sample approximation):
  Fisher's exact test   two independent groups of instances (e.g. aisle vs warehouse layout)
  McNemar's exact test  the same instances solved by two methods; only instances where the
                        methods disagree count (a two-sided binomial test on those)
A small p-value means the difference would be unlikely if the two settings or methods
really had the same success rate. With 10 instances per setting, only large differences
can reach a small p-value.

Output: analysis/significance.txt
Usage:  python analysis/significance.py
"""
import csv
from pathlib import Path

from scipy.stats import binomtest, fisher_exact

ROOT = Path(__file__).resolve().parent.parent


def rows(path):
    return list(csv.DictReader((ROOT / path).open(encoding="utf-8")))


def ok(r):
    return str(r["success"]) == "True"


def fisher(label, a, b):
    """a, b: lists of booleans (success per instance) for two independent groups."""
    p = fisher_exact([[sum(a), len(a) - sum(a)], [sum(b), len(b) - sum(b)]]).pvalue
    return f"{label}: {sum(a)}/{len(a)} vs {sum(b)}/{len(b)}, Fisher exact p = {p:.2g}"


def mcnemar(label, pairs):
    """pairs: list of (success with method 1, success with method 2) on the same instances."""
    gained = sum(1 for x, y in pairs if not x and y)
    lost = sum(1 for x, y in pairs if x and not y)
    n = gained + lost
    p = binomtest(gained, n, 0.5).pvalue if n else 1.0
    return (f"{label}: {sum(x for x, _ in pairs)} -> {sum(y for _, y in pairs)} of {len(pairs)} "
            f"(gained {gained}, lost {lost}), McNemar exact p = {p:.2g}")


def main():
    out = []

    main_rows = rows("outputs/results.csv")
    pick = lambda layout, solver: {r["instance_id"]: ok(r) for r in main_rows
                                   if r["layout"] == layout and r["solver"] == solver}
    out.append("Main experiment (12 x 20 map, 200 instances per layout)")
    out.append("  " + fisher("fixed-order PP, aisle vs warehouse layout",
                             list(pick("aisle", "pp").values()), list(pick("warehouse", "pp").values())))
    for layout in ("aisle", "warehouse"):
        a, b = pick(layout, "pp"), pick(layout, "pp_promote")
        out.append("  " + mcnemar(f"{layout}: fixed order vs failed robot first", [(a[i], b[i]) for i in a]))

    out.append("")
    out.append("Benchmark map, where robots stop (10 instances per size and setting)")
    ep = rows("scale/endpoints_results.csv")
    matched_path = ROOT / "scale/endpoints_matched.csv"
    matched = rows("scale/endpoints_matched.csv") if matched_path.exists() else []
    get = lambda data, area, solver, ks: [ok(r) for r in data if r["goal_area"] == area
                                          and r["solver"] == solver and int(r["num_robots"]) in ks]
    for ks in ((100,), (200,), (300,), (200, 300)):
        name = " and ".join(map(str, ks)) + " robots"
        shelf = get(ep, "shelf_area", "pp", ks)
        out.append("  " + fisher(f"{name}, fixed order, shelf aisles vs open areas", shelf, get(ep, "open_area", "pp", ks)))
        if matched:
            out.append("  " + fisher(f"{name}, fixed order, shelf aisles vs open areas with 2,649 cells",
                                     shelf, get(matched, "open_matched", "pp", ks)))

    out.append("")
    out.append("Benchmark map, cut-off test (same 60 instances)")
    base = {r["instance_id"]: ok(r) for r in rows("scale/results.csv") if r["solver"] == "pp_promote"}
    cut = {r["instance_id"]: ok(r) for r in rows("scale/cutoff_results.csv") if r["solver"] == "pp_promote_cutoff"}
    out.append("  " + mcnemar("failed robot first, without vs with cut-off test", [(base[i], cut[i]) for i in cut]))

    text = "\n".join(out) + "\n"
    (ROOT / "analysis/significance.txt").write_text(text, encoding="utf-8")
    print(text, end="")


if __name__ == "__main__":
    main()
