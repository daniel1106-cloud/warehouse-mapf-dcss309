"""Load MovingAI benchmark maps (.map) into the grid format used by mapf.py."""
from pathlib import Path

MAP_DIR = Path(__file__).resolve().parent / "maps"


def load_map(name):
    """Return the map as a list of strings: '.' free, '#' blocked."""
    lines = (MAP_DIR / f"{name}.map").read_text().splitlines()
    height = int(lines[1].split()[1])
    width = int(lines[2].split()[1])
    assert lines[3].strip() == "map"
    rows = lines[4:4 + height]
    assert len(rows) == height and all(len(r) == width for r in rows)
    # The warehouse maps use only '.' (free) and 'T' (obstacle, here: shelves).
    return ["".join("." if ch in ".G" else "#" for ch in row) for row in rows]
