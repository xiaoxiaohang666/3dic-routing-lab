"""Read-only Guide-cell graph structure; this is not detailed-route topology."""
from pathlib import Path
from collections import Counter, defaultdict
from bisect import bisect_left, bisect_right
import argparse
import csv
import json
import re


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for key in ("def", "lef", "guide", "attribution", "output"):
        p.add_argument("--" + key, dest="definition" if key == "def" else key, type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        p.error("output must be new")
    targets = defaultdict(list)
    with (args.attribution / "owners.csv").open() as stream:
        for r in csv.DictReader(stream):
            if int(r["demand"]) > int(r["mandatory_tsv"]):
                targets[r["net"]].append(tuple(int(r[k]) for k in ("layer", "col", "row")))
    text = args.definition.read_text()
    die = list(map(int, re.search(r"DIEAREA\s*\(\s*(-?\d+)\s+(-?\d+)\s*\)\s*\(\s*(-?\d+)\s+(-?\d+)", text).groups()))
    grids = {}
    for axis, low, high in [("X", die[0], die[2]), ("Y", die[1], die[3])]:
        lines = {low, high}
        for start, count, step in re.findall(r"GCELLGRID\s+" + axis + r"\s+(-?\d+)\s+DO\s+(\d+)\s+STEP\s+(\d+)", text):
            lines.update(int(start) + i * int(step) for i in range(int(count)) if low <= int(start) + i * int(step) <= high)
        grids[axis] = sorted(lines)
    tech = args.lef.read_text()
    horizontal = {int(m[1][5:]): re.search(r"DIRECTION\s+(HORIZONTAL|VERTICAL)", m[2])[1] == "HORIZONTAL"
                  for m in re.finditer(r"(?ms)^LAYER (metal\d+)\s+(.*?)^END \1\s*$", tech)}
    layer_stacks = defaultdict(Counter)
    structures = []

    def process(name, rects):
        if name not in targets:
            return
        nodes = set()
        for x1, y1, x2, y2, layer in rects:
            for row in range(max(0, bisect_right(grids["Y"], y1) - 1), min(len(grids["Y"]) - 1, bisect_left(grids["Y"], y2))):
                for col in range(max(0, bisect_right(grids["X"], x1) - 1), min(len(grids["X"]) - 1, bisect_left(grids["X"], x2))):
                    nodes.add((layer, col, row))

        def adjacent(n):
            layer, col, row = n
            planar = [(layer, col - 1, row), (layer, col + 1, row)] if horizontal[layer] else [(layer, col, row - 1), (layer, col, row + 1)]
            return [m for m in planar + [(layer - 1, col, row), (layer + 1, col, row)] if m in nodes]

        for layer, col, row in targets[name]:
            assert (layer, col, row) in nodes
            lo = hi = layer
            while lo > 1 and (lo - 1, col, row) in nodes:
                lo -= 1
            while hi < 20 and (hi + 1, col, row) in nodes:
                hi += 1
            layer_stacks[layer][hi - lo] += 1
        degrees = Counter(len(adjacent(n)) for n in nodes)
        unseen = set(nodes)
        components = 0
        while unseen:
            components += 1
            todo = [unseen.pop()]
            while todo:
                for other in adjacent(todo.pop()):
                    if other in unseen:
                        unseen.remove(other)
                        todo.append(other)
        edges = sum(k * v for k, v in degrees.items()) // 2
        structures.append({"net": name, "nodes": len(nodes), "edges": edges, "components": components,
                           "cell_contact_cycle_rank": edges - len(nodes) + components,
                           "degree_histogram": dict(degrees), "billed_hotspot_occurrences": len(targets[name])})

    net, rects = None, []
    with args.guide.open() as stream:
        for line in stream:
            f = line.split()
            if f == [")"]:
                process(net, rects)
                rects = []
            elif f == ["("] or not f:
                continue
            elif len(f) == 1:
                net = f[0]
            elif net in targets:
                rects.append((*map(int, f[:4]), int(f[4][5:])))
    assert len(structures) == len(targets)
    report = {"scope": __doc__, "target_nets": len(targets), "grid": {k: len(v) - 1 for k, v in grids.items()},
              "stack_via_count_histogram_by_layer": {k: dict(v) for k, v in layer_stacks.items()},
              "structures": structures, "warning": "Contact cycles and degree alone do not establish removable routing or via cost; preserve PIN/TSV and remeasure serialized candidates."}
    with args.output.open("x") as f:
        json.dump(report, f, indent=2)
        f.write("\n")
    print(json.dumps({k: report[k] for k in ("target_nets", "grid", "stack_via_count_histogram_by_layer")}))


if __name__ == "__main__":
    main()
