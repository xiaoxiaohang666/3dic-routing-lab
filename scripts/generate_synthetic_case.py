#!/usr/bin/env python3
"""Generate a small synthetic two-die case for router regression runs.

Variants cover input shapes seen on public cases without copying them:
  wall        macros fill low layers, pins sit under them, heat rises left to right
  tsv-scarce  macros cover most of the die, so few cells may hold a TSV; one hot spot
  open        no macros (control: new options must not hurt here)
Results on these cases say nothing about the official hidden cases.
"""
import argparse
import json
import math
import pathlib
import random

VARIANTS = {
    "wall": {"macro_fraction": 0.25, "nets": 1500, "cross": 0.15, "heat": "gradient"},
    "tsv-scarce": {"macro_fraction": 0.70, "nets": 1200, "cross": 0.30, "heat": "hotspot"},
    "open": {"macro_fraction": 0.0, "nets": 1500, "cross": 0.15, "heat": "gradient"},
}
PITCH = 150  # DBU per GCell; with a track step of 10 each cell holds 15 tracks.
DBU = 100
GRID = 120
MACRO_CELLS = 16
LOW_PIN, HIGH_PIN = "metal3", "metal12"


def layer_block(name, layers, size):
    rects = "".join(f"    LAYER {layer} ;\n      RECT 0 0 {size} {size} ;\n" for layer in layers)
    return f"  OBS\n{rects}  END\n"


def write_lef(path, macro_pins):
    cell = PITCH / DBU
    size = MACRO_CELLS * cell
    text = []
    for layer in range(1, 21):
        direction = "HORIZONTAL" if layer % 2 else "VERTICAL"
        text.append(f"LAYER metal{layer}\n  TYPE ROUTING ;\n  DIRECTION {direction} ;\n  WIDTH 0.1 ;\nEND metal{layer}\n")
    # The pin layer stays open inside a macro, so a net can step sideways under
    # the full layers above it. Whether it does is the router's decision.
    for name, pin_layer, blocked in (("wall_low", LOW_PIN, [1, 2, 4, 5]), ("wall_high", HIGH_PIN, [11, 13, 14, 15])):
        text.append(f"MACRO {name}\n  CLASS BLOCK ;\n  SIZE {size} BY {size} ;\n")
        for pin, (col, row) in macro_pins.items():
            x, y = (col + 0.5) * cell, (row + 0.5) * cell
            text.append(f"  PIN {pin}\n    PORT\n      LAYER {pin_layer} ;\n      RECT {x - .05:.3f} {y - .05:.3f} {x + .05:.3f} {y + .05:.3f} ;\n    END\n  END {pin}\n")
        text.append(layer_block(name, [f"metal{layer}" for layer in blocked], size))
        text.append(f"END {name}\n")
    for name, layer in (("std_low", "metal2"), ("std_high", "metal12")):
        text.append(f"MACRO {name}\n  CLASS CORE ;\n  SIZE 0.2 BY 0.2 ;\n  PIN A\n    PORT\n      LAYER {layer} ;\n      RECT 0.05 0.05 0.15 0.15 ;\n    END\n  END A\nEND {name}\n")
    path.write_text("".join(text))


def macro_sites(fraction, rng):
    """Whole-macro sites until the requested share of the die is covered."""
    slots = GRID // MACRO_CELLS
    sites = [(c * MACRO_CELLS, r * MACRO_CELLS) for r in range(slots) for c in range(slots)]
    rng.shuffle(sites)
    return sites[: round(fraction * GRID * GRID / (MACRO_CELLS * MACRO_CELLS))]


def write_def(path, spec, macro_pins, rng):
    sites = macro_sites(spec["macro_fraction"], rng)
    components, pool_low, pool_high = [], [], []
    for index, (col, row) in enumerate(sites):
        master, pool = (("wall_low", pool_low), ("wall_high", pool_high))[index % 2]
        name = f"m{index}"
        components.append(f"- {name} {master} + PLACED ( {col * PITCH} {row * PITCH} ) N ;")
        pool.extend((name, pin) for pin in macro_pins)
    taken = {(c + dc, r + dr) for c, r in sites for dc in range(MACRO_CELLS) for dr in range(MACRO_CELLS)}
    free = [(c, r) for r in range(GRID) for c in range(GRID) if (c, r) not in taken]
    rng.shuffle(free)
    needed = 2 * spec["nets"] + 400
    for index in range(needed):
        col, row = free[index % len(free)]
        master, pool = (("std_low", pool_low), ("std_high", pool_high))[index % 2]
        name = f"s{index}"
        components.append(f"- {name} {master} + PLACED ( {col * PITCH + 40} {row * PITCH + 40} ) N ;")
        pool.append((name, "A"))
    rng.shuffle(pool_low)
    rng.shuffle(pool_high)

    def take(pool):
        return pool.pop() if pool else None

    nets = []
    while len(nets) < spec["nets"]:
        degree = rng.choices((2, 3, 4, 6), (60, 25, 10, 5))[0]
        cross = rng.random() < spec["cross"]
        if cross:
            picks = [take(pool_low), take(pool_high)] + [take(rng.choice((pool_low, pool_high))) for _ in range(degree - 2)]
        else:
            pool = rng.choice((pool_low, pool_high))
            picks = [take(pool) for _ in range(degree)]
        picks = [pick for pick in picks if pick]
        if len(picks) < 2:
            break
        nets.append(picks)
    # One wide net keeps a high-fanout shape in every variant.
    hub = [pick for pick in (take(pool_low) for _ in range(32)) if pick] + [pick for pick in (take(pool_high) for _ in range(32)) if pick]
    if len(hub) >= 2:
        nets.append(hub)
    if len(nets) <= 1000:
        raise SystemExit("synthetic case needs more than 1000 nets to use the large-design search")
    die = GRID * PITCH
    lines = [f"UNITS DISTANCE MICRONS {DBU} ;", f"DIEAREA ( 0 0 ) ( {die} {die} ) ;",
             f"GCELLGRID X 0 DO {GRID + 1} STEP {PITCH} ;", f"GCELLGRID Y 0 DO {GRID + 1} STEP {PITCH} ;"]
    tracks = GRID * PITCH // 10
    for layer in range(1, 21):
        lines += [f"TRACKS X 5 DO {tracks} STEP 10 LAYER metal{layer} ;", f"TRACKS Y 5 DO {tracks} STEP 10 LAYER metal{layer} ;"]
    lines.append(f"COMPONENTS {len(components)} ;")
    lines += components
    lines += ["END COMPONENTS", "PINS 0 ;", "END PINS", f"NETS {len(nets)} ;"]
    for index, picks in enumerate(nets):
        lines.append(f"- n{index} " + " ".join(f"( {comp} {pin} )" for comp, pin in picks) + " ;")
    lines += ["END NETS", "END DESIGN"]
    path.write_text("\n".join(lines) + "\n")
    return len(nets), len(sites)


def write_thermal(path, heat):
    size = GRID * PITCH / DBU
    samples = 100
    nodes = []
    for j in range(samples):
        for i in range(samples):
            x, y = (i + .5) * size / samples, (j + .5) * size / samples
            if heat == "gradient":
                value = 25 + 5 * x / size
            else:
                value = 25 + 5 * math.exp(-(((x - .8 * size) ** 2 + (y - .2 * size) ** 2) / (2 * (.15 * size) ** 2)))
            nodes.append([round(x, 3), round(y, 3), 0.0, round(value, 5)])
            nodes.append([round(x, 3), round(y, 3), -0.001, round(value + .01, 5)])
    path.write_text(json.dumps({"meta": {"ambientTemp": 25.0}, "nodes": nodes}))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--variant", required=True, choices=sorted(VARIANTS))
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--output-dir", required=True, type=pathlib.Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)
    spec = VARIANTS[args.variant]
    rng = random.Random(f"{args.variant}:{args.seed}")
    # Edge pins can reach open columns within a few cells; centre pins cannot.
    macro_pins = {}
    for k in range(MACRO_CELLS):
        macro_pins[f"L{k}"] = (k % 3, k)
        macro_pins[f"R{k}"] = (MACRO_CELLS - 1 - k % 3, k)
    for k in range(4):
        macro_pins[f"C{k}"] = (MACRO_CELLS // 2, MACRO_CELLS // 2 - 2 + k)
    stem = args.output_dir / f"synthetic_{args.variant.replace('-', '_')}"
    write_lef(stem.with_suffix(".input.lef"), macro_pins)
    nets, macros = write_def(stem.with_suffix(".input.def"), spec, macro_pins, rng)
    write_thermal(stem.with_suffix(".thermal.input.json"), spec["heat"])
    print(json.dumps({"variant": args.variant, "seed": args.seed, "nets": nets, "macros": macros, **spec}))


if __name__ == "__main__":
    main()
