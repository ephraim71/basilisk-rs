#!/usr/bin/env python3
"""The BSM-versus-MuJoCo runtime tables of the dynamics-engine comparison
(upstream `_runtimeTable.py` and `make -C docs comparison-runtime-tables`).

`just dyncomp-runtime-tables` runs the five accuracy scenarios with
`--timing`, each writing `<scenario>_runtime.csv` (case, BSM and MuJoCo
median propagation seconds of five interleaved trials after a warm-up,
setup excluded, and the BSM/MuJoCo speedup). This script collects the
tables in `--generated-dir`, prints them as one Markdown table, writes it to
`dyncompRuntime.md` and draws `dyncompRuntime.png`, the two engines' times
per case on a log axis. The values describe the machine and build that
measured them; they are not pass/fail results.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

from dyncomp_plotting import COLOR_BSM, COLOR_MUJOCO
from orbit_plotting import DEFAULT_GENERATED_DIR

SCENARIOS = (
    "scenarioCompareOrbit",
    "scenarioCompareTorque",
    "scenarioCompareRwPanels",
    "scenarioCompareOrbitMultibody",
    "scenarioCompareVariableMass",
)


def load(generated_dir: Path) -> list[tuple[str, str, float, float | None, str]]:
    """`(scenario, case, BSM s, MuJoCo s or None, speedup)` of every table found."""
    rows = []
    for scenario in SCENARIOS:
        path = generated_dir / f"{scenario}_runtime.csv"
        if not path.is_file():
            continue
        with path.open(newline="", encoding="utf-8") as handle:
            reader = csv.reader(handle)
            next(reader)
            for case, bsm, mujoco, speedup in reader:
                rows.append((scenario, case, float(bsm), None if mujoco == "n/a" else float(mujoco), speedup))
    return rows


def markdown(rows) -> str:
    lines = ["| Scenario | Case | BSM [s] | MuJoCo [s] | Speedup (BSM/MuJoCo) |", "|---|---|---|---|---|"]
    for scenario, case, bsm, mujoco, speedup in rows:
        mujoco_cell = "n/a" if mujoco is None else f"{mujoco:.4f}"
        lines.append(f"| {scenario} | {case} | {bsm:.4f} | {mujoco_cell} | {speedup} |")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--generated-dir", type=Path, default=DEFAULT_GENERATED_DIR)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--show", action="store_true")
    arguments = parser.parse_args()
    rows = load(arguments.generated_dir)
    if not rows:
        print(f"error: no *_runtime.csv in {arguments.generated_dir} (run `just dyncomp-runtime-tables`)",
              file=sys.stderr)
        return 1
    table = markdown(rows)
    print(table)
    output_dir = arguments.output_dir or arguments.generated_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "dyncompRuntime.md").write_text(table, encoding="utf-8")

    import matplotlib

    if not arguments.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    labels = [f"{scenario.removeprefix('scenarioCompare')}: {case}" for scenario, case, *_ in rows]
    positions = np.arange(len(rows))
    figure, axis = plt.subplots(figsize=(8, 0.6 * len(rows) + 1.5))
    axis.barh(positions - 0.2, [row[2] for row in rows], height=0.4, color=COLOR_BSM, label="BSM")
    axis.barh(positions + 0.2, [row[3] or 0.0 for row in rows], height=0.4, color=COLOR_MUJOCO, label="MuJoCo")
    axis.set_yticks(positions, labels, fontsize=7)
    axis.invert_yaxis()
    axis.set_xscale("log")
    axis.set_xlabel("Median propagation time [s]")
    axis.legend(loc="best")
    figure.tight_layout()
    output = output_dir / "dyncompRuntime.png"
    figure.savefig(output, dpi=160)
    print(f"saved plot: {output}")
    if arguments.show:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
