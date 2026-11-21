#!/usr/bin/env python3
"""Figures of upstream OpNavMC/MonteCarlo.py's data callback ``displayPlots``
from the Rust CSVs: every run's position and velocity errors and their
3-sigma covariance bounds, in percent, at the valid measurements. The Rust
campaign writes them per run to ``<scenario>_run<i>.csv``; ``--scenario``
picks ``scenario_OpNavAttODMC`` (default) or ``scenario_LimbAttOD``."""
from __future__ import annotations

import re
import sys
from pathlib import Path
from typing import Any

from orbit_plotting import NANO2MIN, Table, run_plots

SCENARIOS = ("scenario_OpNavAttODMC", "scenario_LimbAttOD")


def mc_figures(generated_dir: Path, plt: Any, stem: str) -> dict[str, Any]:
    pattern = re.compile(rf"^{stem}_run(\d+)\.csv$")
    runs = sorted(
        (int(match.group(1)), path)
        for path in generated_dir.glob(f"{stem}_run*.csv")
        if (match := pattern.match(path.name))
    )
    if not runs:
        return {}
    figures = {
        "MCErrorPos": (101, "diffPos_percent", r"$\mathbf{r}_\mathrm{Circ}$ errors ($\%$)", "-"),
        "MCCovarPos": (103, "covarPos_percent", r"$\mathbf{r}_\mathrm{Circ}$ covar ($\%$)", "--"),
        "MCErrorVel": (102, "diffVel_percent", r"$\dot{\mathbf{r}}_\mathrm{Circ}$ errors ($\%$)", "-"),
        "MCCovarVel": (104, "covarVel_percent", r"$\dot{\mathbf{r}}_\mathrm{Circ}$ covar ($\%$)", "--"),
    }
    result = {}
    for name, (number, column, ylabel, style) in figures.items():
        figure = plt.figure(number, figsize=(3.5, 2), facecolor="w", edgecolor="k")
        for _, path in runs:
            table = Table(path)
            plt.plot(table.time_ns * NANO2MIN, table[column], linestyle=style)
        plt.ylabel(ylabel)
        plt.xlabel("Time (min)")
        result[f"{stem}_{name}"] = figure
    return result


def scenario_from_argv() -> str:
    """``--scenario <name>``, removed from ``sys.argv`` before
    ``run_plots`` parses it."""
    if "--scenario" in sys.argv:
        index = sys.argv.index("--scenario")
        stem = sys.argv[index + 1]
        del sys.argv[index:index + 2]
        if stem not in SCENARIOS:
            raise SystemExit(f"--scenario must be one of {SCENARIOS}")
        return stem
    return SCENARIOS[0]


if __name__ == "__main__":
    STEM = scenario_from_argv()
    raise SystemExit(run_plots(f"Plot {STEM} Monte Carlo results.",
                               lambda generated_dir, plt: mc_figures(generated_dir, plt, STEM)))
