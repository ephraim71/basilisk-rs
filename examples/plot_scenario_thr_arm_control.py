#!/usr/bin/env python3
"""Figures of upstream scenarioThrArmControl.py from the Rust CSV file.

Upstream ``pull_outputs``: the system centre-of-mass position (the
regulation's position error, the target being the origin) and the hub
attitude ``sigma_BN`` over time in minutes.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

from orbit_plotting import NANO2MIN, Table, line_color, run_plots

STEM = "scenarioThrArmControl"


def plot_xyz(plt: Any, time: Any, values: Any, labels: list[str], ylabel: str) -> Any:
    figure = plt.figure()
    for index, label in enumerate(labels):
        plt.plot(time, values[:, index], color=line_color(index, 3, plt), label=label)
    plt.xlabel("Time [min]")
    plt.ylabel(ylabel)
    plt.legend()
    return figure


def build(generated_dir: Path, plt: Any) -> dict[str, Any]:
    table = Table(generated_dir / f"{STEM}.csv")
    time = table.time_ns * NANO2MIN
    return {
        f"{STEM}_position": plot_xyz(
            plt, time, table.vectors("r_CN_N_m"), ["x", "y", "z"], "Position Error [m]"
        ),
        f"{STEM}_attitude": plot_xyz(
            plt,
            time,
            table.vectors("sigma_BN"),
            [r"$\sigma_1$", r"$\sigma_2$", r"$\sigma_3$"],
            r"Attitude Error [$\boldsymbol{\sigma}$]",
        ),
    }


if __name__ == "__main__":
    raise SystemExit(run_plots(f"Plot {STEM} results.", build))
