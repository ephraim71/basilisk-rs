#!/usr/bin/env python3
"""Figures of upstream scenarioThrArmTracking.py from the Rust CSV files.

Upstream ``pull_outputs``: the position tracking error (system centre of mass
less the ``arcLineHoldRef`` reference) and the attitude tracking error
``sigma_BR`` of ``locationPointing`` over time in hours, for each option
combination whose CSV exists (``scenarioThrArmTracking``,
``..._jointProfiler``, ``..._motionPenalty``,
``..._jointProfiler_motionPenalty``).
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

from orbit_plotting import NANO2HOUR, existing, line_color, run_plots

STEM = "scenarioThrArmTracking"
VARIANTS = ["", "_jointProfiler", "_motionPenalty", "_jointProfiler_motionPenalty"]


def plot_xyz(plt: Any, time: Any, values: Any, labels: list[str], ylabel: str) -> Any:
    figure = plt.figure()
    for index, label in enumerate(labels):
        plt.plot(time, values[:, index], color=line_color(index, 3, plt), label=label)
    plt.xlabel("Time [hr]")
    plt.ylabel(ylabel)
    plt.legend()
    return figure


def build(generated_dir: Path, plt: Any) -> dict[str, Any]:
    figures: dict[str, Any] = {}
    for stem, table in existing(generated_dir, [STEM + variant for variant in VARIANTS]):
        time = table.time_ns * NANO2HOUR
        error = table.vectors("r_CN_N_m") - table.vectors("r_RN_N_m")
        figures[f"{stem}_position"] = plot_xyz(
            plt, time, error, ["x", "y", "z"], "Position Tracking Error [m]"
        )
        figures[f"{stem}_attitude"] = plot_xyz(
            plt,
            time,
            table.vectors("sigma_BR"),
            [r"$\sigma_1$", r"$\sigma_2$", r"$\sigma_3$"],
            r"Attitude Tracking Error [$\boldsymbol{\sigma}_{B/R}$]",
        )
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(f"Plot {STEM} results.", build))
