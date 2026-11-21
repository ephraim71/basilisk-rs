#!/usr/bin/env python3
"""Figures of upstream scenarioVizPoint.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from scenario_plotting import CsvTable, _plot_vector_history, _time_minutes, run_variant_plot

STEM = "scenarioVizPoint"
VARIANTS = ("_marsOrbit", "_dscovr")


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_minutes(table)
    figures = {}
    for number, (prefix, label, ylabel) in enumerate(
        (
            ("sigma_BR", lambda index: rf"$\sigma_{index}$", r"Attitude Error $\sigma_{B/R}$"),
            ("torqueRequestBody_Nm", lambda index: rf"$L_{{r,{index}}}$", "Control Torque $L_r$ [Nm]"),
            ("omega_BR_B_rad_per_s", lambda index: rf"$\omega_{{BR,{index}}}$", "Rate Tracking Error [rad/s] "),
        ),
        start=1,
    ):
        figure = plt.figure()
        _plot_vector_history(figure.gca(), time, table, prefix, label, ylabel)
        figures[f"{STEM}{number}{variant}"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, VARIANTS, build))
