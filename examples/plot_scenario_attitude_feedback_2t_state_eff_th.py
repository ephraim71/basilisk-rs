#!/usr/bin/env python3
"""Figures of upstream scenarioAttitudeFeedback2T_stateEffTH.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from plot_scenario_attitude_feedback_2t_th import build_figures, thruster_count
from scenario_plotting import (
    CsvTable,
    _plot_vector_history,
    _time_minutes,
    run_variant_plot,
)

STEM = "scenarioAttitudeFeedback2T_stateEffTH"
VARIANTS = ("0", "1")


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    figures = build_figures(plt, table, variant, STEM)

    delivered = plt.figure()
    _plot_vector_history(
        delivered.gca(),
        _time_minutes(table),
        table,
        "thrustForce_N",
        lambda index: rf"$thrForce_{{{index}}}$",
        "Force implemented[N]",
        count=thruster_count(table),
    )
    figures[f"{STEM}6{variant}"] = delivered
    return figures


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, VARIANTS, build))
