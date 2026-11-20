#!/usr/bin/env python3
"""Figures of upstream scenarioAttitudeFeedbackNumba.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from scenario_plotting import (
    CsvTable,
    _plot_attitude_error,
    _plot_vector_history,
    _time_minutes,
    run_variant_plot,
)

STEM = "scenarioAttitudeFeedbackNumba"
VARIANTS = ("000", "100", "110", "101")


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_minutes(table)

    attitude = plt.figure()
    _plot_attitude_error(attitude.gca(), time, table)

    torque = plt.figure()
    _plot_vector_history(
        torque.gca(),
        time,
        table,
        "torqueRequestBody_Nm",
        lambda index: rf"$L_{{r,{index}}}$",
        r"Control Torque $L_r$ [Nm]",
    )

    return {f"{STEM}1{variant}": attitude, f"{STEM}2{variant}": torque}


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, VARIANTS, build))
