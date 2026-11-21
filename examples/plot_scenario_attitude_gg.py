#!/usr/bin/env python3
"""Figures of upstream scenarioAttitudeGG.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from plot_scenario_attitude_guidance import plot_attitude_error_norm
from scenario_plotting import (
    CsvTable,
    _line_color,
    _norm,
    _plot_vector_history,
    _time_minutes,
    _vector_rows,
    run_variant_plot,
)

STEM = "scenarioAttitudeGG"


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_minutes(table)

    attitude = plot_attitude_error_norm(plt, time, table)

    torque = plt.figure()
    axis = torque.gca()
    axis.plot(
        time,
        [_norm(row) for row in _vector_rows(table, "torqueRequestBody_Nm")],
        color=_line_color(1),
    )
    axis.set_xlabel("Time [min]")
    axis.set_ylabel(r"Control Torque $L_r$ [Nm]")
    axis.set_yscale("log")

    gravity_gradient = plt.figure()
    _plot_vector_history(
        gravity_gradient.gca(),
        time,
        table,
        "gravityGradientTorque_B_Nm",
        lambda index: rf"$r_{index}$",
        "GG Torque [Nm]",
    )

    return {
        f"{STEM}1{variant}": attitude,
        f"{STEM}2{variant}": torque,
        f"{STEM}3{variant}": gravity_gradient,
    }


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, ("",), build))
