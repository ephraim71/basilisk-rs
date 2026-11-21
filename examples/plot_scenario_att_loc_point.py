#!/usr/bin/env python3
"""Figures of upstream scenarioAttLocPoint.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from plot_scenario_attitude_guidance import plot_attitude_error_norm
from scenario_plotting import (
    CsvTable,
    _plot_vector_history,
    _time_minutes,
    run_variant_plot,
)

STEM = "scenarioAttLocPoint"


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_minutes(table)

    attitude = plot_attitude_error_norm(plt, time, table)

    torque = plt.figure()
    _plot_vector_history(
        torque.gca(),
        time,
        table,
        "torqueRequestBody_Nm",
        lambda index: rf"$L_{{r,{index}}}$",
        "Control Torque $L_r$ [Nm]",
    )

    # Upstream draws the rate error (figure 3) but returns only figures 1-2.
    rate = plt.figure()
    _plot_vector_history(
        rate.gca(),
        time,
        table,
        "omega_BR_B_rad_per_s",
        lambda index: rf"$\omega_{{BR,{index}}}$",
        "Rate Tracking Error [rad/s] ",
    )

    return {
        f"{STEM}1": attitude,
        f"{STEM}2": torque,
        f"{STEM}3": rate,
    }


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, ("",), build))
