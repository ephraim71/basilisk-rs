#!/usr/bin/env python3
"""Figures of upstream scenarioAttitudePointing.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from scenario_plotting import (
    CsvTable,
    _plot_attitude_error,
    _plot_vector_history,
    _time_minutes,
    run_variant_plot,
)

STEM = "scenarioAttitudePointing"
VARIANTS = ("0", "1")


def build_figures(plt: Any, table: CsvTable, variant: str, stem: str) -> dict[str, Any]:
    """Upstream figures 1-3, shared with the Py and Numba variants."""
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
        f"{stem}1{variant}": attitude,
        f"{stem}2{variant}": torque,
        f"{stem}3{variant}": rate,
    }


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    return build_figures(plt, table, variant, STEM)


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, VARIANTS, build))
