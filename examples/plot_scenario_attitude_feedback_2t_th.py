#!/usr/bin/env python3
"""Figures of upstream scenarioAttitudeFeedback2T_TH.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from scenario_plotting import (
    CsvTable,
    _plot_attitude_error,
    _plot_vector_history,
    _time_minutes,
    run_variant_plot,
)

STEM = "scenarioAttitudeFeedback2T_TH"
VARIANTS = ("0", "1")


def thruster_count(table: CsvTable) -> int:
    return sum(header.startswith("thrForce_N_") for header in table.headers)


def build_figures(plt: Any, table: CsvTable, variant: str, stem: str) -> dict[str, Any]:
    """Upstream figures 1-5, shared with scenarioAttitudeFeedback2T_stateEffTH."""
    time = _time_minutes(table)
    count = thruster_count(table)

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

    attitude = plt.figure()
    _plot_attitude_error(attitude.gca(), time, table)

    force = plt.figure()
    _plot_vector_history(
        force.gca(),
        time,
        table,
        "thrForce_N",
        lambda index: rf"$thrForce_{{{index}}}$",
        "Force requested [N]",
        count=count,
    )

    on_time = plt.figure()
    _plot_vector_history(
        on_time.gca(),
        time,
        table,
        "OnTimeRequest_s",
        lambda index: rf"$OnTimeRequest_{{{index}}}$",
        "OnTimeRequest [sec]",
        count=count,
    )

    return {
        f"{stem}1{variant}": torque,
        f"{stem}2{variant}": rate,
        f"{stem}3{variant}": attitude,
        f"{stem}4{variant}": force,
        f"{stem}5{variant}": on_time,
    }


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    return build_figures(plt, table, variant, STEM)


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, VARIANTS, build))
