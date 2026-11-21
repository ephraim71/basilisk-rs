#!/usr/bin/env python3
"""Figures of upstream scenarioAttitudeFeedback2T.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from scenario_plotting import (
    CsvTable,
    _plot_attitude_error,
    _plot_vector_history,
    _time_minutes,
    run_variant_plot,
)

STEM = "scenarioAttitudeFeedback2T"
VARIANTS = ("00", "10", "11")


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

    rate = plt.figure()
    _plot_vector_history(
        rate.gca(),
        time,
        table,
        "omega_BR_B_rad_per_s",
        lambda index: rf"$\omega_{{BR,{index}}}$",
        "Rate Tracking Error [rad/s] ",
    )

    position = plt.figure()
    _plot_vector_history(
        position.gca(),
        time,
        table,
        "r_BN_N_m",
        lambda index: rf"$r_{{BN,{index}}}$",
        "Inertial Position [km] ",
        scale=1.0e-3,
    )

    inertial_attitude = plt.figure()
    _plot_vector_history(
        inertial_attitude.gca(),
        time,
        table,
        "sigma_BN",
        lambda index: rf"$\sigma_{{BN,{index}}}$",
        "Inertial MRP Attitude ",
    )

    return {
        f"{STEM}1{variant}": attitude,
        f"{STEM}2{variant}": torque,
        f"{STEM}3{variant}": rate,
        f"{STEM}4{variant}": position,
        f"{STEM}5{variant}": inertial_attitude,
    }


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, VARIANTS, build))
