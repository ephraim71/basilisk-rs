#!/usr/bin/env python3
"""Figures of upstream scenarioFormationBasic.py from the Rust CSV.

Upstream saves the servicer's attitude error (1), its requested and applied
wheel torques (2) and its wheel speeds (3); it also shows the rate tracking
error and the debris's free-spinning wheel speeds, saved here as 4 and 5.
"""
from __future__ import annotations

from typing import Any

from scenario_plotting import (
    RPM_RAD_PER_S,
    CsvTable,
    _line_color,
    _plot_attitude_error,
    _plot_motor_torque,
    _plot_vector_history,
    _plot_wheel_speeds,
    _scaled,
    _time_minutes,
    _vector_columns,
    run_variant_plot,
)

STEM = "scenarioFormationBasic"


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_minutes(table)

    attitude = plt.figure()
    _plot_attitude_error(attitude.gca(), time, table)

    torque = plt.figure()
    _plot_motor_torque(torque.gca(), time, table, 3, grid=False)

    speeds = plt.figure()
    _plot_wheel_speeds(speeds.gca(), time, table, 3, grid=False)

    rate = plt.figure()
    _plot_vector_history(
        rate.gca(),
        time,
        table,
        "omega_BR_B_rad_per_s",
        lambda index: rf"$\omega_{{BR,{index}}}$",
        "Rate Tracking Error (rad/s) ",
    )

    debris = plt.figure()
    axis = debris.gca()
    for index, values in enumerate(_vector_columns(table, "debrisWheelSpeeds_rad_per_s", 2)):
        axis.plot(
            time,
            _scaled(values, 1.0 / RPM_RAD_PER_S),
            color=_line_color(index),
            label=rf"$\Omega_{{s,{index}}}$",
        )
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("RW2 Omega (rpm)")

    return {
        f"{STEM}1": attitude,
        f"{STEM}2": torque,
        f"{STEM}3": speeds,
        f"{STEM}4": rate,
        f"{STEM}5": debris,
    }


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, ("",), build))
