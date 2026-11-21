#!/usr/bin/env python3
"""Figures of upstream scenarioAttitudeSteering.py from the Rust CSV."""
from __future__ import annotations

import math
from typing import Any

from scenario_plotting import (
    CsvTable,
    _line_color,
    _plot_wheel_speeds,
    _time_minutes,
    _vector_columns,
    run_variant_plot,
)

STEM = "scenarioAttitudeSteering"
VARIANTS = ("0", "1", "2", "3")


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_minutes(table)

    attitude = plt.figure()
    axis = attitude.gca()
    for index, values in enumerate(_vector_columns(table, "sigma_BR", 3)):
        axis.semilogy(
            time,
            [abs(value) for value in values],
            color=_line_color(index),
            label=rf"$|\sigma_{index}|$",
        )
    axis.legend(loc="upper right")
    axis.set_xlabel("Time [min]")
    axis.set_ylabel(r"Attitude Error $\sigma_{B/R}$")

    torque = plt.figure()
    axis = torque.gca()
    for index, values in enumerate(_vector_columns(table, "rwTorque_Nm", 3)):
        axis.semilogy(
            time,
            [abs(value) for value in values],
            color=_line_color(index),
            label=rf"$|u_{{s,{index}}}|$",
        )
    axis.legend(loc="lower right")
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("RW Motor Torque (Nm)")

    rate = plt.figure()
    axis = rate.gca()
    degrees = 180.0 / math.pi
    for index, values in enumerate(_vector_columns(table, "omega_BR_B_rad_per_s", 3)):
        axis.semilogy(
            time,
            [abs(value) * degrees for value in values],
            color=_line_color(index),
            label=rf"$|\omega_{{BR,{index}}}|$",
        )
    for index, values in enumerate(_vector_columns(table, "omega_BastR_B_rad_per_s", 3)):
        axis.semilogy(
            time, [abs(value) * degrees for value in values], "--", color=_line_color(index)
        )
    axis.legend(loc="upper right")
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("Rate Tracking Error (deg/s) ")

    speed = plt.figure()
    axis = speed.gca()
    _plot_wheel_speeds(axis, time, table, 3, grid=False)
    axis.legend(loc="upper right")

    return {
        f"{STEM}SigmaBR{variant}": attitude,
        f"{STEM}rwUs{variant}": torque,
        f"{STEM}omegaBR{variant}": rate,
        f"{STEM}Omega{variant}": speed,
    }


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, VARIANTS, build))
