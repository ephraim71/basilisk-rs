#!/usr/bin/env python3
"""Figures of upstream scenarioMomentumDumping.py from the Rust CSV."""
from __future__ import annotations

import math
from typing import Any

from scenario_plotting import (
    CsvTable,
    _line_color,
    _plot_vector_history,
    _time_seconds,
    _vector_columns,
    run_variant_plot,
)

STEM = "scenarioMomentumDumping"
# Honeywell HR16, 100 N m s at 6000 RPM: Js = maxMomentum / Omega_max.
WHEEL_JS = 100.0 / (6000.0 * 2.0 * math.pi / 60.0)
C = 2.0**-0.5
WHEEL_AXES = ((C, 0.0, C), (0.0, C, C), (-C, 0.0, C), (0.0, -C, C))


def _history(plt: Any, time: list[float], table: CsvTable, prefix: str, count: int,
             label: str, ylabel: str) -> Any:
    figure = plt.figure()
    axis = figure.gca()
    for index, values in enumerate(_vector_columns(table, prefix, count)):
        axis.plot(time, values, color=_line_color(index), label=label.format(index + 1))
    axis.legend(loc="lower right")
    axis.set_xlabel("Time [sec]")
    axis.set_ylabel(ylabel)
    return figure


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_seconds(table)

    attitude = plt.figure()
    _plot_vector_history(
        attitude.gca(), time, table, "sigma_BR", lambda i: rf"$\sigma_{i}$",
        r"Attitude Error $\sigma_{B/R}$",
    )
    attitude.gca().set_xlabel("Time [sec]")
    rate = plt.figure()
    _plot_vector_history(
        rate.gca(), time, table, "omega_BR_B_rad_per_s",
        lambda i: rf"$\omega_{{BR,{i + 1}}}$", "Rate Tracking Error [rad/s] ",
    )
    rate.gca().set_xlabel("Time [sec]")

    momenta = plt.figure()
    axis = momenta.gca()
    speeds = _vector_columns(table, "wheelSpeeds_rad_per_s", 4)
    for index, values in enumerate(speeds):
        axis.plot(time, [WHEEL_JS * value for value in values], color=_line_color(index),
                  label=rf"$H_{{{index + 1}}}$")
    total = []
    for row in range(len(time)):
        h = [sum(WHEEL_JS * speeds[i][row] * WHEEL_AXES[i][k] for i in range(4)) for k in range(3)]
        total.append(math.sqrt(sum(component * component for component in h)))
    axis.plot(time, total, "--", label=r"$\|H\|$")
    axis.legend(loc="lower right")
    axis.set_xlabel("Time [sec]")
    axis.set_ylabel("RW Momentum (Nms)")

    delta_h = _history(plt, time, table, "deltaH_Nms", 3, r"$\Delta H_{{{}}}$",
                       "Dumped momentum (Nms) ")
    impulse = _history(plt, time, table, "thrForce_N", 8, r"$thrImpulse_{{{}}}$",
                       "Thruster impulse request [Ns]")
    on_time = _history(plt, time, table, "OnTimeRequest_s", 8, r"$OnTimeRequest_{{{}}}$",
                       "OnTimeRequest [sec]")
    force = _history(plt, time, table, "thrustForce_N", 8, r"$thrForce_{{{}}}$",
                     "Thruster force [N]")
    return {
        f"{STEM}1": attitude,
        f"{STEM}2": rate,
        f"{STEM}3": momenta,
        f"{STEM}4": delta_h,
        f"{STEM}5": impulse,
        f"{STEM}6": on_time,
        f"{STEM}7": force,
    }


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, ("",), build))
