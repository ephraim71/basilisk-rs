#!/usr/bin/env python3
"""Figures of upstream scenarioInertialSpiral.py from the Rust CSV."""
from __future__ import annotations

import math
from typing import Any, Sequence

from plot_scenario_attitude_guidance import mrp_to_dcm
from scenario_plotting import (
    CsvTable,
    _line_color,
    _plot_vector_history,
    _time_minutes,
    _vector_rows,
    run_variant_plot,
)

STEM = "scenarioInertialSpiral"


def mrp_to_euler321(sigma: Sequence[float]) -> tuple[float, float, float]:
    """Upstream RigidBodyKinematics.MRP2Euler321 (via C2Euler321)."""
    c = mrp_to_dcm(sigma)
    return (
        math.atan2(c[0][1], c[0][0]),
        math.asin(max(-1.0, min(1.0, -c[0][2]))),
        math.atan2(c[1][2], c[2][2]),
    )


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_minutes(table)

    attitude = plt.figure()
    _plot_vector_history(
        attitude.gca(),
        time,
        table,
        "sigma_BR",
        lambda index: rf"$\sigma_{index}$",
        r"Attitude Error $\sigma_{B/R}$",
    )

    torque = plt.figure()
    _plot_vector_history(
        torque.gca(),
        time,
        table,
        "torqueRequestBody_Nm",
        lambda index: rf"$L_{{r,{index}}}$",
        r"Control Torque $L_r$ [Nm]",
        legend="upper right",
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

    # Upstream labels the first 3-2-1 angle "Pitch" and the second "Yaw".
    angles = [mrp_to_euler321(sigma) for sigma in _vector_rows(table, "sigma_BN")]
    pitch = [angle[0] for angle in angles]
    yaw = [angle[1] for angle in angles]

    euler = plt.figure()
    axis = euler.gca()
    axis.plot(time, yaw, color=_line_color(0), label="Yaw")
    axis.plot(time, pitch, color=_line_color(1), label="Pitch")
    axis.legend(loc="lower right")
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("Euler Angles [rad]")

    spiral = plt.figure()
    axis = spiral.gca()
    axis.plot(pitch, yaw)
    axis.set_xlabel("Pitch [rad]")
    axis.set_ylabel("Yaw [rad]")

    return {
        f"{STEM}1": attitude,
        f"{STEM}2": torque,
        f"{STEM}3": rate,
        f"{STEM}4": euler,
        f"{STEM}5": spiral,
    }


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, ("",), build))
