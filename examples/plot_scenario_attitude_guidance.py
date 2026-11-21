#!/usr/bin/env python3
"""Figures of upstream scenarioAttitudeGuidance.py from the Rust CSV."""
from __future__ import annotations

from typing import Any, Sequence

from scenario_plotting import (
    CsvTable,
    _cross,
    _dot,
    _line_color,
    _norm,
    _plot_vector_history,
    _time_minutes,
    _unit,
    _vector_rows,
    run_variant_plot,
)

STEM = "scenarioAttitudeGuidance"
VARIANTS = ("0", "1")


def mrp_to_dcm(sigma: Sequence[float]) -> list[list[float]]:
    """Upstream RigidBodyKinematics.MRP2C."""
    q1, q2, q3 = sigma
    d1 = q1 * q1 + q2 * q2 + q3 * q3
    s = 1.0 - d1
    d = (1.0 + d1) * (1.0 + d1)
    return [
        [(4.0 * (2.0 * q1 * q1 - d1) + s * s) / d, (8.0 * q1 * q2 + 4.0 * q3 * s) / d,
         (8.0 * q1 * q3 - 4.0 * q2 * s) / d],
        [(8.0 * q2 * q1 - 4.0 * q3 * s) / d, (4.0 * (2.0 * q2 * q2 - d1) + s * s) / d,
         (8.0 * q2 * q3 + 4.0 * q1 * s) / d],
        [(8.0 * q3 * q1 + 4.0 * q2 * s) / d, (8.0 * q3 * q2 - 4.0 * q1 * s) / d,
         (4.0 * (2.0 * q3 * q3 - d1) + s * s) / d],
    ]


def plot_attitude_error_norm(plt: Any, time: Sequence[float], table: CsvTable) -> Any:
    """Upstream's |sigma_B/R| on a log axis (also scenarioAttitudeGG figure 1)."""
    figure = plt.figure()
    axis = figure.gca()
    axis.plot(
        time, [_norm(row) for row in _vector_rows(table, "sigma_BR")], color=_line_color(1)
    )
    axis.set_xlabel("Time [min]")
    axis.set_ylabel(r"Attitude Error Norm $|\sigma_{B/R}|$")
    axis.set_yscale("log")
    return figure


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

    projections: list[list[float]] = [[], [], []]
    for position, velocity, sigma in zip(
        _vector_rows(table, "r_BN_N_m"),
        _vector_rows(table, "v_BN_N_m_per_s"),
        _vector_rows(table, "sigma_BN"),
        strict=True,
    ):
        i_r = _unit(position)
        i_h = _unit(_cross(position, velocity))
        i_theta = _cross(i_h, i_r)
        dcm_bn = mrp_to_dcm(sigma)
        for index, axis_vector in enumerate((i_r, i_theta, i_h)):
            projections[index].append(_dot(axis_vector, dcm_bn[index]))
    orientation = plt.figure()
    axis = orientation.gca()
    labels = (
        r"$\hat\imath_r\cdot \hat b_1$",
        r"${\hat\imath}_{\theta}\cdot \hat b_2$",
        r"$\hat\imath_h\cdot \hat b_3$",
    )
    for index, values in enumerate(projections):
        axis.plot(time, values, color=_line_color(index), label=labels[index])
    axis.legend(loc="lower right")
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("Orientation Illustration")

    return {
        f"{STEM}1{variant}": attitude,
        f"{STEM}2{variant}": torque,
        f"{STEM}3{variant}": rate,
        f"{STEM}4{variant}": orientation,
    }


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, VARIANTS, build))
