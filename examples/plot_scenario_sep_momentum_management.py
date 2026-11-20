#!/usr/bin/env python3
"""Figures of upstream scenarioSepMomentumManagement.py from the Rust CSV.

Figure numbers follow upstream's. The SRP torque figures (7 and 9) are not
reproduced: the Rust SRP effector publishes no force or torque.
"""
from __future__ import annotations

import math
from typing import Any

from scenario_plotting import (
    CsvTable,
    _cross,
    _dot,
    _line_color,
    _norm,
    _time_hours,
    _vector_columns,
    _vector_rows,
    run_variant_plot,
)

STEM = "scenarioSepMomentumManagement"
VARIANTS = ("0101", "0000")


def _euler123_to_dcm(theta1: float, theta2: float) -> list[list[float]]:
    """Upstream `rbk.euler1232C([theta1, theta2, 0])`."""
    c1, s1 = math.cos(theta1), math.sin(theta1)
    c2, s2 = math.cos(theta2), math.sin(theta2)
    return [[c2, s1 * s2, -c1 * s2], [0.0, c1, s1], [s2, -s1 * c2, c1 * c2]]


def _lines(plt: Any, time: list[float], series: list[tuple[list[float], str, str]],
           ylabel: str) -> Any:
    figure = plt.figure()
    axis = figure.gca()
    for index, (values, label, style) in enumerate(series):
        axis.plot(time, values, style, color=_line_color(index % 3 if len(series) > 4 else index),
                  label=label)
    axis.legend(loc="lower right")
    axis.set_xlabel("Time [hours]")
    axis.set_ylabel(ylabel)
    axis.grid(True)
    return figure


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_hours(table)
    suffix = variant[1] + variant[3]
    figures: dict[str, Any] = {}

    sigma_bn = _vector_columns(table, "sigma_BN", 3)
    sigma_rn = _vector_columns(table, "sigma_RN", 3)
    figures[f"{STEM}1{suffix}"] = _lines(
        plt, time,
        [(sigma_bn[i], rf"$\sigma_{{BN,{i + 1}}}$", "-") for i in range(3)]
        + [(sigma_rn[i], rf"$\sigma_{{RN,{i + 1}}}$", "--") for i in range(3)],
        r"Attitude $\sigma$",
    )
    sigma_br = _vector_columns(table, "sigma_BR", 3)
    figures[f"{STEM}2{suffix}"] = _lines(
        plt, time, [(sigma_br[i], rf"$\sigma_{i + 1}$", "-") for i in range(3)],
        r"Attitude Tracking Error $\sigma_{B/R}$",
    )
    speeds = _vector_columns(table, "wheelSpeeds_rad_per_s", 4)
    figures[f"{STEM}3{suffix}"] = _lines(
        plt, time,
        [([value * 60.0 / (2.0 * math.pi) for value in speeds[i]], rf"$\Omega_{{{i + 1}}}$", "-")
         for i in range(4)],
        "RW Speed (RPM) ",
    )
    figures[f"{STEM}4{suffix}"] = _lines(
        plt, time,
        [([math.degrees(v) for v in table.column(f"saAngle_rad_{i}")], rf"$\alpha_{{{i + 1}}}$", "-")
         for i in range(2)]
        + [([math.degrees(v) for v in table.column(f"saRefAngle_rad_{i}")],
            rf"$\alpha_{{R,{i + 1}}}$", "--") for i in range(2)],
        "Solar Array Angles [deg]",
    )
    nu = [table.column(f"pltAngle_rad_{i}") for i in range(2)]
    figures[f"{STEM}5{suffix}"] = _lines(
        plt, time,
        [([math.degrees(v) for v in nu[i]], rf"$\nu_{{{i + 1}}}$", "-") for i in range(2)]
        + [([math.degrees(v) for v in table.column(f"pltRefAngle_rad_{i}")],
            rf"$\nu_{{R,{i + 1}}}$", "--") for i in range(2)],
        "Platform Angles [deg]",
    )

    # Thrust direction offset from the center of mass, and the thruster
    # torque about it (upstream `plot_thruster_cm_offset`, `plot_thr_torque`).
    cm = _vector_rows(table, "CoM_B_m")
    location_f = _vector_rows(table, "thrLoc_F_m")
    direction_f = _vector_rows(table, "thrDir_F")
    thrust = table.column("thrustForce_N")
    offsets, torques = [], []
    for row in range(len(time)):
        dcm_fb = _euler123_to_dcm(nu[0][row], nu[1][row])
        to_b = lambda v: [sum(dcm_fb[j][i] * v[j] for j in range(3)) for i in range(3)]
        r_tm_b = to_b(location_f[row])
        r_ct_b = [cm[row][i] - r_tm_b[i] for i in range(3)]
        direction_b = to_b(direction_f[row])
        angle = math.acos(min(max(_dot(r_ct_b, direction_b) / _norm(r_ct_b), -1.0), 1.0))
        cross = _cross(r_ct_b, direction_b)
        offsets.append(math.degrees(-angle if math.atan2(cross[1], cross[0]) < 0 else angle))
        torques.append(_cross([-value for value in r_ct_b], [thrust[row] * d for d in direction_b]))
    figures[f"{STEM}6{suffix}"] = _lines(
        plt, time, [(offsets, r"$\Delta \theta$", "-")], "Thrust-CM offset [deg]"
    )
    figures[f"{STEM}8{suffix}"] = _lines(
        plt, time,
        [([torque[i] for torque in torques], rf"${{}}^BL_{i + 1}$", "-") for i in range(3)],
        "Thruster Torque [Nm]",
    )
    figures[f"{STEM}10{suffix}"] = _lines(
        plt, time,
        [([math.degrees(v) for v in table.column(f"saMissAngle_rad_{i}")],
          rf"$\beta_{{{i + 1}}}$", "-") for i in range(2)],
        "Solar Array Pointing Error [deg]",
    )
    figures[f"{STEM}11{suffix}"] = _lines(
        plt, time, [([math.degrees(v) for v in table.column("hubMissAngle_rad")], r"$\gamma$", "-")],
        "Sensitive Platform Pointing [deg]",
    )
    if variant[3] == "1":
        errors = _vector_columns(table, "stateError_m", 3)
        covariance = _vector_columns(table, "covariance_m", 3)
        figures[f"{STEM}12{suffix}"] = _lines(
            plt, time,
            [(errors[i], rf"$\Delta r_{i + 1}$", "-") for i in range(3)]
            + [([3.0 * math.sqrt(max(v, 0.0)) for v in covariance[i]],
                rf"$3\sigma_{i + 1}$", "--") for i in range(3)],
            "CM estimation error [m]",
        )
        pre = _vector_columns(table, "preFitRes_Nm", 3)
        post = _vector_columns(table, "postFitRes_Nm", 3)
        figures[f"{STEM}13{suffix}"] = _lines(
            plt, time,
            [(pre[i], rf"pre-fit$_{i + 1}$", ".") for i in range(3)]
            + [(post[i], rf"post-fit$_{i + 1}$", ".") for i in range(3)],
            "Residuals [Nm]",
        )
    return figures


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, VARIANTS, build))
