#!/usr/bin/env python3
"""Figures of upstream scenarioCSSFilters.py from the Rust CSV.

The expected states are computed as upstream's script does: the true Sun
heading `sHat_B` (and its rate for uKF/EKF); for SEKF the rates from the
filter's own state through `[BS]`, for SuKF the true body rate in the S frame.
"""
from __future__ import annotations

import math
from typing import Any

from scenario_plotting import CsvTable, _cross, _line_color, _norm, _time_seconds, run_variant_plot

STEM = "scenarioCSSFilters"
VARIANTS = ("SuKF", "uKF", "EKF", "OEKF", "SEKF")
STATES = {"uKF": 6, "EKF": 6, "OEKF": 3, "SEKF": 5, "SuKF": 6}
Q_OBS_SIGMA = 0.017


def _mrp_to_dcm(sigma: list[float]) -> list[list[float]]:
    s1, s2, s3 = sigma
    q = s1 * s1 + s2 * s2 + s3 * s3
    d = (1.0 + q) ** 2
    rows = [
        [4 * (2 * s1 * s1 - q) + (1 - q) ** 2, 8 * s1 * s2 + 4 * s3 * (1 - q), 8 * s1 * s3 - 4 * s2 * (1 - q)],
        [8 * s2 * s1 - 4 * s3 * (1 - q), 4 * (2 * s2 * s2 - q) + (1 - q) ** 2, 8 * s2 * s3 + 4 * s1 * (1 - q)],
        [8 * s3 * s1 + 4 * s2 * (1 - q), 8 * s3 * s2 - 4 * s1 * (1 - q), 4 * (2 * s3 * s3 - q) + (1 - q) ** 2],
    ]
    return [[value / d for value in row] for row in rows]


def _dcm_bs(heading: list[float], b_vec: list[float]) -> list[list[float]]:
    """Upstream `sunlineSEKFComputeDCM_BS`: columns s1, s2, s3."""
    s2 = _cross(heading, b_vec)
    if _norm(s2) < 1e-5:
        return [[1.0 if row == column else 0.0 for column in range(3)] for row in range(3)]
    s1 = [value / _norm(heading) for value in heading]
    s2 = [value / _norm(s2) for value in s2]
    s3 = _cross(heading, s2)
    s3 = [value / _norm(s3) for value in s3]
    return [[s1[row], s2[row], s3[row]] for row in range(3)]


def _row_vector(table: CsvTable, prefix: str, row: int, count: int = 3) -> list[float]:
    return [table.rows[row][f"{prefix}_{index}"] for index in range(count)]


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_seconds(table)
    n = STATES[variant]
    states = [[table.rows[row][f"state_{i}"] for i in range(n)] for row in range(len(time))]
    expected = []
    for row in range(len(time)):
        to_sun_n = [
            table.rows[row][f"sunPosition_N_m_{k}"] - table.rows[row][f"r_BN_N_m_{k}"]
            for k in range(3)
        ]
        magnitude = _norm(to_sun_n)
        dcm_bn = _mrp_to_dcm(_row_vector(table, "sigma_BN", row))
        s_hat_b = [sum(dcm_bn[i][j] * to_sun_n[j] / magnitude for j in range(3)) for i in range(3)]
        omega = _row_vector(table, "omega_BN_B_rad_per_s", row)
        value = list(s_hat_b) + [0.0] * (n - 3)
        if variant in ("uKF", "EKF"):
            value[3:6] = [-component for component in _cross(omega, s_hat_b)]
        elif variant in ("SEKF", "SuKF"):
            dcm = _dcm_bs(states[row][:3], _row_vector(table, "bVec_B", row))
            if variant == "SEKF":
                rate = [0.0, states[row][3], states[row][4]]
                omega_exp = [-sum(dcm[i][j] * rate[j] for j in range(3)) for i in range(3)]
            else:
                omega_exp = [sum(dcm[j][i] * omega[j] for j in range(3)) for i in range(3)]
            value[3], value[4] = omega_exp[1], omega_exp[2]
        expected.append(value)

    vs_expected = plt.figure(figsize=(10, 8))
    errors = plt.figure(figsize=(10, 8))
    for index in range(n):
        sigma3 = [
            3.0 * math.sqrt(max(table.rows[row][f"covar_{index * n + index}"], 0.0))
            for row in range(len(time))
        ]
        axis = vs_expected.add_subplot((n + 1) // 2, 2, index + 1)
        axis.plot(time, [state[index] for state in states], color=_line_color(0), label="state")
        axis.plot(time, [value[index] for value in expected], "--", color=_line_color(1), label="expected")
        axis.set_ylabel(f"state {index}")
        axis = errors.add_subplot((n + 1) // 2, 2, index + 1)
        error = [states[row][index] - expected[row][index] for row in range(len(time))]
        axis.plot(time, error, color=_line_color(0))
        axis.plot(time, sigma3, "--", color=_line_color(3))
        axis.plot(time, [-value for value in sigma3], "--", color=_line_color(3))
        axis.set_ylabel(f"error {index}")
    vs_expected.gca().legend(loc="lower right")

    residuals = plt.figure()
    axis = residuals.gca()
    for index in range(8):
        axis.plot(time, table.column(f"postFitRes_{index}"), ".", markersize=2,
                  color=_line_color(index))
    for bound in (3 * Q_OBS_SIGMA, -3 * Q_OBS_SIGMA):
        axis.plot(time, [bound] * len(time), "--", color="r")
    axis.set_xlabel("Time [s]")
    axis.set_ylabel("Post-fit residuals")

    observations = plt.figure()
    axis = observations.gca()
    axis.plot(time, table.column("numObs"), ".")
    axis.set_xlabel("Time [s]")
    axis.set_ylabel("Number of measurements")
    return {
        f"scenario_Filters_StatesPlot{variant}": errors,
        f"scenario_Filters_StatesExpected{variant}": vs_expected,
        f"scenario_Filters_PostFit{variant}": residuals,
        f"scenario_Filters_Obs{variant}": observations,
    }


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, VARIANTS, build))
