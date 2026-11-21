#!/usr/bin/env python3
"""Figures of upstream scenario_StationKeepingMultiSat.py from the Rust CSVs
(``scenario_StationKeepingMultiSat<True|False>_sat<i>.csv`` and, with the
barycenter chief, ``..._chief.csv``): upstream's 14 figures of one
spacecraft, by default spacecraft 1 as upstream's
``pull_outputs(showPlots, relativeNavigation, 1)``
(``MULTI_SAT_SPACECRAFT`` selects another), for each chief the example ran."""
from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Any

import numpy as np

from formation_plotting import E2M, f2E
from multi_sat_plotting import (
    NANO2MIN,
    NANO2SEC,
    columns,
    plot_attitude,
    plot_attitude_error,
    plot_attitude_reference,
    plot_fuel,
    plot_orbital_element_differences,
    plot_orbits,
    plot_power,
    plot_rate,
    plot_rate_error,
    plot_rate_reference,
    plot_relative_orbits,
    plot_rw_motor_torque,
    plot_rw_speeds,
    plot_thrust_percentage,
    spacecraft_tables,
)
from orbit_plotting import MU_EARTH, REQ_EARTH, Table, existing, rv2elem, run_plots

STEM = "scenario_StationKeepingMultiSat"
# Upstream `oe[i].a` of every spacecraft: 1.4 Earth radii.
SEMI_MAJOR_AXIS = 1.4 * REQ_EARTH


def mrp_to_dcm(sigma: np.ndarray) -> np.ndarray:
    """Upstream ``RigidBodyKinematics.MRP2C``."""
    s2 = float(np.dot(sigma, sigma))
    tilde = np.array([[0.0, -sigma[2], sigma[1]], [sigma[2], 0.0, -sigma[0]], [-sigma[1], sigma[0], 0.0]])
    return np.eye(3) + (8.0 * tilde @ tilde - 4.0 * (1.0 - s2) * tilde) / (1.0 + s2) ** 2


def rv2hill(rc: np.ndarray, vc: np.ndarray, rd: np.ndarray) -> np.ndarray:
    """The position part of upstream ``orbitalMotion.rv2hill``."""
    ir = rc / np.linalg.norm(rc)
    h = np.cross(rc, vc)
    ih = h / np.linalg.norm(h)
    hill = np.array([ir, np.cross(ih, ir), ih])
    return hill @ (rd - rc)


def wrap(angle: float) -> float:
    if angle > math.pi:
        return angle - 2 * math.pi
    if angle < -math.pi:
        return angle + 2 * math.pi
    return angle


def element_differences(chief_r: np.ndarray, chief_v: np.ndarray, r: np.ndarray,
                        v: np.ndarray) -> np.ndarray:
    """Upstream's ``oed``: deputy minus chief, ``da`` normalized."""
    rows = []
    for rc, vc, rd, vd in zip(chief_r, chief_v, r, v, strict=True):
        chief = rv2elem(MU_EARTH, rc, vc)
        deputy = rv2elem(MU_EARTH, rd, vd)
        mean = lambda oe: E2M(f2E(oe["f"], oe["e"]), oe["e"])  # noqa: E731
        rows.append([
            (deputy["a"] - chief["a"]) / chief["a"],
            deputy["e"] - chief["e"],
            deputy["i"] - chief["i"],
            wrap(deputy["Omega"] - chief["Omega"]),
            wrap(deputy["omega"] - chief["omega"]),
            wrap(mean(deputy) - mean(chief)),
        ])
    return np.array(rows)


def build(generated_dir: Path, plt: Any) -> dict[str, Any]:
    index = int(os.environ.get("MULTI_SAT_SPACECRAFT", "1"))
    figures: dict[str, Any] = {}
    for flag in ("True", "False"):
        stem = STEM + flag
        tables = spacecraft_tables(generated_dir, stem)
        if not tables:
            continue
        table: Table = tables[index]
        time = table.time_ns * NANO2MIN
        sigma_bn = table.vectors("sigma_BN")
        omega_rn_n = table.vectors("omega_RN_N_rad_per_s")
        omega_rn_b = np.array([mrp_to_dcm(s) @ w for s, w in zip(sigma_bn, omega_rn_n, strict=True)])
        positions = [sat.vectors("r_BN_N_m") for sat in tables]
        velocities = [sat.vectors("v_BN_N_m_per_s") for sat in tables]
        chief = existing(generated_dir, [f"{stem}_chief"])
        if chief:
            chief_r, chief_v = chief[0][1].vectors("r_BN_N_m"), chief[0][1].vectors("v_BN_N_m_per_s")
            deputies = range(len(tables))
        else:
            chief_r, chief_v = positions[0], velocities[0]
            deputies = range(1, len(tables))
        relative = [
            np.array([rv2hill(rc, vc, rd) for rc, vc, rd in zip(chief_r, chief_v, positions[i], strict=True)])
            for i in deputies
        ]
        oed = element_differences(chief_r, chief_v, positions[index], velocities[index])
        period = 2 * math.pi * math.sqrt(SEMI_MAJOR_AXIS**3 / MU_EARTH)
        thrusters = sum(1 for name in table.columns if name.startswith("thrustFactor_"))
        name = lambda figure: f"{stem}_{figure}"  # noqa: E731
        figures.update({
            name("attitude"): plot_attitude(plt, time, sigma_bn),
            name("rate"): plot_rate(plt, time, table.vectors("omega_BN_B_rad_per_s")),
            name("attitudeTrackingError"): plot_attitude_error(plt, time, table.vectors("sigma_BR")),
            name("trackingErrorRate"): plot_rate_error(plt, time, table.vectors("omega_BR_B_rad_per_s")),
            name("attitudeReference"): plot_attitude_reference(plt, time, table.vectors("sigma_RN")),
            name("rateReference"): plot_rate_reference(plt, time, omega_rn_b),
            name("rwMotorTorque"): plot_rw_motor_torque(
                plt, time, columns(table, "motorTorque_Nm"), columns(table, "rwTorque_Nm")),
            name("rwSpeeds"): plot_rw_speeds(plt, time, columns(table, "wheelSpeeds_rad_per_s")),
            name("orbits"): plot_orbits(plt, positions),
            name("relativeOrbits"): plot_relative_orbits(plt, relative),
            name("oeDifferences"): plot_orbital_element_differences(
                plt, table.time_ns * NANO2SEC / period, oed),
            name("power"): plot_power(plt, time, table["netPower_W"], table["panelPower_W"],
                                      table["sinkPower_W"]),
            name("fuel"): plot_fuel(plt, time, table["fuelMass_kg"]),
            name("thrustPercentage"): plot_thrust_percentage(
                plt, time, [table[f"thrustFactor_{thruster}"] for thruster in range(thrusters)]),
        })
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(f"Plot {STEM} results.", build))
