#!/usr/bin/env python3
"""Figures of upstream BskSim scenario_LambertGuidance.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

import bsk_sim_plotting as bsk
from scenario_plotting import CsvTable

STEM = "scenario_LambertGuidance"
TM2_S = 4980.0
R_TN_N_M = (42164.0 * 1000.0, 0.0, 0.0)


def build(plt: Any, table: CsvTable) -> dict[str, Any]:
    time = bsk.minutes(table)
    r_truth = bsk.vectors(table, "r_BN_N_m")
    v_truth = bsk.vectors(table, "v_BN_N_m_per_s")
    return {
        "orbit": bsk.plot_orbit(plt, r_truth),
        "position": bsk.plot_position(
            plt, time, r_truth, bsk.vectors(table, "meas_r_BN_N_m"), TM2_S / 60.0, R_TN_N_M
        ),
        "velocity": bsk.plot_velocity(plt, time, v_truth, bsk.vectors(table, "meas_v_BN_N_m_per_s")),
        "surfaceRelativeVelocity": bsk.plot_surface_rel_velocity(
            plt,
            time,
            r_truth,
            v_truth,
            bsk.vectors(table, "earth_sigma_BN"),
            bsk.vectors(table, "earth_omega_BN_B_rad_per_s"),
        ),
    }


if __name__ == "__main__":
    raise SystemExit(bsk.run(STEM, build))
