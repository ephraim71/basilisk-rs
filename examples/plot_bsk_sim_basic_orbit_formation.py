#!/usr/bin/env python3
"""Figures of upstream BskSim scenario_BasicOrbitFormation.py from the Rust CSV."""
from __future__ import annotations

import math
from typing import Any

import bsk_sim_plotting as bsk
from scenario_plotting import CsvTable

STEM = "scenario_BasicOrbitFormation"
CHIEF_OE = {"a": 10_000_000.0, "e": 0.01, "omega": math.radians(347.8)}
DEPUTY_OE = {"a": 12_000_000.0, "e": 0.1, "omega": math.radians(347.8)}


def build(plt: Any, table: CsvTable) -> dict[str, Any]:
    time = bsk.minutes(table)
    vec = lambda prefix: bsk.vectors(table, prefix)  # noqa: E731
    figures = {
        "attitude_error_chief": bsk.plot_attitude_error(plt, time, vec("chief_sigma_BR")),
        "rate_error_chief": bsk.plot_rate_error(plt, time, vec("chief_omega_BR_B_rad_per_s")),
        "attitude_error_deputy": bsk.plot_attitude_error(plt, time, vec("deputy_sigma_BR")),
        "rate_error_deputy": bsk.plot_rate_error(plt, time, vec("deputy_omega_BR_B_rad_per_s")),
    }
    orbits = plt.figure()
    bsk.plot_peri_and_orbit(plt, orbits, CHIEF_OE, bsk.MU_EARTH, vec("chief_r_BN_N_m"),
                            vec("chief_v_BN_N_m_per_s"))
    bsk.plot_peri_and_orbit(plt, orbits, DEPUTY_OE, bsk.MU_EARTH, vec("deputy_r_BN_N_m"),
                            vec("deputy_v_BN_N_m_per_s"))
    figures["orbits"] = orbits
    return figures


if __name__ == "__main__":
    raise SystemExit(bsk.run(STEM, build))
