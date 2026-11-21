#!/usr/bin/env python3
"""Figures of upstream BskSim scenario_BasicOrbit.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

import bsk_sim_plotting as bsk
from scenario_plotting import CsvTable

STEM = "scenario_BasicOrbit"


def build(plt: Any, table: CsvTable) -> dict[str, Any]:
    time = bsk.minutes(table)
    r_bn = bsk.vectors(table, "r_BN_N_m")
    return {
        "orbit": bsk.plot_orbit(plt, r_bn),
        "orientation": bsk.plot_orientation(
            plt, time, r_bn, bsk.vectors(table, "v_BN_N_m_per_s"), bsk.vectors(table, "sigma_BN")
        ),
    }


if __name__ == "__main__":
    raise SystemExit(bsk.run(STEM, build))
