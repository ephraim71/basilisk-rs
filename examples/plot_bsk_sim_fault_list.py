#!/usr/bin/env python3
"""Figures of upstream BskSim scenario_FaultList.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

import numpy as np

import bsk_sim_plotting as bsk
from scenario_plotting import CsvTable

STEM = "scenario_FaultList"


def build(plt: Any, table: CsvTable) -> dict[str, Any]:
    table = bsk.drop_first(table)
    time = bsk.minutes(table)
    c_pn = np.array(
        [[table.column(f"J20002Pfix_{row}{column}") for column in range(3)] for row in range(3)]
    ).transpose(2, 0, 1)
    return {
        "attitudeErrorNorm": bsk.plot_attitude_error(plt, time, bsk.vectors(table, "sigma_BR")),
        "rateError": bsk.plot_rate_error(plt, time, bsk.vectors(table, "omega_BR_B_rad_per_s")),
        "RWSpeeds": bsk.plot_rw_speeds(plt, time, bsk.vectors(table, "wheelSpeeds_rad_per_s", 4), 4),
        "Magnetometer": bsk.plot_data_tam(plt, time, bsk.vectors(table, "tam_S_T")),
        "Latitude": bsk.plot_data_lat(plt, time, bsk.vectors(table, "nav_r_BN_N_m"), c_pn),
    }


if __name__ == "__main__":
    raise SystemExit(bsk.run(STEM, build))
