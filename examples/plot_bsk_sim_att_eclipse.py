#!/usr/bin/env python3
"""Figures of upstream BskSim scenario_AttEclipse.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

import bsk_sim_plotting as bsk
from scenario_plotting import CsvTable

STEM = "scenario_AttEclipse"


def build(plt: Any, table: CsvTable) -> dict[str, Any]:
    table = bsk.drop_first(table)
    time = bsk.minutes(table)
    return {
        "attitudeErrorNorm": bsk.plot_attitude_error(plt, time, bsk.vectors(table, "sigma_BR")),
        "rwMotorTorque": bsk.plot_rw_cmd_torque(
            plt, time, bsk.vectors(table, "motorTorque_Nm", 4), 4
        ),
        "rateError": bsk.plot_rate_error(plt, time, bsk.vectors(table, "omega_BR_B_rad_per_s")),
        "rwSpeed": bsk.plot_rw_speeds(plt, time, bsk.vectors(table, "wheelSpeeds_rad_per_s", 4), 4),
        "illuminationFraction": bsk.plot_illumination_fraction(
            plt, time, table.column("illuminationFactor")
        ),
        "sunDirectionVector": bsk.plot_sun_point(plt, time, bsk.vectors(table, "vehSunPntBdy")),
    }


if __name__ == "__main__":
    raise SystemExit(bsk.run(STEM, build))
