#!/usr/bin/env python3
"""Figures of upstream BskSim scenario_AttGuidance.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

import bsk_sim_plotting as bsk
from scenario_plotting import CsvTable

STEM = "scenario_AttGuidance"


def build(plt: Any, table: CsvTable) -> dict[str, Any]:
    table = bsk.drop_first(table)
    time = bsk.minutes(table)
    vec = lambda prefix: bsk.vectors(table, prefix)  # noqa: E731
    return {
        "attitudeErrorNorm": bsk.plot_attitude_error(plt, time, vec("sigma_BR")),
        "rwMotorTorque": bsk.plot_control_torque(plt, time, vec("torqueRequestBody_Nm")),
        "rateError": bsk.plot_rate_error(plt, time, vec("omega_BR_B_rad_per_s")),
        "orientation": bsk.plot_orientation(
            plt, time, vec("r_BN_N_m"), vec("v_BN_N_m_per_s"), vec("sigma_BN")
        ),
        "attitudeGuidance": bsk.plot_attitude_guidance(plt, time, vec("sigma_RN")),
    }


if __name__ == "__main__":
    raise SystemExit(bsk.run(STEM, build))
