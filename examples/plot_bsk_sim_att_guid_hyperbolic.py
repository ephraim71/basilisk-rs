#!/usr/bin/env python3
"""Figures of upstream BskSim scenario_AttGuidHyperbolic.py from the Rust CSV.

As upstream, the figures come from scenarioAttGuideHyperbolic's plot
functions (``plot_scenario_att_guide_hyperbolic.py``), whose hyperbola this
scenario flies.
"""
from __future__ import annotations

from typing import Any

import bsk_sim_plotting as bsk
from plot_scenario_att_guide_hyperbolic import plot_orbit
from plot_scenario_attitude_guidance import plot_attitude_error_norm
from scenario_plotting import CsvTable

STEM = "scenario_AttGuidHyperbolic"


def build(plt: Any, table: CsvTable) -> dict[str, Any]:
    table = bsk.drop_first(table)
    time = bsk.minutes(table)
    return {
        "attitudeErrorNorm": plot_attitude_error_norm(plt, list(time), table),
        "rwMotorTorque": bsk.plot_control_torque(
            plt, time, bsk.vectors(table, "torqueRequestBody_Nm")
        ),
        "rateError": bsk.plot_rate_error(plt, time, bsk.vectors(table, "omega_BR_B_rad_per_s")),
        "orbit": plot_orbit(plt, table),
    }


if __name__ == "__main__":
    raise SystemExit(bsk.run(STEM, build))
