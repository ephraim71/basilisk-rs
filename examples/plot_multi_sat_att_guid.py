#!/usr/bin/env python3
"""Figures of upstream scenario_AttGuidMultiSat.py from the Rust CSVs
(``scenario_AttGuidMultiSat_sat<i>.csv``): attitude, rate, attitude tracking
error, tracking error rate, wheel motor torques and wheel speeds of one
spacecraft, by default spacecraft 2 as upstream's ``pull_outputs(show_plots, 2)``
(``MULTI_SAT_SPACECRAFT`` selects another)."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any

from multi_sat_plotting import (
    NANO2MIN,
    columns,
    plot_attitude,
    plot_attitude_error,
    plot_rate,
    plot_rate_error,
    plot_rw_motor_torque,
    plot_rw_speeds,
)
from orbit_plotting import Table, existing, run_plots

STEM = "scenario_AttGuidMultiSat"


def build(generated_dir: Path, plt: Any) -> dict[str, Any]:
    spacecraft = int(os.environ.get("MULTI_SAT_SPACECRAFT", "2"))
    found = existing(generated_dir, [f"{STEM}_sat{spacecraft}"])
    if not found:
        return {}
    table: Table = found[0][1]
    time = table.time_ns * NANO2MIN
    return {
        f"{STEM}_attitude": plot_attitude(plt, time, table.vectors("sigma_BN")),
        f"{STEM}_rate": plot_rate(plt, time, table.vectors("omega_BN_B_rad_per_s")),
        f"{STEM}_attitude_tracking_error": plot_attitude_error(plt, time, table.vectors("sigma_BR")),
        f"{STEM}_tracking_error_rate": plot_rate_error(plt, time, table.vectors("omega_BR_B_rad_per_s")),
        f"{STEM}_rw_motor_torque": plot_rw_motor_torque(
            plt, time, columns(table, "motorTorque_Nm"), columns(table, "rwTorque_Nm")),
        f"{STEM}_rw_speeds": plot_rw_speeds(plt, time, columns(table, "wheelSpeeds_rad_per_s")),
    }


if __name__ == "__main__":
    raise SystemExit(run_plots(f"Plot {STEM} results.", build))
