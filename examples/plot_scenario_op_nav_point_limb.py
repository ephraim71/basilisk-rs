#!/usr/bin/env python3
"""Figures of upstream scenario_OpNavPointLimb.py from the Rust CSV: the
wheel torques, the attitude error and the rate error."""
from __future__ import annotations

from pathlib import Path
from typing import Any

import opnav_plotting as opnav
from orbit_plotting import Table, run_plots

STEM = "scenario_OpNavPointLimb"


def build(generated_dir: Path, plt: Any) -> dict[str, Any]:
    table = Table(generated_dir / f"{STEM}.csv")
    time = table.time_ns * opnav.NS2MIN
    figures = {}
    figures.update(opnav.plot_rw_motor_torque(plt, time, opnav.columns(table, "motorTorque_Nm", 4),
                                              opnav.wheel_torques(table)))
    figures.update(opnav.plot_attitude_error(plt, time, opnav.columns(table, "sigma_BR", 3)))
    figures.update(opnav.plot_rate_error(plt, time, opnav.columns(table, "omega_BR_B_rad_per_s", 3)))
    return {f"{STEM}_{name}": figure for name, figure in figures.items()}


if __name__ == "__main__":
    raise SystemExit(run_plots(f"Plot {STEM} results.", build))
