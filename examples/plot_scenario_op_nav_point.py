#!/usr/bin/env python3
"""Figures of upstream scenario_OpNavPoint.py from the Rust CSV: the wheel
torques and the circles found against the truth (``imgProcVsExp``)."""
from __future__ import annotations

from pathlib import Path
from typing import Any

import opnav_plotting as opnav
from orbit_plotting import Table, run_plots

STEM = "scenario_OpNavPoint"


def build(generated_dir: Path, plt: Any) -> dict[str, Any]:
    table = Table(generated_dir / f"{STEM}.csv")
    time = table.time_ns
    truth = opnav.camera_truth(opnav.columns(table, "r_BN_N_m", 3), opnav.columns(table, "sigma_BN", 3))
    figures = {}
    figures.update(opnav.plot_rw_motor_torque(
        plt, time * opnav.NS2MIN, opnav.columns(table, "motorTorque_Nm", 4), opnav.wheel_torques(table)))
    figures.update(opnav.img_proc_vs_exp(plt, time, truth, opnav.columns(table, "circlesCenters", 2),
                                         table["circlesRadii_0"]))
    return {f"{STEM}_{name}": figure for name, figure in figures.items()}


if __name__ == "__main__":
    raise SystemExit(run_plots(f"Plot {STEM} results.", build))
