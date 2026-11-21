#!/usr/bin/env python3
"""Figures of upstream OpNavMC/scenario_LimbAttOD.py's ``pull_outputs``
from the Rust CSVs of ``scenario_limb_att_od --single`` (its retained
messages, ``scenario_LimbAttOD.csv``, and limbs,
``scenario_LimbAttOD_limb.csv``): the orbit and the measurements, the
measurement errors in the camera frame, the limbs, and the filter's state
errors with their covariance (the post-fit residuals upstream also draws
are not among the retained messages). Its campaign's
percent errors are drawn by ``plot_scenario_op_nav_att_od_mc.py --scenario
scenario_LimbAttOD``."""
from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

import opnav_plotting as opnav
from orbit_plotting import Table, run_plots

STEM = "scenario_LimbAttOD"


def build(generated_dir: Path, plt: Any) -> dict[str, Any]:
    table = Table(generated_dir / f"{STEM}.csv")
    time = table.time_ns
    position = opnav.columns(table, "scMsg.r_BN_N", 3)
    velocity = opnav.columns(table, "scMsg.v_BN_N", 3)
    truth = opnav.camera_truth(position, opnav.columns(table, "scMsg.sigma_BN", 3))
    valid = table["opnavMsg.valid_0"]
    state = opnav.columns(table, "filtMsg.state", 6)
    variances = opnav.columns(table, "filtMsg.covar", 36)[:, [0, 7, 14, 21, 28, 35]]
    running = np.any(variances != 0, axis=1)
    error = np.where(running[:, None], np.column_stack([position, velocity]) - state, np.nan)
    figures = {}
    figures.update(opnav.plot_two_orbits(plt, position, opnav.columns(table, "opnavMsg.r_BN_N", 3)))
    figures.update(opnav.diff_vectors(plt, time, truth["r_C"], opnav.columns(table, "opnavMsg.r_BN_C", 3),
                                      valid, "Limb"))
    limb_path = generated_dir / f"{STEM}_limb.csv"
    if limb_path.is_file():
        figures.update(opnav.plot_limb(plt, Table(limb_path)))
    figures.update(opnav.plot_state_covar(plt, time, error, np.where(running[:, None], variances, np.nan)))
    return {f"{STEM}_{name}": figure for name, figure in figures.items()}


if __name__ == "__main__":
    raise SystemExit(run_plots(f"Plot {STEM} results.", build))
