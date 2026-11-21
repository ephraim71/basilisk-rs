#!/usr/bin/env python3
"""Figures of upstream scenario_OpNavHeading.py from the Rust CSV: the
filtered heading against the truth in the camera frame (``vecTrack``), from
the first circle on, as upstream's ``pull_outputs``."""
from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

import opnav_plotting as opnav
from orbit_plotting import Table, run_plots

STEM = "scenario_OpNavHeading"


def build(generated_dir: Path, plt: Any) -> dict[str, Any]:
    table = Table(generated_dir / f"{STEM}.csv")
    position = opnav.columns(table, "r_BN_N_m", 3)
    sigma = opnav.columns(table, "sigma_BN", 3)
    # Upstream `trueRhat_C`: `trueR_C` scaled by `focal / z`, which turns it
    # towards the planet, then normalized.
    true_r_c = opnav.camera_truth(position, sigma)["r_C"]
    true_rhat = true_r_c / true_r_c[:, 2:3]
    true_rhat /= np.linalg.norm(true_rhat, axis=1)[:, None]
    # Upstream `r_NB_hat_C`: minus the filtered OpNav direction, the first
    # (zero) sample set to +z.
    r_c = opnav.columns(table, "headingOpnav_r_BN_C_m", 3).astype(float)
    r_c[0] = [0.0, 0.0, 1.0]
    norms = np.linalg.norm(r_c, axis=1)
    norms[norms == 0] = 1.0
    track = -r_c / norms[:, None]
    # Upstream's `covarC`: the filter's heading covariance in the camera
    # frame (`sigma_CB = 0`, so the body frame's).
    covar = opnav.columns(table, "heading_covar", 25).reshape(-1, 5, 5)[:, :3, :3]
    centers = table["circlesCenters_0"]
    switch = int(np.argmax(centers > 0)) if np.any(centers > 0) else 0
    figures = opnav.vec_track(plt, table.time_ns[switch:], true_rhat[switch:], track[switch:], covar[switch:])
    return {f"{STEM}_{name}": figure for name, figure in figures.items()}


if __name__ == "__main__":
    raise SystemExit(run_plots(f"Plot {STEM} results.", build))
