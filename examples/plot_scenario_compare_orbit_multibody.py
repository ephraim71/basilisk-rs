#!/usr/bin/env python3
"""Upstream `scenarioCompareOrbitMultibody` figures from
`scenarioCompareOrbitMultibody.csv`: the compact body's center-of-mass
errors against Kepler and between the engines (`_compactOrbit`), the
extended body's hub attitude difference (`_attError`) and both regimes'
cross-engine center-of-mass difference (`_regimes`)."""

import numpy as np

from dyncomp_plotting import COLOR_BSM, COLOR_DIFF, COLOR_MUJOCO, R2D, has
from orbit_plotting import NANO2SEC, existing, run_plots

STEM = "scenarioCompareOrbitMultibody"


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        seconds = table.time_ns * NANO2SEC
        mujoco = has(table, "compact_cross_orbit_m")

        figure, axis = plt.subplots()
        axis.semilogy(seconds, np.maximum(table["compact_bsm_vs_kepler_m"], 1e-12), lw=4, alpha=0.4,
                      color=COLOR_BSM, label="BSM vs Kepler")
        if mujoco:
            axis.semilogy(seconds, np.maximum(table["compact_mujoco_vs_kepler_m"], 1e-12), lw=1.3,
                          color=COLOR_MUJOCO, label="MuJoCo vs Kepler")
            axis.semilogy(seconds, np.maximum(table["compact_cross_orbit_m"], 1e-12), lw=1.3, ls="--",
                          color=COLOR_DIFF, label="BSM vs MuJoCo")
        axis.set_xlabel("Time [s]")
        axis.set_ylabel("Center-of-mass\nposition error [m]")
        axis.legend(loc="best")
        figures[f"{STEM}_compactOrbit"] = figure

        figure, axis = plt.subplots(figsize=(3.5, 2.25))
        if mujoco:
            axis.plot(seconds, table["extended_att_error_rad"] * R2D, color=COLOR_MUJOCO,
                      label="BSM vs MuJoCo hub attitude")
            axis.legend(loc="best")
        axis.set_xlabel("Time [s]")
        axis.set_ylabel("Hub attitude difference [deg]")
        figures[f"{STEM}_attError"] = figure

        figure, axis = plt.subplots(figsize=(3.5, 2.25))
        if mujoco:
            axis.semilogy(seconds, np.maximum(table["compact_cross_orbit_m"], 1e-12), color=COLOR_BSM,
                          label="Compact (hub + wheels)")
            axis.semilogy(seconds, np.maximum(table["extended_cross_orbit_m"], 1e-12), color=COLOR_MUJOCO,
                          label="Extended (+ offset panels)")
            axis.legend(loc="best")
        axis.set_xlabel("Time [s]")
        axis.set_ylabel("CoM position difference [m]")
        figures[f"{STEM}_regimes"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(__doc__, build))
