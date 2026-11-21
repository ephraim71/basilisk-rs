#!/usr/bin/env python3
"""Upstream `scenarioCompareOrbit` figures from `scenarioCompareOrbit.csv`:
the inertial trajectory of the analytic, BSM and MuJoCo orbits
(`_trajectory`), each engine's error against Kepler (`_accuracy`) and the
cross-engine difference (`_crossError`)."""

import numpy as np

from dyncomp_plotting import COLOR_BSM, COLOR_DIFF, COLOR_MUJOCO, has, vectors_or_none
from orbit_plotting import NANO2SEC, existing, run_plots

STEM = "scenarioCompareOrbit"


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        hours = table.time_ns * NANO2SEC / 3600.0
        truth = table.vectors("truth_r_N_m")
        bsm = table.vectors("bsm_r_N_m")
        mujoco = vectors_or_none(table, "mujoco_r_N_m")

        figure, axis = plt.subplots()
        axis.plot(truth[:, 0] / 1e3, truth[:, 1] / 1e3, "-", lw=1.5, color=COLOR_DIFF, label="Analytic Kepler")
        axis.plot(bsm[:, 0] / 1e3, bsm[:, 1] / 1e3, "-", lw=4, alpha=0.4, color=COLOR_BSM,
                  label="Backsubstitution (BSM)")
        if mujoco is not None:
            axis.plot(mujoco[:, 0] / 1e3, mujoco[:, 1] / 1e3, "-", lw=1.3, color=COLOR_MUJOCO, label="MuJoCo")
        axis.set_xlabel("Inertial x [km]")
        axis.set_ylabel("Inertial y [km]")
        axis.axis("equal")
        axis.legend(loc="best")
        figures[f"{STEM}_trajectory"] = figure

        figure, axis = plt.subplots()
        axis.semilogy(hours, np.maximum(table["bsm_vs_kepler_m"], 1e-12), lw=4, alpha=0.4, color=COLOR_BSM,
                      label="BSM vs Kepler")
        if has(table, "mujoco_vs_kepler_m"):
            axis.semilogy(hours, np.maximum(table["mujoco_vs_kepler_m"], 1e-12), lw=1.3, color=COLOR_MUJOCO,
                          label="MuJoCo vs Kepler")
        axis.set_xlabel("Time [hr]")
        axis.set_ylabel("Position error [m]")
        axis.legend(loc="best")
        figures[f"{STEM}_accuracy"] = figure

        figure, axis = plt.subplots()
        if has(table, "cross_error_m"):
            axis.semilogy(hours, np.maximum(table["cross_error_m"], 1e-15), color=COLOR_MUJOCO)
        axis.set_xlabel("Time [hr]")
        axis.set_ylabel(r"$\|{\bf r}_{\rm BSM}-{\bf r}_{\rm MuJoCo}\|$ [m]")
        figures[f"{STEM}_crossError"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(__doc__, build))
