#!/usr/bin/env python3
"""Upstream `scenarioCompareFlexPanels` figures from
`scenarioCompareFlexPanels.csv` (one row per segment count): the attitude
difference of both BSM effectors to MuJoCo against the system degrees of
freedom (`_validity`) and each implementation's wall-clock for the fixed
600-step budget (`_runtime`)."""

import numpy as np

from dyncomp_plotting import COLOR_BSM, COLOR_DIFF, COLOR_MUJOCO, has
from orbit_plotting import existing, run_plots

STEM = "scenarioCompareFlexPanels"


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        dof = table["dof"]
        figure, axis = plt.subplots()
        if has(table, "attitude_error_max_rad"):
            floor = np.degrees(1e-16)
            axis.semilogy(dof, np.maximum(np.degrees(table["attitude_error_max_rad"]), floor), "o-",
                          color=COLOR_BSM, label="nHinged vs MuJoCo")
            axis.semilogy(dof, np.maximum(np.degrees(table["ndof_attitude_error_max_rad"]), floor), "^-",
                          color=COLOR_DIFF, label="spinningBodyNDOF vs MuJoCo")
            axis.legend(loc="best")
        axis.set_xlabel("System degrees of freedom")
        axis.set_ylabel("Attitude difference [deg]")
        figures[f"{STEM}_validity"] = figure

        figure, axis = plt.subplots()
        axis.loglog(dof, table["bsm_wall_s"], "o-", lw=4, alpha=0.4, ms=8, color=COLOR_BSM,
                    label="BSM — nHingedRigidBody")
        axis.loglog(dof, table["ndof_wall_s"], "^-", lw=2, ms=6, color=COLOR_DIFF, label="BSM — spinningBodyNDOF")
        if has(table, "mujoco_wall_s"):
            axis.loglog(dof, table["mujoco_wall_s"], "s-", lw=1.3, ms=5, color=COLOR_MUJOCO, label="MuJoCo")
        axis.set_xlabel("System degrees of freedom")
        axis.set_ylabel("Wall-clock for\nfixed step budget [s]")
        axis.legend(loc="best")
        figures[f"{STEM}_runtime"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(__doc__, build))
