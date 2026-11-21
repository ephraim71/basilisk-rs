#!/usr/bin/env python3
"""Upstream `sweepVariableMassPendulumInertia` figure from
`sweepVariableMassPendulumInertia.csv`: the deep-space cross-engine attitude
difference against the MuJoCo bob's initial inertia, one line per RK4 step."""

import numpy as np

from dyncomp_plotting import COLOR_BSM, COLOR_DIFF, COLOR_MUJOCO
from orbit_plotting import existing, run_plots

STEM = "sweepVariableMassPendulumInertia"


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        inertia = table["bob_inertia_kg_m2"]
        figure, axis = plt.subplots(figsize=(4.8, 2.5))
        steps = [name for name in table.columns if name.startswith("att_error_rad_dt_")]
        for index, column in enumerate(steps):
            dt = float(column.removeprefix("att_error_rad_dt_"))
            axis.loglog(inertia, np.degrees(table[column]), color=(COLOR_BSM, COLOR_MUJOCO, COLOR_DIFF)[index % 3],
                        marker=("o", "s", "^")[index % 3], label=rf"$\Delta t={dt:g}$ s")
        axis.set_xlabel(r"Initial bob inertia [kg m$^2$]")
        axis.set_ylabel("Attitude difference [deg]")
        axis.legend(loc="best", fontsize=8)
        figures[STEM] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(__doc__, build))
