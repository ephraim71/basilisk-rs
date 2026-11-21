#!/usr/bin/env python3
"""Upstream `scenarioCompareRwPanels` figures from
`scenarioCompareRwPanels.csv`: the hub rates per component (`_rate_x`, ...),
the two hinge angles (`_panelPositive`, `_panelNegative`, with their
`_difference`) and the cross-engine hub attitude difference (`_attError`)."""

import numpy as np

from dyncomp_plotting import (COLOR_MUJOCO, R2D, component_comparison, has, scalar_comparison,
                              vectors_or_none)
from orbit_plotting import NANO2SEC, existing, run_plots

STEM = "scenarioCompareRwPanels"


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        seconds = table.time_ns * NANO2SEC
        figures.update(component_comparison(
            plt, f"{STEM}_rate", seconds, table.vectors("bsm_omega_BN_B_rad_per_s"),
            vectors_or_none(table, "mujoco_omega_BN_B_rad_per_s"), r"$\omega_{BN}$", "rad/s"))
        for name, panel in (("panelPositive", "panelP"), ("panelNegative", "panelM")):
            mujoco = f"mujoco_{panel}_theta_rad"
            figures.update(scalar_comparison(
                plt, f"{STEM}_{name}", seconds, table[f"bsm_{panel}_theta_rad"] * R2D,
                table[mujoco] * R2D if has(table, mujoco) else None, "Hinge angle [deg]"))
        figure, axis = plt.subplots(figsize=(4.8, 2.4))
        if has(table, "att_error_rad"):
            axis.semilogy(seconds, np.maximum(np.degrees(table["att_error_rad"]), np.degrees(1e-16)),
                          color=COLOR_MUJOCO)
        axis.set_xlabel("Time [s]")
        axis.set_ylabel("Attitude difference [deg]")
        figures[f"{STEM}_attError"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(__doc__, build))
