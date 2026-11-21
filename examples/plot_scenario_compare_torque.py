#!/usr/bin/env python3
"""Upstream `scenarioCompareTorque` figures from `scenarioCompareTorque.csv`:
the body rates of both engines per component (`_rate_x`, ... and their
differences), each engine's orbiting-versus-rest attitude change
(`_motionAttError`), the cross-engine attitude difference at rest and
orbiting (`_attError`) and the principal-axis control's errors against the
exact solution (`_analyticRateError`, `_analyticAttError`)."""

import numpy as np

from dyncomp_plotting import COLOR_BSM, COLOR_MUJOCO, component_comparison, has, vectors_or_none
from orbit_plotting import NANO2SEC, existing, run_plots

STEM = "scenarioCompareTorque"


def _two(plt, minutes, table, bsm, mujoco, floor, ylabel, degrees, figsize=None):
    figure, axis = plt.subplots(figsize=figsize) if figsize else plt.subplots()
    scale = np.degrees if degrees else (lambda values: values)
    axis.semilogy(minutes, np.maximum(scale(table[bsm]), scale(floor)), color=COLOR_BSM, label="BSM")
    if has(table, mujoco):
        axis.semilogy(minutes, np.maximum(scale(table[mujoco]), scale(floor)), color=COLOR_MUJOCO, label="MuJoCo")
    axis.set_xlabel("Time [min]")
    axis.set_ylabel(ylabel)
    axis.legend(loc="best")
    return figure


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        minutes = table.time_ns * NANO2SEC / 60.0
        figures.update(component_comparison(
            plt, f"{STEM}_rate", minutes, table.vectors("bsm_omega_BN_B_rad_per_s"),
            vectors_or_none(table, "mujoco_omega_BN_B_rad_per_s"), r"$\omega_{BN}$", "rad/s",
            xlabel="Time [min]"))
        figures[f"{STEM}_motionAttError"] = _two(
            plt, minutes, table, "bsm_motion_att_rad", "mujoco_motion_att_rad", 1e-18,
            "Attitude difference [deg]", True)

        figure, axis = plt.subplots()
        if has(table, "att_error_orbit_rad"):
            floor = np.degrees(1e-16)
            axis.semilogy(minutes, np.maximum(np.degrees(table["att_error_rest_rad"]), floor), color=COLOR_BSM,
                          label="At rest (no linear velocity)")
            axis.semilogy(minutes, np.maximum(np.degrees(table["att_error_orbit_rad"]), floor),
                          color=COLOR_MUJOCO, label="Orbiting (~7.5 km/s)")
            axis.legend(loc="best")
        axis.set_xlabel("Time [min]")
        axis.set_ylabel("Attitude difference [deg]")
        figures[f"{STEM}_attError"] = figure

        figures[f"{STEM}_analyticRateError"] = _two(
            plt, minutes, table, "bsm_analytic_rate_error_rad_per_s", "mujoco_analytic_rate_error_rad_per_s",
            1e-18, "Rate error [rad/s]", False, figsize=(4.8, 2.2))
        figures[f"{STEM}_analyticAttError"] = _two(
            plt, minutes, table, "bsm_analytic_att_error_rad", "mujoco_analytic_att_error_rad", 1e-18,
            "Attitude error [deg]", True, figsize=(4.8, 2.2))
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(__doc__, build))
