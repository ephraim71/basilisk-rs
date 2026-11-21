#!/usr/bin/env python3
"""Upstream `scenarioCompareVariableMass` figures from
`scenarioCompareVariableMass[_deepSpace[_rigid]].csv`: the semi-major-axis
rise in orbit (`_orbit`) or the burn speed in deep space (`_burnSpeed`), the
total mass (`_fuelMass`, its difference in grams), the hub rates
(`_rate_x`, ...) and slosh displacements (`_slosh_x`, ...) of both engines
with their differences, and the cross-engine attitude and center-of-mass
differences (`_attError`, `_comError`)."""

import numpy as np

from dyncomp_plotting import COLOR_MUJOCO, component_comparison, has, scalar_comparison
from orbit_plotting import NANO2SEC, existing, run_plots

STEM = "scenarioCompareVariableMass"
VARIANTS = ("", "_deepSpace", "_deepSpace_rigid")


def _vectors(table, prefix):
    return np.column_stack([table[f"{prefix}_{axis}"] for axis in range(3)]) if has(table, f"{prefix}_0") else None


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, [STEM + variant for variant in VARIANTS]):
        t = table.time_ns * NANO2SEC
        mujoco = has(table, "mujoco_total_mass_kg")
        if has(table, "bsm_sma_rise_m"):
            figures.update(scalar_comparison(
                plt, f"{stem}_orbit", t, table["bsm_sma_rise_m"] * 1e-3,
                table["mujoco_sma_rise_m"] * 1e-3 if mujoco else None, "Semi-major axis rise [km]"))
        else:
            figures.update(scalar_comparison(
                plt, f"{stem}_burnSpeed", t, table["bsm_burn_speed_mps"],
                table["mujoco_burn_speed_mps"] if mujoco else None, "Burn speed [m/s]"))
        figures.update(scalar_comparison(
            plt, f"{stem}_fuelMass", t, table["bsm_total_mass_kg"],
            table["mujoco_total_mass_kg"] if mujoco else None, "Total mass [kg]", diff_unit="g", diff_scale=1e3))
        figures.update(component_comparison(
            plt, f"{stem}_rate", t, _vectors(table, "bsm_omega_BN_B_rad_per_s"),
            _vectors(table, "mujoco_omega_BN_B_rad_per_s"), r"$\omega_{BN}$", "mrad/s", scale=1e3))
        figures.update(component_comparison(
            plt, f"{stem}_slosh", t, _vectors(table, "bsm_rho_m"), _vectors(table, "mujoco_rho_m"),
            r"$\rho$", "mm", scale=1e3))
        if mujoco:
            figure, axis = plt.subplots(figsize=(4.8, 2.4))
            axis.semilogy(t, np.maximum(np.degrees(table["att_error_rad"]), np.degrees(1e-12)), color=COLOR_MUJOCO)
            axis.set_xlabel("Time [s]")
            axis.set_ylabel("Attitude difference [deg]")
            figures[f"{stem}_attError"] = figure
            figure, axis = plt.subplots(figsize=(4.8, 2.4))
            axis.plot(t, table["com_error_m"], color=COLOR_MUJOCO)
            axis.set_xlabel("Time [s]")
            axis.set_ylabel("CoM difference [m]")
            figures[f"{stem}_comError"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(__doc__, build))
