#!/usr/bin/env python3
"""Upstream `scenarioRotTransArrayDeployment` figures from
`scenarioRotTransArrayDeployment.csv`: the array elements' angles and
displacements, and the hub's attitude, position and rate."""

import numpy as np

from orbit_plotting import NANO2MIN, existing, run_plots

STEM = "scenarioRotTransArrayDeployment"
ELEMENTS = 4


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        t = table.time_ns * NANO2MIN
        for name, column, scale, ylabel in (
            ("_ArrayElementsTheta", "theta_array{a}Element{k}_rad", np.degrees(1.0),
             r"$\theta$ (deg)"),
            ("_ArrayElementsRho", "rho_array{a}Element{k}_m", 1.0, r"$\rho$ (m)"),
        ):
            figure = plt.figure()
            axis = figure.gca()
            for array, style in ((1, "-"), (2, "--")):
                for k in range(1, ELEMENTS + 1):
                    axis.plot(t, scale * table[column.format(a=array, k=k)], linestyle=style,
                              label=f"Array {array} Element {k}")
            axis.set_xlabel("Time (min)")
            axis.set_ylabel(ylabel)
            axis.legend(loc="center left", bbox_to_anchor=(1, 0.5))
            axis.grid(True)
            figures[STEM + name] = figure
        for name, prefix, scale, ylabel, labels in (
            ("_HubInertialMRPAttitude", "sigma_BN", 1.0, r"$\sigma_{\mathcal{B}/\mathcal{N}}$",
             (r"$\sigma_1$", r"$\sigma_2$", r"$\sigma_3$")),
            ("_HubInertialPosition", "r_BN_N_m", 1.0, r"${}^\mathcal{N} r_{B/N}$ (m)",
             ("x", "y", "z")),
            ("_HubInertialAngularVelocity", "omega_BN_B_rad_per_s", np.degrees(1.0),
             r"${}^\mathcal{B} \omega_{\mathcal{B}/\mathcal{N}}$ (deg/s)",
             (r"$\omega_1$", r"$\omega_2$", r"$\omega_3$")),
        ):
            figure = plt.figure()
            axis = figure.gca()
            values = scale * table.vectors(prefix)
            for i, label in enumerate(labels):
                axis.plot(t, values[:, i], label=label)
            axis.set_xlabel("Time (min)")
            axis.set_ylabel(ylabel)
            axis.legend()
            axis.grid(True)
            figures[STEM + name] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the rotating and translating array deployment scenario.", build))
