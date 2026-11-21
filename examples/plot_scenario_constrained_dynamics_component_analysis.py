#!/usr/bin/env python3
"""Upstream `scenarioConstrainedDynamicsComponentAnalysis` figures from
`scenarioConstrainedDynamicsComponentAnalysis_<components>_<model>.csv`: the
direction and attitude constraint violations, and the panel angle or slosh
displacement errors against the truth model."""

import numpy as np

from multibody_plotting import model_geometry, violation_angles_deg, violations
from orbit_plotting import NANO2SEC, existing, line_color, run_plots

STEM = "scenarioConstrainedDynamicsComponentAnalysis"
VARIANTS = [f"_{components}_{model}"
            for components in ("panels", "slosh", "None", "panels_slosh")
            for model in ("bskSat", "MEV1", "MEV2")]


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, [STEM + variant for variant in VARIANTS]):
        components, model = stem[len(STEM) + 1:].rsplit("_", 1)
        label = components.split("_")[0]
        time = table.time_ns * NANO2SEC
        psi, sigma = violations(table, *model_geometry(model, np.ones(3)))

        figure = plt.figure()
        axis = figure.gca()
        for index in range(3):
            axis.semilogy(time, np.abs(psi[:, index]), alpha=0.8, color=line_color(index, 4, plt))
        axis.semilogy(time, np.linalg.norm(psi, axis=1), color=line_color(3, 4, plt))
        axis.legend([r"$\psi_1$", r"$\psi_2$", r"$\psi_3$", r"$|\mathbf{\psi}|$"], loc="best")
        axis.set_xlabel("Time [sec]")
        axis.set_ylabel("Direction Constraint \nViolation " r"$\psi$ [m]")
        figures[STEM + "DirectionConstraint_" + label] = figure

        angles = violation_angles_deg(sigma)
        figure = plt.figure()
        axis = figure.gca()
        for index in range(3):
            axis.semilogy(time, np.abs(angles[:, index]), alpha=0.8,
                          color=line_color(index, 4, plt))
        axis.semilogy(time, np.linalg.norm(angles, axis=1), color=line_color(3, 4, plt))
        axis.legend([r"$\phi_1$", r"$\phi_2$", r"$\phi_3$", r"$|\mathbf{\phi}|$"], loc="best")
        axis.set_xlabel("Time [sec]")
        axis.set_ylabel("Attitude Constraint \nViolation " r"$\phi$ [deg]")
        figures[STEM + "AttitudeConstraint_" + label] = figure

        if "panels" in components:
            figure = plt.figure()
            axis = figure.gca()
            for index, (docked, truth) in enumerate(((1, 3), (2, 4))):
                axis.plot(time, np.degrees(table[f"theta{docked}_rad"] - table[f"theta{truth}_rad"]),
                          color=line_color(index, 2, plt), label=rf"$\Delta\theta_{index + 1}$")
            axis.legend(loc="best")
            axis.set_xlabel("Time [sec]")
            axis.set_ylabel("Panel Angle Error [deg]")
            figures[STEM + "PanelAngleErrors"] = figure
        if "slosh" in components:
            figure = plt.figure()
            axis = figure.gca()
            for index in range(3):
                axis.plot(time, table[f"rho{index + 1}_m"] - table[f"rho{index + 4}_m"],
                          color=line_color(index, 3, plt), label=rf"$\Delta\rho_{index + 1}$")
            axis.legend(loc="best")
            axis.set_xlabel("Time [sec]")
            axis.set_ylabel("Mass Displacement Error [m]")
            figures[STEM + "SloshMassDisplacementErrors"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the constrained dynamics component analysis.", build))
