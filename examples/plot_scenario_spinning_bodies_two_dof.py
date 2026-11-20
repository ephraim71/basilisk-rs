#!/usr/bin/env python3
"""Upstream `scenarioSpinningBodiesTwoDOF` figures from
`scenarioSpinningBodiesTwoDOF<panels>.csv`: the joint angles and rates, the
hub velocity and the hub angular velocity, for each panel case found."""

import numpy as np

from orbit_plotting import NANO2SEC, existing, line_color, run_plots

STEM = "scenarioSpinningBodiesTwoDOF"


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, [f"{STEM}1", f"{STEM}2"]):
        panels = stem[-1]
        seconds = table.time_ns * NANO2SEC

        figure = plt.figure()
        axis = figure.gca()
        axis.plot(seconds, np.degrees(table["theta1_rad"]), label=r"$\theta_1$")
        axis.plot(seconds, np.degrees(table["theta2_rad"]), label=r"$\theta_2$")
        axis.legend()
        axis.set_xlabel("time [s]")
        axis.set_ylabel(r"$\theta$ [deg]")
        figures[f"{STEM}theta{panels}"] = figure

        figure = plt.figure()
        axis = figure.gca()
        axis.plot(seconds, np.degrees(table["theta1Dot_rad_per_s"]), label=r"$\dot{\theta}_1$")
        axis.plot(seconds, np.degrees(table["theta2Dot_rad_per_s"]), label=r"$\dot{\theta}_2$")
        axis.legend()
        axis.set_xlabel("time [s]")
        axis.set_ylabel(r"$\dot{\theta}$ [deg/s]")
        figures[f"{STEM}thetaDot{panels}"] = figure

        for name, prefix, label, ylabel in (
            ("velocity", "v_BN_N_m_per_s", "$v_{{BN,{i}}}$", "Velocity [m/s]"),
            ("angularVelocity", "omega_BN_B_rad_per_s", r"$\omega_{{BN,{i}}}$",
             "Angular Velocity [rad/s]"),
        ):
            figure = plt.figure()
            axis = figure.gca()
            values = table.vectors(prefix)
            for index in range(3):
                axis.plot(seconds, values[:, index], color=line_color(index, 3, plt),
                          label=label.format(i=index))
            axis.legend()
            axis.set_xlabel("time [s]")
            axis.set_ylabel(ylabel)
            figures[f"{STEM}{name}{panels}"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the two-degree-of-freedom spinning body scenario.", build))
