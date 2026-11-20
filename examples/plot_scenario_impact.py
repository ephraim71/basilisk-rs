#!/usr/bin/env python3
"""Upstream `scenarioImpact` figures from `scenarioImpact.csv`: the struck
sub-panel's joint angles and rates, and the hub's angular velocity."""

import numpy as np

from orbit_plotting import NANO2SEC, existing, line_color, run_plots


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, ["scenarioImpact"]):
        seconds = table.time_ns * NANO2SEC
        for name, suffix, ylabel, labels in (
            ("_theta", "rad", r"$\theta$ [deg]", (r"$\theta_1$", r"$\theta_2$")),
            ("_thetaDot", "rad_per_s", r"$\dot{\theta}$ [deg/s]", (r"$\dot{\theta}_1$", r"$\dot{\theta}_2$")),
        ):
            figure = plt.figure()
            axis = figure.gca()
            column = "theta{}_" + suffix if name == "_theta" else "theta{}Dot_" + suffix
            for index, label in enumerate(labels):
                axis.plot(seconds, np.degrees(table[column.format(index + 1)]), label=label)
            axis.set_xlabel("time [s]")
            axis.set_ylabel(ylabel)
            figures["scenarioImpact" + name] = figure
        figure = plt.figure()
        axis = figure.gca()
        omega = table.vectors("omega_BN_B_rad_per_s")
        for index in range(3):
            axis.plot(seconds, omega[:, index], color=line_color(index, 3, plt),
                      label=rf"$\omega_{{BN,{index}}}$")
        axis.set_xlabel("time [s]")
        axis.set_ylabel(r"Angular Velocity [rad/s]")
        figures["scenarioImpact_angularVelocity"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the debris impact scenario.", build))
