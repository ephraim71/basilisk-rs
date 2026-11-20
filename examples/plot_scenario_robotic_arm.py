#!/usr/bin/env python3
"""Upstream `scenarioRoboticArm` figures from `scenarioRoboticArm.csv`: the
joint angles and rates, and the hub's attitude and rate."""

import numpy as np

from orbit_plotting import NANO2MIN, existing, line_color, run_plots

STEM = "scenarioRoboticArm"


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        t = table.time_ns * NANO2MIN
        for name, column, title, ylabel, label in (
            ("_theta", "theta{k}_rad", "Angles", r"$\theta$ [deg]", r"$\theta_{k}$"),
            ("_thetaDot", "theta{k}Dot_rad_per_s", "Angle Rates", r"$\dot{\theta}$ [deg/s]",
             r"$\dot{{\theta}}_{k}$"),
        ):
            figure = plt.figure()
            axis = figure.gca()
            for k in range(1, 5):
                axis.plot(t, np.degrees(table[column.format(k=k)]), label=label.format(k=k))
            axis.legend()
            axis.set_title(title)
            axis.set_xlabel("time [min]")
            axis.set_ylabel(ylabel)
            figures[STEM + name] = figure
        for name, prefix, title, ylabel, symbol in (
            ("_sigma_BN", "sigma_BN", "Attitude", r"$\sigma_{B/N}$", r"\sigma"),
            ("_omega_BN_B", "omega_BN_B_rad_per_s", "Attitude Rate", r"$\omega_{B/N}$", r"\omega"),
        ):
            figure = plt.figure()
            axis = figure.gca()
            values = table.vectors(prefix)
            for i in range(3):
                axis.plot(t, values[:, i], color=line_color(i, 3, plt), label=rf"${symbol}_{i}$")
            axis.legend()
            axis.set_title(title)
            axis.set_xlabel("time [min]")
            axis.set_ylabel(ylabel)
            figures[STEM + name] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the robotic arm scenario.", build))
