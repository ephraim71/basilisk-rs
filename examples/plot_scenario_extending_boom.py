#!/usr/bin/env python3
"""Upstream `scenarioExtendingBoom` figures from `scenarioExtendingBoom.csv`:
the segment displacements and rates, the hub attitude and its rate."""

import numpy as np

from orbit_plotting import NANO2MIN, existing, line_color, run_plots

STEM = "scenarioExtendingBoom"


def styled(axis, ylabel):
    axis.legend(fontsize="14")
    axis.set_xlabel("time [min]", fontsize="18")
    axis.set_ylabel(ylabel, fontsize="18")
    axis.tick_params(labelsize=14)


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        minutes = table.time_ns * NANO2MIN

        for name, column, symbol, ylabel in (
            ("Rho", "rho{}_m", r"\rho", r"$\rho$ [m]"),
            ("RhoDot", "rho{}Dot_m_per_s", r"\dot{\rho}", r"$\dot{\rho}$ [m/s]"),
        ):
            figure = plt.figure()
            axis = figure.gca()
            for index in range(1, 5):
                axis.plot(minutes, table[column.format(index)], label=rf"${symbol}_{index}$")
            styled(axis, ylabel)
            figures[STEM + name] = figure

        figure = plt.figure()
        axis = figure.gca()
        sigma = table.vectors("sigma_BN")
        for index in range(3):
            axis.plot(minutes, sigma[:, index], color=line_color(index, 3, plt),
                      label=rf"$\sigma_{index}$")
        styled(axis, r"$\sigma_{B/N}$")
        figures[STEM + "Sigma"] = figure

        figure = plt.figure()
        axis = figure.gca()
        omega = table.vectors("omega_BN_B_rad_per_s")
        for index in range(3):
            axis.plot(minutes, omega[:, index], color=line_color(index, 4, plt),
                      label=rf"$\omega_{index}$")
        axis.plot(minutes, np.linalg.norm(omega, axis=1), color=line_color(3, 4, plt),
                  label=r"$|\mathbf{\omega}|$", linestyle="dashed")
        styled(axis, r"$\omega_{B/N}$ [rad/s]")
        figures[STEM + "Omega"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the extending boom scenario.", build))
