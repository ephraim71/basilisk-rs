#!/usr/bin/env python3
"""Upstream `scenarioDragSensitivity` figures from `scenarioDragSensitivity.csv`
and `scenarioDragSensitivityTrajectories.csv`: every fifth run's Hill
trajectory, and the terminal position and velocity errors over the initial
displacement and the density multiplier."""

import numpy as np

from orbit_plotting import Table, run_plots


def build(generated_dir, plt):
    import matplotlib.colors as colors

    figures = {}
    path = generated_dir / "scenarioDragSensitivityTrajectories.csv"
    if path.is_file():
        table = Table(path)
        figure = plt.figure()
        axis = figure.gca()
        for run in np.unique(table["run"]):
            rows = table["run"] == run
            axis.plot(table["r_DC_H_m_0"][rows], table["r_DC_H_m_1"][rows])
        axis.grid()
        axis.set_xlabel("Hill X (m)")
        axis.set_ylabel("Hill Y (m)")
        figures["scenarioDragSensitivity_hillTrajectories"] = figure

    path = generated_dir / "scenarioDragSensitivity.csv"
    if path.is_file():
        table = Table(path)
        ranges = np.unique(table["initial_range_m"].round(decimals=2))
        densities = np.unique(table["density_multiplier"])
        x, y = np.meshgrid(ranges, densities)
        z = table["final_range_m"].reshape([len(ranges), len(densities)])
        z_vel = table["final_speed_mps"].reshape([len(ranges), len(densities)])
        for name, values, label in (
            ("_positionSensitivity", z, "Terminal Positioning Error (m)"),
            ("_velocitySensitivity", z_vel, "Terminal Velocity Error (m/s)"),
        ):
            figure = plt.figure()
            contour = plt.contourf(x.T, y.T, values, 30, norm=colors.LogNorm(z.min(), z.max()))
            plt.ylabel("Density Multiplier")
            plt.xlabel("Initial Displacement (m)")
            figure.colorbar(contour).set_label(label)
            figures["scenarioDragSensitivity" + name] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the drag sensitivity sweep.", build))
