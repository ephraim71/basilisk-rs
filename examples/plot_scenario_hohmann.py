#!/usr/bin/env python3
"""Upstream `scenarioHohmann` figures from `scenarioHohmann.csv`: the orbit in
3D, the attitude error sigma_BR, the attitude sigma_BN and the reference
sigma_RN over the minutes."""

import numpy as np

from orbit_plotting import NANO2MIN, existing, plot_components, run_plots


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, ["scenarioHohmann"]):
        r = table.vectors("r_BN_N_m") / 1000
        figure = plt.figure()
        axis = figure.add_subplot(projection="3d")
        limit = 1.1 * np.max(np.linalg.norm(r, axis=1))
        u, v = np.mgrid[0:2 * np.pi:40j, 0:np.pi:40j]
        radius = 6378.1
        axis.plot_surface(radius * np.cos(u) * np.sin(v), radius * np.sin(u) * np.sin(v),
                          radius * np.cos(v), color="#008800")
        axis.plot3D(r[:, 0], r[:, 1], r[:, 2], color="orangered", label="Simulated Flight")
        axis.set_xlim3d(-limit, limit)
        axis.set_ylim3d(-limit, limit)
        axis.set_zlim3d(-limit, limit)
        axis.set_xlabel("x [km]")
        axis.set_ylabel("y [km]")
        axis.set_zlabel("z [km]")
        axis.set_aspect("equal")
        figures["scenarioHohmann1"] = figure

        minutes = table.time_ns * NANO2MIN
        for number, prefix, label in (
            (2, "sigma_BR", r"Attitude Error $\sigma_{B/R}$"),
            (3, "sigma_BN", r"Attitude $\sigma_{B/N}$"),
            (4, "sigma_RN", r"Reference Attitude $\sigma_{R/N}$"),
        ):
            figure = plt.figure()
            axis = figure.gca()
            plot_components(axis, minutes, table.vectors(prefix), r"$\sigma_{i}$", plt, legend="best")
            axis.set_xlabel("Time [min]")
            axis.set_ylabel(label)
            figures[f"scenarioHohmann{number}"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the Hohmann transfer scenario.", build))
