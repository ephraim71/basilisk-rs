#!/usr/bin/env python3
"""Upstream `scenarioHaloOrbit` figures from `scenarioHaloOrbit.csv`: the
spacecraft and Moon orbits in the Earth-centered inertial frame, and the
spacecraft in the frame rotating with the Moon (Moon-velocity and
out-of-plane axes)."""

import math

import numpy as np

from orbit_plotting import MU_EARTH, existing, rv2elem, run_plots
from plot_scenario_lagrange_point_orbit import orbit_lines, rotating


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, ["scenarioHaloOrbit"]):
        r = table.vectors("r_BN_N_m")
        v = table.vectors("v_BN_N_m_per_s")
        moon_r = table.vectors("moon_r_N_m")
        moon_v = table.vectors("moon_v_N_m_per_s")
        oe = rv2elem(MU_EARTH, moon_r[0], moon_v[0])
        b = oe["a"] * math.sqrt(1 - oe["e"] ** 2)
        size = tuple(np.array((1.0, b / oe["a"])) * 4.75)

        figure = plt.figure(figsize=size, dpi=100)
        axis = figure.gca()
        axis.axis(np.array([-oe["rApoap"], oe["rPeriap"], -b, b]) / 1000 * 1.25)
        axis.ticklabel_format(style="scientific", scilimits=[-5, 3])
        axis.add_artist(plt.Circle((0, 0), 0.2e5, color="b"))
        orbit_lines(axis, r, v, moon_r, moon_v, oe)
        axis.set_xlabel(r"$i_e$ Coord. [km]")
        axis.set_ylabel(r"$i_p$ Coord. [km]")
        axis.grid()
        axis.legend()
        figures["scenarioHaloOrbitFig1"] = figure

        spacecraft = np.array([rotating(mr, mv, p) for mr, mv, p in zip(moon_r, moon_v, r)])
        for number, column, label in ((2, 1, "Moon Velocity axis [km]"), (3, 2, "Earth-Moon perpendicular axis [km]")):
            figure = plt.figure(figsize=size, dpi=100)
            axis = figure.gca()
            axis.axis(np.array([-1e5, 5e5, -3e5, 3e5]) * 1.25)
            axis.ticklabel_format(style="scientific", scilimits=[-5, 3])
            axis.add_artist(plt.Circle((0, 0), 0.2e5, color="b"))
            axis.plot(spacecraft[:, 0] / 1000, spacecraft[:, column] / 1000, color="g", linewidth=3.0,
                      label="Spacecraft")
            axis.set_xlabel("Earth-Moon axis [km]")
            axis.set_ylabel(label)
            axis.grid()
            axis.legend()
            figures[f"scenarioHaloOrbitFig{number}"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the halo orbit scenario.", build))
