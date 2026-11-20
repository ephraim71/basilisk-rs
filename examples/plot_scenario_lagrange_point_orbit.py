#!/usr/bin/env python3
"""Upstream `scenarioLagrangePointOrbit` figures from
`scenarioLagrangePointOrbitL<n>.csv`: the spacecraft and Moon orbits in the
Earth-centered inertial frame (initial perifocal frame), and the spacecraft
in the frame rotating with the Moon. `rotating` and `orbit_lines` are shared
with `plot_scenario_halo_orbit.py`."""

import math

import numpy as np

from orbit_plotting import MU_EARTH, elements, existing, rv2elem, run_plots


def rotating(moon_r, moon_v, position):
    h = np.cross(moon_r, moon_v)
    dcm = np.array([moon_r / np.linalg.norm(moon_r), moon_v / np.linalg.norm(moon_v), h / np.linalg.norm(h)])
    return dcm @ position


def orbit_lines(axis, r, v, moon_r, moon_v, oe0):
    for positions, velocities, color, label in ((r, v, "g", "Spacecraft"), (moon_r, moon_v, "0.5", "Moon")):
        oes = elements(MU_EARTH, positions, velocities)
        radius = np.array([oe["rmag"] for oe in oes])
        angle = np.array([oe["f"] + oe["omega"] - oe0["omega"] for oe in oes])
        axis.plot(radius * np.cos(angle) / 1000, radius * np.sin(angle) / 1000, color=color,
                  linewidth=3.0, label=label)


def build(generated_dir, plt):
    figures = {}
    stems = [f"scenarioLagrangePointOrbitL{point}" for point in range(1, 6)]
    for stem, table in existing(generated_dir, stems):
        point = stem[-2:]
        r = table.vectors("r_BN_N_m")
        v = table.vectors("v_BN_N_m_per_s")
        moon_r = table.vectors("moon_r_N_m")
        moon_v = table.vectors("moon_v_N_m_per_s")
        oe0 = rv2elem(MU_EARTH, r[0], v[0])
        b = oe0["a"] * math.sqrt(1 - oe0["e"] ** 2)
        for number in (1, 2):
            figure = plt.figure(figsize=tuple(np.array((1.0, b / oe0["a"])) * 4.75), dpi=100)
            axis = figure.gca()
            axis.axis(np.array([-oe0["rApoap"], oe0["rPeriap"], -b, b]) / 1000 * 1.25)
            axis.ticklabel_format(style="scientific", scilimits=[-5, 3])
            axis.add_artist(plt.Circle((0, 0), 0.2e5, color="b"))
            if number == 1:
                orbit_lines(axis, r, v, moon_r, moon_v, oe0)
                axis.set_xlabel("$i_e$ Coord. [km]")
                axis.set_ylabel("$i_p$ Coord. [km]")
            else:
                # Upstream: polar angle from the Moon's true anomaly.
                spacecraft = elements(MU_EARTH, r, v)
                moon = elements(MU_EARTH, moon_r, moon_v)
                radius = np.array([oe["rmag"] for oe in spacecraft])
                angle = np.array([sc["f"] - mo["f"] + sc["omega"] - oe0["omega"]
                                  for sc, mo in zip(spacecraft, moon)])
                axis.plot(radius * np.cos(angle) / 1000, radius * np.sin(angle) / 1000, color="g",
                          linewidth=3.0, label="Spacecraft")
                moon_radius = np.array([oe["rmag"] for oe in moon])
                axis.plot(moon_radius / 1000, np.zeros(len(moon_radius)), color="0.5", linewidth=3.0,
                          label="Moon")
                axis.set_xlabel("Earth-Moon axis [km]")
                axis.set_ylabel("Earth-Moon perpendicular axis [km]")
            axis.grid()
            axis.legend()
            figures[f"scenarioLagrangePointOrbit{point}Fig{number}"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the Lagrange point orbit scenario.", build))
