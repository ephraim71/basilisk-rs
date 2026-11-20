#!/usr/bin/env python3
"""Upstream `scenarioDebrisReorbitET` figures from `scenarioDebrisReorbitET.csv`:
the servicer-debris separation, the servicer's orbit and the increase of the
debris' semi-major axis, over orbits."""

import math

import numpy as np

from orbit_plotting import MU_EARTH, REQ_EARTH, elements, existing, rv2elem, run_plots


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, ["scenarioDebrisReorbitET"]):
        rs, vs = table.vectors("servicer_r_BN_N_m"), table.vectors("servicer_v_BN_N_m_per_s")
        rd, vd = table.vectors("debris_r_BN_N_m"), table.vectors("debris_v_BN_N_m_per_s")
        oe0 = rv2elem(MU_EARTH, rs[0], vs[0])
        period = 2 * math.pi * math.sqrt(oe0["a"] ** 3 / MU_EARTH)
        orbits = table.time_ns * 1e-9 / period

        figure = plt.figure()
        axis = figure.gca()
        axis.plot(orbits, np.linalg.norm(rs - rd, axis=1))
        axis.set_xlabel("Time [orbits]")
        axis.set_ylabel("Separation [m]")
        figures["scenarioDebrisReorbitET1"] = figure

        figure = plt.figure()
        axis = figure.gca()
        axis.axis("equal")
        axis.add_artist(plt.Circle((0, 0), REQ_EARTH / 1000, color="#008800"))
        radius, angle = [], []
        for oe in elements(MU_EARTH, rs, vs):
            radius.append(oe["rmag"])
            angle.append(oe["f"] + oe["omega"] - oe0["omega"])
        radius, angle = np.array(radius), np.array(angle)
        axis.plot(radius * np.cos(angle) / 1000, radius * np.sin(angle) / 1000, color="#aa0000", linewidth=3.0)
        p = oe0["a"] * (1 - oe0["e"] ** 2)
        f = np.linspace(0, 2 * np.pi, 100)
        r = p / (1 + oe0["e"] * np.cos(f))
        axis.plot(r * np.cos(f) / 1000, r * np.sin(f) / 1000, "--", color="#555555")
        axis.set_xlabel("$x$ Cord. [km]")
        axis.set_ylabel("$y$ Cord. [km]")
        axis.grid()
        figures["scenarioDebrisReorbitET2"] = figure

        figure = plt.figure()
        axis = figure.gca()
        axis.ticklabel_format(useOffset=False, style="plain")
        semi_major = np.array([oe["a"] for oe in elements(MU_EARTH, rd, vd)])
        axis.plot(orbits, (semi_major - oe0["a"]) / 1000.0, color="#aa0000", linewidth=3.0)
        axis.set_xlabel("Time [orbits]")
        axis.set_ylabel("Increase of semi-major axis [km]")
        figures["scenarioDebrisReorbitET3"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the electrostatic tractor reorbit scenario.", build))
