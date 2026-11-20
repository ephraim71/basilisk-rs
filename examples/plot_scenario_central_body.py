#!/usr/bin/env python3
"""Upstream `scenarioCentralBody` figures from `scenarioCentralBody<0|1>.csv`:
the Earth-relative position components over the orbits, and the orbit in the
equatorial plane with the initial osculating orbit dashed."""

import math

import numpy as np

from orbit_plotting import MU_EARTH, NANO2SEC, REQ_EARTH, existing, plot_components, rv2elem, run_plots


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, ["scenarioCentralBody0", "scenarioCentralBody1"]):
        case = stem[-1]
        r = table.vectors("r_BN_N_m") - table.vectors("earth_r_N_m")
        v = table.vectors("v_BN_N_m_per_s") - table.vectors("earth_v_N_m_per_s")
        oe = rv2elem(MU_EARTH, r[0], v[0])
        period = 2 * math.pi * math.sqrt(oe["a"] ** 3 / MU_EARTH)
        figure = plt.figure()
        axis = figure.gca()
        axis.ticklabel_format(useOffset=False, style="plain")
        plot_components(axis, table.time_ns * NANO2SEC / period, r / 1000.0, "$r_{{BN,{i}}}$", plt)
        axis.set_xlabel("Time [orbits]")
        axis.set_ylabel("Inertial Position [km]")
        figures[f"scenarioCentralBody1{case}"] = figure

        b = oe["a"] * math.sqrt(1 - oe["e"] ** 2)
        p = oe["a"] * (1 - oe["e"] ** 2)
        figure = plt.figure(figsize=tuple(np.array((1.0, b / oe["a"])) * 4.75), dpi=100)
        axis = figure.gca()
        axis.axis(np.array([-oe["rApoap"], oe["rPeriap"], -b, b]) / 1000 * 1.25)
        axis.add_artist(plt.Circle((0, 0), REQ_EARTH / 1000, color="#008800"))
        axis.plot(r[:, 0] / 1000, r[:, 1] / 1000, color="#aa0000", linewidth=3.0)
        f = np.linspace(0, 2 * np.pi, 100)
        radius = p / (1 + oe["e"] * np.cos(f))
        axis.plot(radius * np.cos(f) / 1000, radius * np.sin(f) / 1000, "--", color="#555555")
        axis.set_xlabel("$i_e$ Cord. [km]")
        axis.set_ylabel("$i_p$ Cord. [km]")
        axis.grid()
        figures[f"scenarioCentralBody2{case}"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the central body scenario.", build))
