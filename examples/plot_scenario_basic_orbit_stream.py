#!/usr/bin/env python3
"""Upstream `scenarioBasicOrbitStream` figures from
`scenarioBasicOrbitStream<orbit><sh><planet>.csv`: the inertial position over
the orbits, and the perifocal orbit (point mass) or the semi-major axis
(spherical harmonics)."""

import math

import numpy as np

from orbit_plotting import (MU_EARTH, MU_MARS, NANO2SEC, REQ_EARTH, REQ_MARS, elements, existing,
                            perifocal_orbit, plot_components, rv2elem, run_plots)

VARIANTS = ("LEO0Earth", "GTO0Earth", "GEO0Earth", "LEO1Earth", "LEO0Mars", "LEO1Mars")


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, [f"scenarioBasicOrbitStream{v}" for v in VARIANTS]):
        variant = stem.removeprefix("scenarioBasicOrbitStream")
        mars = variant.endswith("Mars")
        mu, radius = (MU_MARS, REQ_MARS) if mars else (MU_EARTH, REQ_EARTH)
        r = table.vectors("r_BN_N_m")
        v = table.vectors("v_BN_N_m_per_s")
        oe0 = rv2elem(mu, r[0], v[0])
        orbits = table.time_ns * NANO2SEC / (2 * math.pi * math.sqrt(oe0["a"] ** 3 / mu))
        figure = plt.figure()
        axis = figure.gca()
        axis.ticklabel_format(useOffset=False, style="plain")
        plot_components(axis, orbits, r / 1000.0, "$r_{{BN,{i}}}$", plt)
        axis.set_xlabel("Time [orbits]")
        axis.set_ylabel("Inertial Position [km]")
        figures[f"scenarioBasicOrbitStream1{variant}"] = figure
        if variant[3] == "0":
            b = oe0["a"] * math.sqrt(1 - oe0["e"] ** 2)
            figure = plt.figure(figsize=tuple(np.array((1.0, b / oe0["a"])) * 4.75), dpi=100)
            perifocal_orbit(figure.gca(), mu, oe0, r, v, radius, "#884400" if mars else "#008800", plt)
        else:
            figure = plt.figure()
            axis = figure.gca()
            axis.ticklabel_format(useOffset=False, style="plain")
            axis.plot(orbits, [oe["a"] / 1000.0 for oe in elements(mu, r, v)], color="#aa0000")
            axis.set_xlabel("Time [orbits]")
            axis.set_ylabel("SMA [km]")
        figures[f"scenarioBasicOrbitStream2{variant}"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the basic orbit stream scenario.", build))
