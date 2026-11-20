#!/usr/bin/env python3
"""Upstream `scenarioOrbitMultiBody` figures from
`scenarioOrbitMultiBody<Hubble|NewHorizons>.csv`: the inertial position over
the minutes (km, or AU for New Horizons), for Hubble the orbit against the
SPICE trajectory, and the position difference from SPICE."""

import math

import numpy as np

from orbit_plotting import AU, MU_EARTH, NANO2MIN, REQ_EARTH, elements, existing, plot_components, rv2elem, run_plots


def build(generated_dir, plt):
    figures = {}
    stems = ["scenarioOrbitMultiBodyHubble", "scenarioOrbitMultiBodyNewHorizons"]
    for stem, table in existing(generated_dir, stems):
        case = stem.removeprefix("scenarioOrbitMultiBody")
        minutes = table.time_ns * NANO2MIN
        r = table.vectors("r_BN_N_m")
        v = table.vectors("v_BN_N_m_per_s")
        spice_r = table.vectors("spice_r_BN_N_m")
        spice_v = table.vectors("spice_v_BN_N_m_per_s")
        scale, unit = (AU, "[AU]") if case == "NewHorizons" else (1000.0, "[km]")
        figure = plt.figure()
        axis = figure.gca()
        axis.ticklabel_format(useOffset=False, style="plain")
        plot_components(axis, minutes, r / scale, "$r_{{BN,{i}}}$", plt)
        axis.set_xlabel("Time [min]")
        axis.set_ylabel("Inertial Position " + unit)
        figures[f"scenarioOrbitMultiBody1{case}"] = figure

        if case == "Hubble":
            oe0 = rv2elem(MU_EARTH, r[0], v[0])
            b = oe0["a"] * math.sqrt(1 - oe0["e"] ** 2)
            figure = plt.figure(figsize=tuple(np.array((1.0, b / oe0["a"])) * 4.75), dpi=100)
            axis = figure.gca()
            axis.axis(np.array([-oe0["rApoap"], oe0["rPeriap"], -b, b]) / 1000 * 1.25)
            axis.add_artist(plt.Circle((0, 0), REQ_EARTH / 1000, color="#008800"))
            for positions, velocities, style, label in ((r, v, "-", "Basilisk"), (spice_r, spice_v, "--", "Spice")):
                oes = elements(MU_EARTH, positions, velocities)
                radius = np.array([oe["rmag"] for oe in oes])
                angle = np.array([oe["f"] + oe["omega"] - oe0["omega"] for oe in oes])
                axis.plot(radius * np.cos(angle) / 1000, radius * np.sin(angle) / 1000, style,
                          color="#aa0000" if label == "Basilisk" else "#555555",
                          linewidth=0.5 if label == "Basilisk" else 1.0, label=label)
            axis.legend(loc="lower right")
            axis.set_xlabel("$i_e$ Cord. [km]")
            axis.set_ylabel("$i_p$ Cord. [km]")
            axis.grid()
            figures[f"scenarioOrbitMultiBody2{case}"] = figure

        figure = plt.figure()
        axis = figure.gca()
        axis.ticklabel_format(useOffset=False, style="plain")
        plot_components(axis, minutes, r - spice_r, r"$\Delta r_{{{i}}}$", plt)
        axis.set_xlabel("Time [min]")
        axis.set_ylabel("Inertial Position Differences [m]")
        figures[f"scenarioOrbitMultiBody3{case}"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the multi-body orbit scenario.", build))
