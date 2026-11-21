#!/usr/bin/env python3
"""Upstream `scenarioOrbitManeuver` figures from `scenarioOrbitManeuver<case>.csv`:
the inertial position over the hours, and the inclination (case 1, with
upstream's 8.93845 degree reference) or the radius (case 0)."""

import math

import numpy as np

from orbit_plotting import MU_EARTH, NANO2HOUR, elements, existing, plot_components, run_plots


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, ["scenarioOrbitManeuver0", "scenarioOrbitManeuver1"]):
        case = stem[-1]
        hours = table.time_ns * NANO2HOUR
        r = table.vectors("r_BN_N_m")
        v = table.vectors("v_BN_N_m_per_s")
        figure = plt.figure()
        axis = figure.gca()
        axis.ticklabel_format(useOffset=False, style="plain")
        plot_components(axis, hours, r / 1000.0, "$r_{{BN,{i}}}$", plt)
        axis.set_xlabel("Time [h]")
        axis.set_ylabel("Inertial Position [km]")
        figures[f"scenarioOrbitManeuver1{case}"] = figure

        figure = plt.figure()
        axis = figure.gca()
        axis.ticklabel_format(useOffset=False, style="plain")
        if case == "1":
            axis.plot(hours, np.ones(len(hours)) * 8.93845, "--", color="#444444")
            axis.plot(hours, [math.degrees(oe["i"]) for oe in elements(MU_EARTH, r, v)], color="#aa0000")
            axis.set_ylim([-1, 10])
            axis.set_ylabel("Inclination [deg]")
        else:
            axis.plot(hours, np.linalg.norm(r, axis=1) / 1000.0, color="#aa0000")
            axis.set_ylabel("Radius [km]")
        axis.set_xlabel("Time [h]")
        figures[f"scenarioOrbitManeuver2{case}"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the orbit maneuver scenario.", build))
