#!/usr/bin/env python3
"""Upstream `scenarioOrbitManeuverTH` figures from `scenarioOrbitManeuverTH.csv`:
the inertial position and the radius over the hours, and the thrust of the
two thrusters."""

import numpy as np

from orbit_plotting import NANO2HOUR, existing, plot_components, run_plots


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, ["scenarioOrbitManeuverTH"]):
        hours = table.time_ns * NANO2HOUR
        r = table.vectors("r_BN_N_m")
        figure = plt.figure()
        axis = figure.gca()
        axis.ticklabel_format(useOffset=False, style="plain")
        plot_components(axis, hours, r / 1000.0, "$r_{{BN,{i}}}$", plt)
        axis.set_xlabel("Time [h]")
        axis.set_ylabel("Inertial Position [km]")
        figures["scenarioOrbitManeuverTH1"] = figure

        figure = plt.figure()
        axis = figure.gca()
        axis.ticklabel_format(useOffset=False, style="plain")
        axis.plot(hours, np.linalg.norm(r, axis=1) / 1000.0, color="#aa0000")
        axis.set_xlabel("Time [h]")
        axis.set_ylabel("Radius [km]")
        figures["scenarioOrbitManeuverTH2"] = figure

        figure = plt.figure()
        axis = figure.gca()
        axis.ticklabel_format(useOffset=False, style="plain")
        axis.plot(hours, table["thrustForce_N_0"], color="#aa0000", label="Thruster 1")
        axis.plot(hours, table["thrustForce_N_1"], label="Thruster 2")
        axis.legend(loc="upper right")
        axis.set_xlabel("Time [h]")
        axis.set_ylabel("Force implemented[N]")
        figures["scenarioOrbitManeuverTH3"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the thruster orbit maneuver scenario.", build))
