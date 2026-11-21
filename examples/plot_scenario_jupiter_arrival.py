#!/usr/bin/env python3
"""Upstream `scenarioJupiterArrival` figures from `scenarioJupiterArrival.csv`:
the inertial position over time, and the arrival and capture in Jupiter radii
with the desired circular parking orbit."""

import numpy as np

from orbit_plotting import NANO2SEC, REQ_JUPITER, existing, line_color, run_plots

PARKING_RADIUS_M = 800000.0 * 1000


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, ["scenarioJupiterArrival"]):
        positions = table.vectors("r_BN_N_m")
        figure = plt.figure()
        axis = figure.gca()
        axis.ticklabel_format(useOffset=False, style="plain")
        for index in range(3):
            axis.plot(table.time_ns * NANO2SEC, positions[:, index] / 1000.0,
                      color=line_color(index, 3, plt), label=f"$r_{{BN,{index}}}$")
        axis.legend(loc="lower right")
        axis.set_xlabel("Time [orbits]")
        axis.set_ylabel("Inertial Position [km]")
        figures["scenarioJupiterArrival1"] = figure

        figure = plt.figure(figsize=(5, 5))
        axis = figure.gca()
        axis.axis([-16, 16, -16, 16])
        axis.set_aspect("equal")
        axis.add_artist(plt.Circle((0, 0), 1.0, color="#008800"))
        axis.plot(positions[:, 0] / REQ_JUPITER, positions[:, 1] / REQ_JUPITER, color="orangered",
                  label="Simulated Flight")
        f = np.linspace(0, 2 * np.pi, 100)
        r = PARKING_RADIUS_M / REQ_JUPITER * np.ones_like(f)
        axis.plot(r * np.cos(f), r * np.sin(f), "--", color="#555555", label="Desired Circ. Parking Orbit")
        axis.set_xlabel("Jupiter Velocity Direction [DU]")
        axis.set_ylabel("Anti-Sunward Direction [DU]")
        axis.legend(loc="upper right")
        axis.grid()
        figures["scenarioJupiterArrival2"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the Jupiter arrival scenario.", build))
