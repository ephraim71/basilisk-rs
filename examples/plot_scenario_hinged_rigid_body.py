#!/usr/bin/env python3
"""Upstream `scenarioHingedRigidBody` figures from `scenarioHingedRigidBody.csv`:
the inertial position, the orbit radius and both panel angles."""

import numpy as np

from orbit_plotting import NANO2MIN, existing, plot_components, run_plots

STEM = "scenarioHingedRigidBody"


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        minutes = table.time_ns * NANO2MIN
        position = table.vectors("r_BN_N_m")

        figure = plt.figure()
        axis = figure.gca()
        axis.ticklabel_format(useOffset=False, style="plain")
        plot_components(axis, minutes, position / 1000.0, "$r_{{BN,{i}}}$", plt)
        axis.set_xlabel("Time [h]")
        axis.set_ylabel("Inertial Position [km]")
        figures[STEM + "10"] = figure

        figure = plt.figure()
        axis = figure.gca()
        axis.ticklabel_format(useOffset=False, style="plain")
        axis.plot(minutes, np.linalg.norm(position, axis=1) / 1000.0, color="#aa0000")
        axis.set_xlabel("Time [min]")
        axis.set_ylabel("Radius [km]")
        figures[STEM + "20"] = figure

        for panel in (1, 2):
            figure = plt.figure()
            axis = figure.gca()
            axis.ticklabel_format(useOffset=False, style="plain")
            axis.plot(minutes, table[f"panel{panel}_theta_rad"])
            axis.set_xlabel("Time [min]")
            axis.set_ylabel(f"Panel {panel} Angular Displacement [r]")
            figures[f"{STEM}panel{panel}theta0"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the hinged rigid body maneuver scenario.", build))
