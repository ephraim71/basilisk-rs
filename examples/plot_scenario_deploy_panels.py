#!/usr/bin/env python3
"""Upstream `scenarioDeployPanels` figures from `scenarioDeployPanels.csv`
(and `scenarioDeployPanelsInitialSpin.csv`): each panel's desired and
achieved angle, and its motor torque."""

import numpy as np

from orbit_plotting import NANO2SEC, existing, run_plots

PANELS = ("p1", "p2", "p3", "n1", "n2", "n3")


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, ["scenarioDeployPanels",
                                                "scenarioDeployPanelsInitialSpin"]):
        time = table.time_ns * NANO2SEC
        figure, axes = plt.subplots(2, 3)
        for axis, panel in zip(axes.flat, PANELS):
            first = axis is axes.flat[0]
            axis.plot(time, np.degrees(table[f"panel_{panel}_desired_rad"]), "-",
                      label="Desired Profile" if first else None)
            axis.plot(time, np.degrees(table[f"panel_{panel}_measured_rad"]), "--",
                      label="Achieved" if first else None)
            axis.set_title(f"Panel {panel}")
            axis.set_xticks([])
            axis.set_ylabel("Angle [deg]")
            axis.set_xlabel("Time [s]")
        figure.suptitle("Panel angle")
        figure.legend(loc="lower center", ncols=2)
        figures[stem + "_angle"] = figure

        figure, axes = plt.subplots(2, 3)
        for axis, panel in zip(axes.flat, PANELS):
            axis.plot(time, table[f"panel_{panel}_torque_Nm"])
            axis.set_title(f"Panel {panel}")
            axis.set_ylabel("Torque [Nm]")
            axis.set_xlabel("Time [s]")
        figures[stem + "_torque"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the MuJoCo panel-deployment scenario.", build))
