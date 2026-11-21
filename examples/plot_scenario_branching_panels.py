#!/usr/bin/env python3
"""Upstream `scenarioBranchingPanels` figure from
`scenarioBranchingPanels.csv`: each panel's desired and achieved angle over
the three deployment legs."""

import numpy as np

from orbit_plotting import NANO2MIN, existing, run_plots

BLUE = "#004488"
RED = "#BB5566"
PANELS = ("10", "20", "1p", "2p", "1n", "2n")
LEG_MIN = 90.0


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, ["scenarioBranchingPanels"]):
        time = table.time_ns * NANO2MIN
        figure, axes = plt.subplots(ncols=2, nrows=3, sharex="all", sharey="all", squeeze=False)
        for axis, panel in zip(axes.flat, PANELS):
            first = axis is axes.flat[0]
            axis.plot(time, np.degrees(table[f"panel_{panel}_desired_rad"]), "-", color=BLUE, lw=2,
                      label="Desired Profile" if first else None)
            axis.plot(time, np.degrees(table[f"panel_{panel}_measured_rad"]), "--", color=RED, lw=2,
                      label="Achieved" if first else None)
            for line in (1, 2):
                axis.axvline(line * LEG_MIN, color="k", linestyle=":")
            axis.set_title(f"Panel {panel}")
            axis.set_yticks([0, 45, 90, 135, 180])
            if axis in axes[:, 0]:
                axis.set_ylabel("Angle [deg]")
            if axis in axes[-1, :]:
                axis.set_xlabel("Time [min]")
        figure.suptitle("Panel angle")
        figure.legend(loc="lower center", ncols=2)
        figures[stem + "_angle"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the MuJoCo branching-panels scenario.", build))
