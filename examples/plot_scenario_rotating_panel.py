#!/usr/bin/env python3
"""Upstream `scenarioRotatingPanel` figures from `scenarioRotatingPanel.csv`:
the hub attitude, the panel angle against the panel power, and the panel
angle against the two coarse Sun sensor signals."""

import numpy as np

from orbit_plotting import NANO2MIN, existing, plot_components, run_plots

STEM = "scenarioRotatingPanel"


def angle_axes(plt, figure, minutes, theta_deg):
    """The dashed panel angle (mod 360) on the left axis and a frameless
    twin sharing its time axis, upstream's two-axis layout."""
    left = figure.add_subplot(111)
    left.plot(minutes, theta_deg, "--", color="royalblue")
    left.set_yticks([0, 90, 180, 270, 360])
    left.set_xlabel("Time [min]")
    left.set_ylabel("Panel Angle [deg]", color="royalblue")
    right = figure.add_subplot(111, sharex=left, frameon=False)
    right.yaxis.tick_right()
    right.yaxis.set_label_position("right")
    return left, right


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        minutes = table.time_ns * NANO2MIN
        theta_deg = np.degrees(table["panel1_theta_rad"]) % 360

        figure = plt.figure()
        axis = figure.gca()
        axis.ticklabel_format(useOffset=False, style="plain")
        plot_components(axis, minutes, table.vectors("sigma_BN"), r"$\sigma_{i}$", plt)
        axis.set_xlabel("Time [min]")
        axis.set_ylabel(r"MRP Attitude $\sigma_{B/N}$")
        figures[STEM + "1"] = figure

        figure = plt.figure()
        left, right = angle_axes(plt, figure, minutes, theta_deg)
        left.fill_between(minutes, 0, 90, facecolor="gold")
        left.fill_between(minutes, 270, 360, facecolor="gold")
        right.plot(minutes, table["netPower_W"], color="goldenrod")
        right.set_ylabel("Solar Panel Power [W]", color="goldenrod")
        figures[STEM + "2panel1theta"] = figure

        figure = plt.figure()
        left, right = angle_axes(plt, figure, minutes, theta_deg)
        right.plot(minutes, table["CSS1_OutputData"], color="tab:pink", label=r"CSS$_1$")
        right.plot(minutes, table["CSS2_OutputData"], color="tab:olive", label=r"CSS$_2$")
        left.fill_between(minutes, 225, 315, facecolor="pink")
        left.fill_between(minutes, 315, 360, facecolor="palegoldenrod")
        left.fill_between(minutes, 0, 45, facecolor="palegoldenrod")
        right.legend(loc="lower right")
        right.set_ylabel("CSS Signals")
        figures[STEM + "3"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the rotating panel scenario.", build))
