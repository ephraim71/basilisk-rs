#!/usr/bin/env python3
"""Upstream `scenarioDeployingPanel` figures from `scenarioDeployingPanel.csv`:
the hub attitude and rate, each panel's angle and rate, and the panel power."""

from orbit_plotting import NANO2SEC, existing, plot_components, run_plots

STEM = "scenarioDeployingPanel"


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        seconds = table.time_ns * NANO2SEC
        for number, prefix, label, ylabel in (
            (1, "sigma_BN", r"$\sigma_{i}$", r"MRP Attitude $\sigma_{B/N}$"),
            (2, "omega_BN_B_rad_per_s", r"$\omega_{i}$", r"Rate $\omega_{B/N}$"),
        ):
            figure = plt.figure()
            axis = figure.gca()
            axis.ticklabel_format(useOffset=False, style="plain")
            plot_components(axis, seconds, table.vectors(prefix), label, plt)
            axis.set_xlabel("Time [s]")
            axis.set_ylabel(ylabel)
            figures[f"{STEM}{number}"] = figure

        for panel in (1, 2):
            figure = plt.figure()
            left = figure.add_subplot(111)
            left.plot(seconds, table[f"panel{panel}_theta_rad"], color="royalblue")
            left.set_xlabel("Time [s]")
            left.set_ylabel(f"Panel {panel} Angle [rad]", color="royalblue")
            right = figure.add_subplot(111, sharex=left, frameon=False)
            right.plot(seconds, table[f"panel{panel}_thetaDot_rad_per_s"], color="indianred")
            right.yaxis.tick_right()
            right.yaxis.set_label_position("right")
            right.set_ylabel(f"Panel {panel} Angle Rate [rad/s]", color="indianred")
            figures[f"{STEM}{panel + 2}"] = figure

        figure = plt.figure()
        axis = figure.gca()
        axis.plot(seconds, table["panel1_netPower_W"], color="goldenrod", label="Panel 1")
        axis.plot(seconds, table["panel2_netPower_W"], "--", color="goldenrod", label="Panel 2")
        axis.set_xlabel("Time [s]")
        axis.set_ylabel("Panel Power [W]")
        axis.legend(loc="lower right")
        figures[f"{STEM}5"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the deploying panel scenario.", build))
