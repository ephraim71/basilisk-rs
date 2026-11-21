#!/usr/bin/env python3
"""Upstream `scenarioFlexiblePanelMuJoCo` figures from
`scenarioFlexiblePanelMuJoCo.csv` (bending and twisting angles and rates)
and `scenarioFlexiblePanelMuJoCoAttError.csv` (attitude and rate errors)."""

from orbit_plotting import NANO2MIN, existing, line_color, run_plots

STEM = "scenarioFlexiblePanelMuJoCo"
SEGMENTS = 5


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, [STEM]):
        minutes = table.time_ns * NANO2MIN
        for key, column, label in (
            ("_thetas", "theta_{n}_rad", r"$\theta_{n}$ [rad]"),
            ("_betas", "beta_{n}_rad", r"$\beta_{n}$ [rad]"),
            ("_thetaDots", "thetaDot_{n}_rad_per_s", r"$\dot\theta_{n}$ [rad/s]"),
            ("_betaDots", "betaDot_{n}_rad_per_s", r"$\dot\beta_{n}$ [rad/s]"),
        ):
            figure, axis = plt.subplots()
            for n in range(1, SEGMENTS + 1):
                axis.plot(minutes, table[column.format(n=n)], color=line_color(n - 1, SEGMENTS, plt),
                          label=label.format(n=n).split(" [")[0])
            axis.legend(loc="lower right")
            axis.set_xlabel("Time [min]")
            axis.set_ylabel(label.format(n="").replace("_", ""))
            figures[stem + key] = figure
    for stem, table in existing(generated_dir, [STEM + "AttError"]):
        minutes = table.time_ns * NANO2MIN
        for key, prefix, label, title in (
            ("_sigma_BR", "sigma_BR", r"$\sigma_{B/R}$", "Attitude Error"),
            ("_omega_BR_B", "omega_BR_B_rad_per_s", r"$\omega_{B/R}$ [rad/s]", "Attitude Error Rate"),
        ):
            values = table.vectors(prefix)
            figure, axis = plt.subplots()
            for index in range(3):
                axis.plot(minutes, values[:, index], color=line_color(index, 3, plt),
                          label=f"{index + 1}")
            axis.legend(loc="lower right")
            axis.set_xlabel("Time [min]")
            axis.set_ylabel(label)
            axis.set_title(title)
            figures[STEM + key] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the MuJoCo flexible-panel scenario.", build))
