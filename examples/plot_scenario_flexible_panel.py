#!/usr/bin/env python3
"""Upstream `scenarioFlexiblePanel` figures from `scenarioFlexiblePanel<N>.csv`:
the bending and torsional angles and rates of every sub-panel, and the
attitude tracking error (at upstream's 0.5 s flight-software rate)."""

import re

import numpy as np

from orbit_plotting import NANO2MIN, line_color, run_plots, Table

STEM = "scenarioFlexiblePanel"
FSW_PERIOD_NS = 500_000_000


def build(generated_dir, plt):
    figures = {}
    for path in sorted(generated_dir.glob(f"{STEM}*.csv")):
        if not re.fullmatch(rf"{STEM}\d+", path.stem):
            continue
        table = Table(path)
        segments = sum(1 for name in table.columns if re.fullmatch(r"theta\d+_rad", name))
        minutes = table.time_ns * NANO2MIN
        for name, symbol, title, column, ylabel in (
            ("_theta", r"\theta", "Bending Angles", "theta{}_rad", r"$\theta$ [deg]"),
            ("_beta", r"\beta", "Torsional Angles", "beta{}_rad", r"$\beta$ [deg]"),
            ("_thetaDot", r"\dot{\theta}", "Bending Angle Rates", "theta{}Dot_rad_per_s",
             r"$\dot{\theta}$ [deg/s]"),
            ("_betaDot", r"\dot{\beta}", "Torsional Angle Rates", "beta{}Dot_rad_per_s",
             r"$\dot{\beta}$ [deg/s]"),
        ):
            figure = plt.figure()
            axis = figure.gca()
            for index in range(1, segments + 1):
                axis.plot(minutes, np.degrees(table[column.format(index)]),
                          label=rf"${symbol}_{index}$")
            axis.legend(fontsize="14")
            axis.set_title(title, fontsize="22")
            axis.set_xlabel("time [min]", fontsize="18")
            axis.set_ylabel(ylabel, fontsize="18")
            figures[STEM + name] = figure

        fsw = table.time_ns.astype(np.int64) % FSW_PERIOD_NS == 0
        for name, prefix, symbol, title, ylabel in (
            ("_sigma_BR", "sigma_BR", r"\sigma", "Attitude Error", r"$\sigma_{B/R}$"),
            ("_omega_BR_B", "omega_BR_B_rad_per_s", r"\omega", "Attitude Error Rate",
             r"$\omega_{B/R}$"),
        ):
            figure = plt.figure()
            axis = figure.gca()
            values = table.vectors(prefix)[fsw]
            for index in range(3):
                axis.plot(minutes[fsw], values[:, index], color=line_color(index, 3, plt),
                          label=rf"${symbol}_{index}$")
            axis.legend(fontsize="14")
            axis.set_title(title, fontsize="22")
            axis.set_xlabel("time [min]", fontsize="18")
            axis.set_ylabel(ylabel, fontsize="18")
            figures[STEM + name] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the flexible panel slew scenario.", build))
