#!/usr/bin/env python3
"""Upstream `scenarioThrusterArm` figures from `scenarioThrusterArm_branched.csv`
and `scenarioThrusterArm_direct.csv`: the joint angles and rates of both
cases overlaid, and the hub velocity and angular velocity differences."""

import numpy as np

from orbit_plotting import NANO2SEC, PlotError, Table, line_color, run_plots

STEM = "scenarioThrusterArm"


def build(generated_dir, plt):
    paths = [generated_dir / f"{STEM}_{case}.csv" for case in ("branched", "direct")]
    if not all(path.is_file() for path in paths):
        return {}
    branched, direct = (Table(path) for path in paths)
    if len(branched.time_ns) != len(direct.time_ns):
        raise PlotError("the branched and hub-direct runs have different lengths")
    seconds = branched.time_ns * NANO2SEC
    figures = {}
    for name, column, symbol, ylabel in (
        ("theta", "theta{}_rad", r"\theta", r"$\theta$ [deg]"),
        ("thetaDot", "theta{}Dot_rad_per_s", r"\dot{\theta}", r"$\dot{\theta}$ [deg/s]"),
    ):
        figure = plt.figure()
        axis = figure.gca()
        for joint, color in ((1, "C0"), (2, "C1")):
            axis.plot(seconds, np.degrees(branched[column.format(joint)]), f"{color}-",
                      label=rf"${symbol}_{joint}$ branched")
            axis.plot(seconds, np.degrees(direct[column.format(joint)]), f"{color}--",
                      label=rf"${symbol}_{joint}$ hub-direct")
        axis.legend(loc="upper right")
        axis.set_xlabel("time [s]")
        axis.set_ylabel(ylabel)
        figures[STEM + name] = figure

    for name, prefix, label, ylabel in (
        ("velocityDiff", "v_BN_N_m_per_s", r"$\Delta v_{{BN,{i}}}$", r"$v_{BN}^{\rm branched} - v_{BN}^{\rm direct}$ [m/s]"),
        ("angularVelocityDiff", "omega_BN_B_rad_per_s", r"$\Delta \omega_{{BN,{i}}}$",
         r"$\omega_{BN}^{\rm branched} - \omega_{BN}^{\rm direct}$ [rad/s]"),
    ):
        figure = plt.figure()
        axis = figure.gca()
        difference = branched.vectors(prefix) - direct.vectors(prefix)
        for index in range(3):
            axis.plot(seconds, difference[:, index], color=line_color(index, 3, plt),
                      label=label.format(i=index))
        axis.legend()
        axis.set_xlabel("time [s]")
        axis.set_ylabel(ylabel)
        figures[STEM + name] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the thruster arm branching scenario.", build))
