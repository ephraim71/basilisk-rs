#!/usr/bin/env python3
"""Upstream `scenarioMomentumDumpingMuJoCo` figures from
`scenarioMomentumDumpingMuJoCo.csv`: attitude and rate errors, wheel momenta,
the momentum-change request, wheel speeds, thruster impulses, on-time
requests and delivered thruster forces."""

import numpy as np

from orbit_plotting import NANO2MIN, existing, line_color, run_plots

STEM = "scenarioMomentumDumpingMuJoCo"
RPM = 2.0 * np.pi / 60.0
# Honeywell HR16 at 100 N m s: Js = 100 / 6000 rpm; upstream's canted axes.
JS = 100.0 / (6000.0 * RPM)
C = 2.0 ** -0.5
AXES = np.array([[C, 0, C], [0, C, C], [-C, 0, C], [0, -C, C]])
WHEELS = 4
THRUSTERS = 8


def series(axis, time, values, label, plt):
    count = values.shape[1]
    for index in range(count):
        axis.plot(time, values[:, index], color=line_color(index, count, plt),
                  label=label.format(i=index + 1))
    axis.legend(loc="lower right")


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, [STEM]):
        minutes = table.time_ns * NANO2MIN
        columns = lambda name, count: np.column_stack([table[f"{name}_{i}"] for i in range(count)])

        for key, prefix, ylabel, label in (
            ("_attError", "sigma_BR", r"Attitude Error $\sigma_{B/R}$", r"$\sigma_{i}$"),
            ("_rateError", "omega_BR_B_rad_per_s", "Rate Tracking Error (rad/s) ", r"$\omega_{{BR,{i}}}$"),
            ("_DH", "deltaH_Nms", "Requested momentum change [N m s]", r"$\Delta H_{{{i}}}$"),
        ):
            figure, axis = plt.subplots()
            series(axis, minutes, table.vectors(prefix), label, plt)
            axis.set_xlabel("Time [min]")
            axis.set_ylabel(ylabel)
            figures[stem + key] = figure

        speeds = columns("wheelSpeeds_rad_per_s", WHEELS)
        figure, axis = plt.subplots()
        series(axis, minutes, speeds * JS, r"$H_{{{i}}}$", plt)
        axis.plot(minutes, np.linalg.norm(speeds @ (JS * AXES), axis=1), "--", label=r"$\|H\|$")
        axis.legend(loc="lower right")
        axis.set_xlabel("Time [min]")
        axis.set_ylabel("RW Momentum (Nms)")
        figures[stem + "_rwMomenta"] = figure

        figure, axis = plt.subplots()
        series(axis, minutes, speeds / RPM, r"$\Omega_{{{i}}}$", plt)
        axis.set_xlabel("Time [min]")
        axis.set_ylabel("RW Speed (RPM) ")
        figures[stem + "_rwSpeeds"] = figure

        for key, name, ylabel, label in (
            ("_thrImpulse", "thrImpulse_Ns", "Impulse requested [Ns]", "$thrImpulse_{{{i}}}$"),
            ("_OnTimeReq", "OnTimeRequest_s", "OnTimeRequest [sec]", "$OnTimeRequest_{{{i}}}$"),
            ("_thrForce", "thrForce_N", "Thruster force [N]", "$thrForce_{{{i}}}$"),
        ):
            figure, axis = plt.subplots()
            series(axis, minutes, columns(name, THRUSTERS), label, plt)
            axis.set_xlabel("Time [min]")
            axis.set_ylabel(ylabel)
            figures[stem + key] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the MuJoCo momentum-dumping scenario.", build))
