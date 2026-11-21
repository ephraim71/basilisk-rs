#!/usr/bin/env python3
"""Upstream `scenarioDeployingSolarArrays` figures from
`scenarioDeployingSolarArrays.csv`: every element's angle and rate for both
arrays, and the hub's inertial position, attitude, angular velocity and its
norm."""

import numpy as np

from orbit_plotting import NANO2MIN, existing, run_plots

STEM = "scenarioDeployingSolarArrays"
ELEMENTS = 10


def finish(axis, title, ylabel, xlabel="Time (min)", legend_size=8):
    axis.set_title(title, fontsize=16)
    axis.set_ylabel(ylabel, fontsize=14)
    axis.set_xlabel(xlabel, fontsize=14)
    if legend_size:
        axis.legend(bbox_to_anchor=(1.25, 0.5), loc="center right", prop={"size": legend_size})
    axis.grid(True)


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        minutes = table.time_ns * NANO2MIN
        for array in (1, 2):
            for name, column, symbol, title, ylabel in (
                ("ElementTheta", "theta_rad", r"\theta", "Element Angles", "(deg)"),
                ("ElementThetaDot", "thetaDot_rad_per_s", r"\dot{\theta}", "Element Angle Rates",
                 "(deg/s)"),
            ):
                figure = plt.figure()
                axis = figure.gca()
                for element in range(1, ELEMENTS + 1):
                    axis.plot(minutes, np.degrees(table[f"array{array}Element{element}_{column}"]),
                              label=rf"${symbol}_{{{element}}}$")
                finish(axis, f"Array {array} {title}", ylabel)
                figures[f"{STEM}_Array{array}{name}"] = figure

        for name, prefix, scale, symbol, title, ylabel in (
            ("HubInertialPosition", "r_BN_N_m", 1.0, "r",
             r"${}^\mathcal{N} r_{\mathcal{B}/\mathcal{N}}$ Spacecraft Inertial Trajectory", "(m)"),
            ("HubInertialMRPAttitude", "sigma_BN", 1.0, r"\sigma",
             r"$\sigma_{\mathcal{B}/\mathcal{N}}$ Spacecraft Inertial MRP Attitude", ""),
            ("HubInertialAngularVelocity", "omega_BN_B_rad_per_s", np.degrees(1.0), r"\omega",
             r"Spacecraft Hub Angular Velocity ${}^\mathcal{B} \omega_{\mathcal{B}/\mathcal{N}}$",
             "(deg/s)"),
        ):
            figure = plt.figure()
            axis = figure.gca()
            values = table.vectors(prefix) * scale
            for index in range(3):
                axis.plot(minutes, values[:, index], label=rf"${symbol}_{{{index + 1}}}$")
            finish(axis, title, ylabel, legend_size=12)
            figures[f"{STEM}_{name}"] = figure

        figure = plt.figure()
        axis = figure.gca()
        axis.plot(minutes, np.degrees(np.linalg.norm(table.vectors("omega_BN_B_rad_per_s"), axis=1)))
        finish(axis, r"Hub Angular Velocity Norm $|{}^\mathcal{B} \omega_{\mathcal{B}/\mathcal{N}}|$",
               r"(deg/s)", xlabel=r"(min)", legend_size=0)
        figures[f"{STEM}_HubInertialAngularVelocityNorm"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the deploying solar arrays scenario.", build))
