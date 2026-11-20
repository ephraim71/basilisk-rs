#!/usr/bin/env python3
"""Upstream ``scenarioSmallBodyNavUKF`` figures from
``scenarioSmallBodyNavUKF.csv``: the trajectory in the inertial and Vesta-fixed
frames, the Vesta-fixed position, velocity and non-Keplerian acceleration
(truth and estimate), and their estimation errors with ``2 sigma`` bounds."""

import numpy as np

from orbit_plotting import NANO2SEC, existing, run_plots
from small_body_plotting import columns, component_figure, error_figure

STEM = "scenarioSmallBodyNavUKF"
DAY = 24 * 3600.0


def trajectory_figure(plt, positions, frame):
    """Upstream ``plot_3Dposition``."""
    figure = plt.figure()
    axis = figure.add_subplot(1, 1, 1, projection="3d")
    axis.plot(positions[:, 0] / 1000.0, positions[:, 1] / 1000.0, positions[:, 2] / 1000.0, "b")
    for setter, name in zip((axis.set_xlabel, axis.set_ylabel, axis.set_zlabel), "xyz"):
        setter(f"${{}}^{{{frame}}}r_{{{name}}}$ [km]")
    axis.set_title("Inertial frame" if frame == "N" else "Small body fixed frame")
    return figure


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        days = table.time_ns * NANO2SEC / DAY
        r_truth_n = table.vectors("r_BN_N_m") - table.vectors("vesta_r_BdyZero_N_m")
        r_truth = table.vectors("r_truth_A_m")
        v_truth = table.vectors("v_truth_A_m_per_s")
        a_truth = table.vectors("a_truth_A_m_per_s2")
        x_hat = columns(table, "x_hat", range(9))
        sigma = np.sqrt(columns(table, "P", range(9)))
        truth_style = {"fmt": "b", "label": "truth"}
        estimate_style = {"color": "orange", "label": "estimate"}

        figures[f"{STEM}1"] = trajectory_figure(plt, r_truth_n, "N")
        figures[f"{STEM}2"] = trajectory_figure(plt, r_truth, "A")
        figures[f"{STEM}3"] = component_figure(
            plt,
            [(days, r_truth / 1000, truth_style), (days, x_hat[:, 0:3] / 1000, estimate_style)],
            [f"${{}}^{{A}}r_{{{axis}}}$ [km]" for axis in "xyz"],
            "Time [days]", "Spacecraft Position",
        )
        figures[f"{STEM}4"] = component_figure(
            plt,
            [(days, v_truth, truth_style), (days, x_hat[:, 3:6], estimate_style)],
            [f"${{}}^{{A}}v_{{{axis}}}$ [m/s]" for axis in "xyz"],
            "Time [days]", "Spacecraft Velocity",
        )
        figures[f"{STEM}5"] = component_figure(
            plt,
            [(days, a_truth * 1000, truth_style), (days, x_hat[:, 6:9] * 1000, estimate_style)],
            [f"${{}}^{{A}}a_{{{axis}}}$ [mm/s$^2$]" for axis in "xyz"],
            "Time [days]", "Inhomogeneous gravity acceleration",
        )
        figures[f"{STEM}6"] = error_figure(
            plt, days, r_truth - x_hat[:, 0:3], sigma[:, 0:3],
            [f"${{}}^{{A}}r_{{{axis}}}$ [m]" for axis in "xyz"],
            "Time [days]", "Position Error and Covariance", color="b",
        )
        figures[f"{STEM}7"] = error_figure(
            plt, days, v_truth - x_hat[:, 3:6], sigma[:, 3:6],
            [f"${{}}^{{A}}v_{{{axis}}}$ [m/s]" for axis in "xyz"],
            "Time [days]", "Velocity Error and Covariance", color="b",
        )
        figures[f"{STEM}8"] = error_figure(
            plt, days, (a_truth - x_hat[:, 6:9]) * 1000, sigma[:, 6:9] * 1000,
            [f"${{}}^{{A}}a_{{{axis}}}$ [mm/s$^2$]" for axis in "xyz"],
            "Time [days]", "Acceleration Error and Covariance", color="b",
        )
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(f"Plot the {STEM} scenario.", build))
