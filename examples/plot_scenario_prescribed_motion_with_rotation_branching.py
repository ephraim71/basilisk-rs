#!/usr/bin/env python3
"""Upstream `scenarioPrescribedMotionWithRotationBranching` figures from
`scenarioPrescribedMotionWithRotationBranching.csv`: the trusses' prescribed
angles, rates and accelerations, the panels' angles and rates, and the hub's
position, attitude and rate."""

import numpy as np

from orbit_plotting import NANO2SEC, existing, run_plots

STEM = "scenarioPrescribedMotionWithRotationBranching"
COLORS = ("teal", "darkviolet", "blue")


def _figure(plt, name, title, ylabel, curves, legend):
    figure = plt.figure()
    axis = figure.gca()
    for (time, values, label), color in zip(curves, COLORS):
        axis.plot(time, values, label=label, color=color)
    axis.set_title(title, fontsize=16)
    axis.set_ylabel(ylabel, fontsize=16)
    axis.set_xlabel("Time (s)", fontsize=16)
    axis.legend(loc=legend, prop={"size": 16})
    axis.grid(True)
    return name, figure


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        t = table.time_ns * NANO2SEC
        axis_m = np.array([1.0, 0.0, 0.0])
        rate = [np.degrees(table.vectors(f"omega_P{k}M_P{k}_rad_per_s") @ axis_m) for k in (1, 2)]
        accel = [np.degrees(table.vectors(f"omegaPrime_P{k}M_P{k}_rad_per_s2") @ axis_m)
                 for k in (1, 2)]
        r, sigma = table.vectors("r_BN_N_m"), table.vectors("sigma_BN")
        omega = np.degrees(table.vectors("omega_BN_B_rad_per_s"))
        for name, figure in (
            _figure(plt, STEM + "_1", "Prescribed Truss Angles", "(deg)",
                    [(t, np.degrees(table["theta_P1_rad"]), r"$\theta_{P_1}$"),
                     (t, np.degrees(table["theta_P2_rad"]), r"$\theta_{P_2}$")], "center right"),
            _figure(plt, STEM + "_2", "Prescribed Truss Angle Rates", "(deg/s)",
                    [(t, rate[0], r"$\dot{\theta}_{P_1}$"),
                     (t, rate[1], r"$\dot{\theta}_{P_2}$")], "upper right"),
            _figure(plt, STEM + "_3", "Prescribed Truss Accelerations", "(deg/s$^2$)",
                    [(t, accel[0], r"$\ddot{\theta}_{P_1}$"),
                     (t, accel[1], r"$\ddot{\theta}_{P_2}$")], "upper right"),
            _figure(plt, STEM + "_4", "Solar Array Angles", "(deg)",
                    [(t, np.degrees(table["theta_S1_rad"]), r"$\theta_{S_{1,2,5,6}}$"),
                     (t, np.degrees(table["theta_S3_rad"]), r"$\theta_{S_{3,4,7,8}}$")],
                    "upper right"),
            _figure(plt, STEM + "_5", "Solar Array Angle Rates", "(deg/s)",
                    [(t, np.degrees(table["thetaDot_S1_rad_per_s"]), r"$\dot{\theta}_{S_{1,2,5,6}}$"),
                     (t, np.degrees(table["thetaDot_S3_rad_per_s"]), r"$\dot{\theta}_{S_{3,4,7,8}}$")],
                    "upper right"),
            _figure(plt, STEM + "_6", r"Hub Inertial Position ${}^\mathcal{N} r_{B/N}$", "(m)",
                    [(t, r[:, i], rf"$r_{i + 1}$") for i in range(3)], "center left"),
            _figure(plt, STEM + "_7", r"Hub Inertial Attitude $\sigma_{\mathcal{B}/\mathcal{N}}$", "",
                    [(t, sigma[:, i], rf"$\sigma_{i + 1}$") for i in range(3)], "center right"),
            _figure(plt, STEM + "_8",
                    r"Hub Inertial Angular Velocity ${}^\mathcal{B} \omega_{\mathcal{B}/\mathcal{N}}$",
                    "(deg/s)", [(t, omega[:, i], rf"$\omega_{i + 1}$") for i in range(3)],
                    "center right"),
        ):
            figures[name] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the prescribed motion with rotation branching scenario.", build))
