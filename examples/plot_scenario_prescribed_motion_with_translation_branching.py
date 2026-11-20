#!/usr/bin/env python3
"""Upstream `scenarioPrescribedMotionWithTranslationBranching` figures from
`scenarioPrescribedMotionWithTranslationBranching.csv`: the platform's
prescribed angle and displacement with their rates and accelerations, the
struts' displacements and rates, and the hub's position, attitude and rate."""

import numpy as np

from orbit_plotting import NANO2SEC, existing, run_plots

STEM = "scenarioPrescribedMotionWithTranslationBranching"
ROT_AXIS_M = np.array([0.0, 1.0, 0.0])
TRANS_AXIS_M = np.array([0.0, 0.0, -1.0])
COLORS = ("teal", "darkviolet", "blue")


def _twin(plt, t, rotation, translation, labels, units, legend, skip_first=False):
    figure, axis = plt.subplots()
    axis.plot(t, rotation, label=labels[0], color="teal")
    axis.tick_params(axis="y", labelcolor="teal")
    axis.set_xlabel("Time (s)", fontsize=16)
    axis.set_ylabel(units[0], color="teal", fontsize=16)
    other = axis.twinx()
    start = 1 if skip_first else 0
    other.plot(t[start:], 100.0 * translation[start:], label=labels[1], color="darkviolet")
    other.set_ylabel(units[1], color="darkviolet", fontsize=16)
    other.tick_params(axis="y", labelcolor="darkviolet")
    handles = axis.get_legend_handles_labels()[0] + other.get_legend_handles_labels()[0]
    other.legend(handles=handles, labels=[h.get_label() for h in handles], loc=legend,
                 prop={"size": 16})
    axis.grid(True)
    return figure


def _lines(plt, t, curves, title, ylabel, legend):
    figure = plt.figure()
    axis = figure.gca()
    for (values, label), color in zip(curves, COLORS):
        axis.plot(t, values, label=label, color=color)
    axis.set_title(title, fontsize=16)
    axis.set_ylabel(ylabel, fontsize=16)
    axis.set_xlabel("Time (s)", fontsize=16)
    axis.legend(loc=legend, prop={"size": 16})
    axis.grid(True)
    return figure


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        t = table.time_ns * NANO2SEC
        theta = np.degrees(table["theta_P_rad"])
        rho = table.vectors("r_PM_M_m") @ TRANS_AXIS_M
        theta_dot = np.degrees(table.vectors("omega_PM_P_rad_per_s") @ ROT_AXIS_M)
        rho_dot = table.vectors("rPrime_PM_M_m_per_s") @ TRANS_AXIS_M
        theta_ddot = np.degrees(table.vectors("omegaPrime_PM_P_rad_per_s2") @ ROT_AXIS_M)
        rho_ddot = table.vectors("rPrimePrime_PM_M_m_per_s2") @ TRANS_AXIS_M
        r, sigma = table.vectors("r_BN_N_m"), table.vectors("sigma_BN")
        omega = np.degrees(table.vectors("omega_BN_B_rad_per_s"))
        figures[STEM + "_1"] = _twin(plt, t, theta, rho, (r"$\theta_{P}$", r"$\rho_{P}$"),
                                     ("(deg)", "(cm)"), "center right", skip_first=True)
        figures[STEM + "_2"] = _twin(plt, t, theta_dot, rho_dot,
                                     (r"$\dot{\theta}_{P}$", r"$\dot{\rho}_{P}$"),
                                     ("(deg/s)", "(cm/s)"), "center right")
        figures[STEM + "_3"] = _twin(plt, t, theta_ddot, rho_ddot,
                                     (r"$\ddot{\theta}_{P}$", r"$\ddot{\rho}_{P}$"),
                                     ("(deg/$s^2$)", "(cm/$s^2$)"), "upper right")
        figures[STEM + "_4"] = _lines(
            plt, t, [(100.0 * table[f"rho_T{k}_m"], rf"$\rho_{{T_{k}}}$") for k in (1, 2, 3)],
            "Telescoping Strut Displacements", "(cm)", "center left")
        figures[STEM + "_5"] = _lines(
            plt, t, [(100.0 * table[f"rhoDot_T{k}_m_per_s"], rf"$\dot{{\rho}}_{{T_{k}}}$")
                     for k in (1, 2, 3)],
            "Telescoping Strut Displacement Rates", "(cm/s)", "upper right")
        figures[STEM + "_6"] = _lines(plt, t, [(r[:, i], rf"$r_{i + 1}$") for i in range(3)],
                                      r"Hub Inertial Position ${}^\mathcal{N} r_{B/N}$", "(m)",
                                      "center left")
        figures[STEM + "_7"] = _lines(plt, t, [(sigma[:, i], rf"$\sigma_{i + 1}$") for i in range(3)],
                                      r"Hub Inertial Attitude $\sigma_{\mathcal{B}/\mathcal{N}}$", "",
                                      "center right")
        figures[STEM + "_8"] = _lines(
            plt, t, [(omega[:, i], rf"$\omega_{i + 1}$") for i in range(3)],
            r"Hub Inertial Angular Velocity ${}^\mathcal{B} \omega_{\mathcal{B}/\mathcal{N}}$",
            "(deg/s)", "center right")
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the prescribed motion with translation branching scenario.",
                               build))
