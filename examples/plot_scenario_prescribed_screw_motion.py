#!/usr/bin/env python3
"""Upstream `scenarioPrescribedScrewMotion` figures from
`scenarioPrescribedScrewMotion.csv`: the bodies' displacements against their
angles and the screw truth `rho0 + c theta`, the displacement errors, the
prescribed angles, displacements, rates and accelerations, and the hub's
position, attitude and rate."""

import numpy as np

from orbit_plotting import NANO2SEC, existing, run_plots

STEM = "scenarioPrescribedScrewMotion"
AXES = (np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0]))
SCREW_CONSTANTS = (0.1, 0.12)
INIT_DISPLACEMENT_M = 0.4


def _twin(plt, t, rotations, translations, labels, units, title):
    figure, axis = plt.subplots()
    for values, label in zip(rotations, labels[0]):
        axis.plot(t, values, label=label, color="teal")
    axis.tick_params(axis="y", labelcolor="teal")
    axis.set_xlabel("Time (s)")
    axis.set_ylabel(units[0], color="teal")
    other = axis.twinx()
    for values, label in zip(translations, labels[1]):
        other.plot(t, 100.0 * values, label=label, color="darkviolet")
    other.set_ylabel(units[1], color="darkviolet")
    other.tick_params(axis="y", labelcolor="darkviolet")
    handles = axis.get_legend_handles_labels()[0] + other.get_legend_handles_labels()[0]
    other.legend(handles=handles, labels=[h.get_label() for h in handles], loc="center left",
                 bbox_to_anchor=(1.25, 0.5))
    axis.set_title(title)
    axis.grid(True)
    return figure


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        t = table.time_ns * NANO2SEC
        theta = [np.degrees(table[f"theta_P{k}_rad"]) for k in (1, 2)]
        rho = [table.vectors(f"r_P{k}M_M_m") @ AXES[k - 1] for k in (1, 2)]
        truth = [SCREW_CONSTANTS[i] * np.radians(theta[i]) + INIT_DISPLACEMENT_M for i in (0, 1)]

        figure = plt.figure()
        axis = figure.gca()
        for i, color in ((0, "teal"), (1, "darkviolet")):
            axis.plot(theta[i][1:], rho[i][1:], label=rf"$P_{i + 1}$ Sim", color=color)
            axis.plot(theta[i][1:], truth[i][1:], "--", label=rf"$P_{i + 1}$ Truth", color=color)
        axis.set_title("Prescribed Displacements VS Angles")
        axis.set_ylabel("(m)")
        axis.set_xlabel("(deg)")
        axis.legend(loc="center left", bbox_to_anchor=(1, 0.5))
        axis.grid(True)
        figures[STEM + "_1"] = figure

        figure = plt.figure()
        axis = figure.gca()
        for i, color in ((0, "teal"), (1, "darkviolet")):
            axis.plot(t[1:], np.abs(truth[i][1:] - rho[i][1:]), label=rf"$P_{i + 1}$ Error",
                      color=color)
        axis.set_title("Prescribed Displacement Errors")
        axis.set_ylabel("(m)")
        axis.set_xlabel("(sec)")
        axis.legend(loc="center left", bbox_to_anchor=(1, 0.5))
        axis.grid(True)
        figures[STEM + "_2"] = figure

        names = ((r"$\theta_{P_1}$", r"$\theta_{P_2}$"), (r"$\rho_{P_1}$", r"$\rho_{P_2}$"))
        figures[STEM + "_3"] = _twin(plt, t, theta, rho, names, ("(deg)", "(cm)"),
                                     "Prescribed Displacements")
        rates = [np.degrees(table.vectors(f"omega_P{k}M_P{k}_rad_per_s") @ AXES[k - 1]) for k in (1, 2)]
        rho_rates = [table.vectors(f"rPrime_P{k}M_M_m_per_s") @ AXES[k - 1] for k in (1, 2)]
        figures[STEM + "_4"] = _twin(
            plt, t, rates, rho_rates,
            ((r"$\dot{\theta}_{P_1}$", r"$\dot{\theta}_{P_2}$"),
             (r"$\dot{\rho}_{P_1}$", r"$\dot{\rho}_{P_2}$")),
            ("(deg/s)", "(cm/s)"), "Prescribed Velocities")
        accels = [np.degrees(table.vectors(f"omegaPrime_P{k}M_P{k}_rad_per_s2") @ AXES[k - 1])
                  for k in (1, 2)]
        rho_accels = [table.vectors(f"rPrimePrime_P{k}M_M_m_per_s2") @ AXES[k - 1] for k in (1, 2)]
        figures[STEM + "_5"] = _twin(
            plt, t, accels, rho_accels,
            ((r"$\ddot{\theta}_{P_1}$", r"$\ddot{\theta}_{P_2}$"),
             (r"$\ddot{\rho}_{P_1}$", r"$\ddot{\rho}_{P_2}$")),
            ("(deg/$s^2$)", "(cm/$s^2$)"), "Prescribed Accelerations")

        for index, (prefix, scale, title, ylabel, label) in enumerate((
            ("r_BN_N_m", 1.0, r"Hub Inertial Position ${}^\mathcal{N} r_{B/N}$", "(m)", "r"),
            ("sigma_BN", 1.0, r"Hub Inertial Attitude $\sigma_{\mathcal{B}/\mathcal{N}}$", "", r"\sigma"),
            ("omega_BN_B_rad_per_s", np.degrees(1.0),
             r"Hub Inertial Angular Velocity ${}^\mathcal{B} \omega_{\mathcal{B}/\mathcal{N}}$",
             "(deg/s)", r"\omega"),
        )):
            figure = plt.figure()
            axis = figure.gca()
            values = scale * table.vectors(prefix)
            for i, color in enumerate(("teal", "darkviolet", "blue")):
                axis.plot(t, values[:, i], label=rf"${label}_{i + 1}$", color=color)
            axis.set_title(title)
            axis.set_ylabel(ylabel)
            axis.set_xlabel("Time (s)")
            axis.legend(loc="center left", bbox_to_anchor=(1, 0.5))
            axis.grid(True)
            figures[f"{STEM}_{6 + index}"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the prescribed screw motion scenario.", build))
