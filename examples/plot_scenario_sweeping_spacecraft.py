#!/usr/bin/env python3
"""Upstream `scenarioSweepingSpacecraft` figures from
`scenarioSweepingSpacecraft<alt>.csv`: the attitude error norm, the control
torque, the rate error and the body orientation relative to the Hill frame."""

import numpy as np

from orbit_plotting import NANO2MIN, existing, line_color, run_plots

STEM = "scenarioSweepingSpacecraft"


def mrp_to_dcm(sigma):
    """Upstream `RigidBodyKinematics.MRP2C`: `[BN]` from `sigma_BN`."""
    s2 = float(np.dot(sigma, sigma))
    tilde = np.array([[0.0, -sigma[2], sigma[1]], [sigma[2], 0.0, -sigma[0]],
                      [-sigma[1], sigma[0], 0.0]])
    return np.eye(3) + (8.0 * tilde @ tilde - 4.0 * (1.0 - s2) * tilde) / (1.0 + s2) ** 2


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, [f"{STEM}0", f"{STEM}1"]):
        alt = stem[-1]
        minutes = table.time_ns * NANO2MIN

        figure = plt.figure()
        axis = figure.gca()
        axis.plot(minutes, np.linalg.norm(table.vectors("sigma_BR"), axis=1),
                  color=line_color(1, 3, plt))
        axis.set_xlabel("Time [min]")
        axis.set_ylabel(r"Attitude Error Norm $|\sigma_{B/R}|$")
        axis.set_yscale("log")
        figures[f"{STEM}1{alt}"] = figure

        figure = plt.figure()
        axis = figure.gca()
        torque = table.vectors("torqueRequestBody_Nm")
        for index in range(3):
            axis.plot(minutes, torque[:, index], color=line_color(index, 3, plt),
                      label=f"$L_{{r,{index}}}$")
        axis.legend(loc="lower right")
        axis.set_xlabel("Time [min]")
        axis.set_ylabel("Control Torque $L_r$ [Nm]")
        figures[f"{STEM}2{alt}"] = figure

        # Upstream draws the rate error but does not save it.
        figure = plt.figure()
        axis = figure.gca()
        omega = table.vectors("omega_BR_B_rad_per_s")
        for index in range(3):
            axis.plot(minutes, omega[:, index], color=line_color(index, 3, plt),
                      label=rf"$\omega_{{BR,{index}}}$")
        axis.legend(loc="lower right")
        axis.set_xlabel("Time [min]")
        axis.set_ylabel("Rate Tracking Error [rad/s] ")
        plt.close(figure)

        positions = table.vectors("r_BN_N_m")
        velocities = table.vectors("v_BN_N_m_per_s")
        sigmas = table.vectors("sigma_BN")
        data = np.empty((len(positions), 3))
        for index, (r, v, sigma) in enumerate(zip(positions, velocities, sigmas, strict=True)):
            i_r = r / np.linalg.norm(r)
            h = np.cross(r, v)
            i_h = h / np.linalg.norm(h)
            i_theta = np.cross(i_h, i_r)
            dcm_bn = mrp_to_dcm(sigma)
            data[index] = [np.dot(i_r, dcm_bn[0]), np.dot(i_theta, dcm_bn[1]), np.dot(i_h, dcm_bn[2])]
        figure = plt.figure()
        axis = figure.gca()
        labels = (r"$\hat\imath_r\cdot \hat b_1$", r"${\hat\imath}_{\theta}\cdot \hat b_2$",
                  r"$\hat\imath_h\cdot \hat b_3$")
        for index in range(3):
            axis.plot(minutes, data[:, index], color=line_color(index, 3, plt), label=labels[index])
        axis.legend(loc="lower right")
        axis.set_xlabel("Time [min]")
        axis.set_ylabel("Orientation Illustration")
        figures[f"{STEM}4{alt}"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the sweeping spacecraft scenario.", build))
