#!/usr/bin/env python3
"""Upstream `scenarioConstrainedDynamics` figures from
`scenarioConstrainedDynamics<env>.csv`: the direction and attitude
constraint violations of the two hubs."""

import numpy as np

from multibody_plotting import violation_angles_deg, violations
from orbit_plotting import NANO2SEC, existing, run_plots

STEM = "scenarioConstrainedDynamics"
COM_OFFSET_M = 0.1
ARM_LENGTH_M = 0.1


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, [STEM + "Gravity", STEM + "NoGravity"]):
        time = table.time_ns * NANO2SEC
        # The arm lies along the initial hub offset (the frames start aligned).
        offset = table.vectors("r_B2N_N_m")[0] - table.vectors("r_B1N_N_m")[0]
        direction = offset / np.linalg.norm(offset)
        psi, sigma = violations(table, COM_OFFSET_M * direction, -COM_OFFSET_M * direction,
                                ARM_LENGTH_M * direction)
        suffix = "" if stem == STEM + "Gravity" else "NoGravity"

        figure = plt.figure()
        axis = figure.gca()
        for index in range(3):
            axis.semilogy(time, np.abs(psi[:, index]))
        axis.semilogy(time, np.linalg.norm(psi, axis=1))
        axis.legend([r"$\psi_1$", r"$\psi_2$", r"$\psi_3$", r"$\psi$ magnitude"])
        axis.set_xlabel("time (seconds)")
        axis.set_ylabel(r"relative connection position: $\psi$ (meters)")
        axis.set_title("Direction Constraint Violation Components")
        figures[STEM + suffix + "directionConstraint"] = figure

        angles = violation_angles_deg(sigma)
        figure = plt.figure()
        axis = figure.gca()
        for index in range(3):
            axis.semilogy(time, np.abs(angles[:, index]))
        axis.semilogy(time, np.linalg.norm(angles, axis=1))
        axis.legend([r"$\phi_1$", r"$\phi_2$", r"$\phi_3$", r"$\phi$ magnitude"])
        axis.set_xlabel("time (seconds)")
        axis.set_ylabel(r"relative attitude angle: $\phi$ (deg)")
        axis.set_title("Attitude Constraint Violation Components")
        figures[STEM + suffix + "attitudeConstraint"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the constrained dynamics scenario.", build))
