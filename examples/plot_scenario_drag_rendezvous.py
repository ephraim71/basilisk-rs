#!/usr/bin/env python3
"""Upstream `scenarioDragRendezvous` figures from `scenarioDragRendezvous.csv`
(`--stem` picks another run): the Hill-frame position and velocity
components, the densities, the in-plane Hill trajectory, the relative
attitude and the drag forces."""

import sys

import numpy as np

from orbit_plotting import existing, run_plots

STEM = "scenarioDragRendezvous"


def sub_mrp(q1, q2):
    """Upstream `RigidBodyKinematics.subMRP`: the MRP of `q1` relative to
    `q2`, switched to the short rotation."""
    q1, q2 = np.asarray(q1, dtype=float), np.asarray(q2, dtype=float)
    den = 1 + q1.dot(q1) * q2.dot(q2) + 2 * q1.dot(q2)
    if abs(den) < 0.1:
        q2 = -q2 / q2.dot(q2)
        den = 1 + q1.dot(q1) * q2.dot(q2) + 2 * q1.dot(q2)
    q = ((1 - q2.dot(q2)) * q1 - (1 - q1.dot(q1)) * q2 + 2 * np.cross(q1, q2)) / den
    return -q / q.dot(q) if q.dot(q) > 1 else q


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        time = table.time_ns * 1e-9
        hill_r, hill_v = table.vectors("r_DC_H_m"), table.vectors("v_DC_H_m_per_s")
        for name, values, label in (
            ("_hillX", hill_r[:, 0], "Hill X Position (m)"),
            ("_hillY", hill_r[:, 1], "Hill Y Position (m)"),
            ("_hilldX", hill_v[:, 0], "Hill X Velocity (m/s)"),
            ("_hilldy", hill_v[:, 1], "Hill Y Velocity (m/s)"),
        ):
            figure = plt.figure()
            axis = figure.gca()
            axis.plot(time[1:], values[1:])
            axis.grid()
            axis.set_xlabel("Time")
            axis.set_ylabel(label)
            figures[STEM + name] = figure

        figure = plt.figure()
        axis = figure.gca()
        axis.semilogy(time[1:], table["chiefDensity_kg_per_m3"][1:], label=r"Chief $\rho$")
        axis.semilogy(time[1:], table["depDensity_kg_per_m3"][1:], label=r"Deputy $\rho$")
        axis.grid()
        axis.legend()
        axis.set_xlabel("Time")
        axis.set_ylabel("Density (kg/m3)")
        figures[STEM + "_densities"] = figure

        figure = plt.figure()
        axis = figure.gca()
        axis.plot(hill_r[1:, 0], hill_r[1:, 1])
        axis.grid()
        axis.set_xlabel("Hill X (m)")
        axis.set_ylabel("Hill Y (m)")
        figures[STEM + "_hillTraj"] = figure

        chief, deputy = table.vectors("wiggum_sigma_BN"), table.vectors("lou_sigma_BN")
        relative = np.array([sub_mrp(d, c) for d, c in zip(deputy, chief, strict=True)])
        figure = plt.figure()
        axis = figure.gca()
        axis.plot(time, relative)
        axis.grid()
        axis.set_xlabel("Time")
        axis.set_ylabel("Relative MRP Value")
        figures[STEM + "_relativeAtt"] = figure

        chief_drag, dep_drag = table.vectors("chiefDrag_B_N"), table.vectors("depDrag_B_N")
        for name, values, prefix in (
            ("_relativeDrag", dep_drag - chief_drag, "delta a"),
            ("_chiefDrag", chief_drag, "chief a"),
            ("_depDrag", dep_drag, "dep a"),
        ):
            figure = plt.figure()
            axis = figure.gca()
            for index in range(3):
                axis.plot(time[1:], values[1:, index], label=f"{prefix}_{index + 1}")
            axis.grid()
            axis.legend()
            axis.set_xlabel("Time")
            axis.set_ylabel("Relative acceleration due to drag, body frame (m/s)")
            figures[STEM + name] = figure
    return figures


if __name__ == "__main__":
    if "--stem" in sys.argv:
        index = sys.argv.index("--stem")
        STEM = sys.argv[index + 1]
        del sys.argv[index:index + 2]
    raise SystemExit(run_plots("Plot the drag rendezvous scenario.", build))
