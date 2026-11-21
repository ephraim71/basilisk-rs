#!/usr/bin/env python3
"""Upstream `scenarioHingedRigidBodyMuJoCo` figures from
`scenarioHingedRigidBodyMuJoCo.csv`: the inertial position, the orbit radius
and the two panels' hinge angles."""

import numpy as np

from orbit_plotting import NANO2MIN, elements, existing, line_color, run_plots

MU = 0.3986004415e15


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, ["scenarioHingedRigidBodyMuJoCo"]):
        minutes = table.time_ns * NANO2MIN
        r = table.vectors("hub_r_BN_N_m")
        v = table.vectors("hub_v_BN_N_m_per_s")
        figure, axis = plt.subplots()
        for index in range(3):
            axis.plot(minutes, r[:, index] / 1000.0, color=line_color(index, 3, plt),
                      label=f"$r_{{BN,{index}}}$")
        axis.legend(loc="lower right")
        axis.set_xlabel("Time [min]")
        axis.set_ylabel("Inertial Position [km]")
        figures[stem + "_inertPos"] = figure

        figure, axis = plt.subplots()
        radius = np.array([oe["rmag"] for oe in elements(MU, r, v)])
        axis.plot(minutes, radius / 1000.0, color="#aa0000")
        axis.set_xlabel("Time [min]")
        axis.set_ylabel("Radius [km]")
        figures[stem + "_orbit"] = figure

        figure, (axis_1, axis_2) = plt.subplots(1, 2)
        axis_1.plot(minutes, table["hinge1_rad"])
        axis_1.set_xlabel("Time [min]")
        axis_1.set_ylabel(r"$\theta$ [rad]")
        axis_1.set_title("Panel 1 Angular Displacement")
        axis_2.plot(minutes, table["hinge2_rad"])
        axis_2.set_xlabel("Time [min]")
        axis_2.set_ylabel(r"$\theta$ [rad]")
        axis_2.set_title("Panel 2 Angular Displacement")
        figures[stem + "_panelTheta"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the MuJoCo hinged-rigid-body scenario.", build))
