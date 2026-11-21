#!/usr/bin/env python3
"""Upstream `scenarioArmWithThrusters` figure from
`scenarioArmWithThrusters.csv`: the hub attitude (and, as an extra, the
constrained joint angles)."""

import numpy as np

from orbit_plotting import NANO2SEC, existing, run_plots

JOINTS = ("arm_1_elev", "arm_2_elev", "arm_3_elev", "arm_1_roll", "arm_4_elev")


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, ["scenarioArmWithThrusters"]):
        time = table.time_ns * NANO2SEC
        sigma = table.vectors("hub_sigma_BN")
        figure, axis = plt.subplots()
        for index in range(3):
            axis.plot(time, sigma[:, index], "-", label=f"Hub sigma [{index}]")
        axis.set_xlabel("Time [s]")
        axis.set_ylabel(r"$\sigma_{BN}$ [rad]")
        axis.legend()
        figures[stem + "_sigma_BN"] = figure

        figure, axis = plt.subplots()
        for joint in JOINTS:
            axis.plot(time, np.degrees(table[f"{joint}_rad"]), label=joint)
        axis.set_xlabel("Time [s]")
        axis.set_ylabel("Joint angle [deg]")
        axis.legend()
        figures[stem + "_joints"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the MuJoCo arm-with-thrusters scenario.", build))
