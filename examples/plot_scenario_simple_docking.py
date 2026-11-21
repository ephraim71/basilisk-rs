#!/usr/bin/env python3
"""Upstream `scenarioSimpleDocking` figure from `scenarioSimpleDocking.csv`:
the distance between the two contact points."""

import numpy as np

from orbit_plotting import NANO2SEC, existing, run_plots


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, ["scenarioSimpleDocking"]):
        one = table.vectors("hub_1_contact_r_BN_N_m")
        two = table.vectors("hub_2_contact_r_BN_N_m")
        figure, axis = plt.subplots()
        axis.plot(table.time_ns * NANO2SEC, np.linalg.norm(one - two, axis=1))
        axis.set_xlabel("Time [s]")
        axis.set_ylabel("Distance Between Hubs [m]")
        figures[stem + "_distance"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the MuJoCo docking scenario.", build))
