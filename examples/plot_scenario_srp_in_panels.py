#!/usr/bin/env python3
"""Upstream `scenarioSRPInPanels` figures from `scenarioSRPInPanels.csv`
(and `scenarioSRPInPanelsInitialSpin.csv`): each panel's SRP force and the
hub's z velocity."""

from orbit_plotting import NANO2SEC, existing, run_plots

PANELS = ("p1", "p2", "p3", "n1", "n2", "n3")


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, ["scenarioSRPInPanels",
                                                "scenarioSRPInPanelsInitialSpin"]):
        time = table.time_ns * NANO2SEC
        figure, axes = plt.subplots(2, 3)
        for axis, panel in zip(axes.flat, PANELS):
            axis.plot(time, table[f"panel_{panel}_srp_N"])
            axis.set_title(f"Panel {panel}")
            axis.set_xticks([])
            axis.set_xlabel("Time [sec]")
            axis.set_ylabel("Force [N]")
        figure.suptitle("Solar Radiation Pressure Force")
        figures[stem + "_srpForce"] = figure

        figure, axis = plt.subplots()
        axis.plot(time, table["hub_v_BN_N_m_per_s_2"], marker=".")
        axis.set_xlabel("Time [sec]")
        axis.set_ylabel("Z-axis Velocity [m/s]")
        figures[stem + "_velocityZ"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the MuJoCo SRP-in-panels scenario.", build))
