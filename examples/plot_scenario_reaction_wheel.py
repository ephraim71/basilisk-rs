#!/usr/bin/env python3
"""Upstream `scenarioReactionWheel` figure from `scenarioReactionWheel.csv`:
the hub attitude while the wheel torque reverses at 60 s."""

from orbit_plotting import NANO2SEC, existing, run_plots


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, ["scenarioReactionWheel"]):
        time = table.time_ns * NANO2SEC
        sigma = table.vectors("hub_sigma_BN")
        figure, axis = plt.subplots()
        for index in range(3):
            axis.plot(time, sigma[:, index], "-", label=f"Hub sigma [{index}]")
        axis.set_xlabel("Time [s]")
        axis.set_ylabel(r"$\sigma_{BN}$ [rad]")
        axis.legend()
        figures[stem + "_sigma_BN"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the MuJoCo reaction-wheel scenario.", build))
