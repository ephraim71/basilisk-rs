#!/usr/bin/env python3
"""Upstream `scenarioUnbalancedThrusters` figures from
`scenarioUnbalancedThrusters.csv`: the tank masses and the hub attitude."""

from orbit_plotting import NANO2SEC, existing, run_plots


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, ["scenarioUnbalancedThrusters"]):
        time = table.time_ns * NANO2SEC
        figure, axis = plt.subplots()
        for index, style in enumerate(("-", "--", ":", "-")):
            axis.plot(time, table[f"tank_{index + 1}_mass_kg"], style, label=f"Thruster {index + 1}")
        axis.set_xlabel("Time [s]")
        axis.set_ylabel("Mass [kg]")
        axis.legend()
        figures[stem + "_mass"] = figure

        sigma = table.vectors("hub_sigma_BN")
        figure, axis = plt.subplots()
        for index in range(3):
            axis.plot(time, sigma[:, index], label=f"Hub sigma [{index}]")
        axis.set_xlabel("Time [s]")
        axis.set_ylabel(r"$\sigma_{BN}$ [rad]")
        axis.legend()
        figures[stem + "_sigma_BN"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the MuJoCo unbalanced-thrusters scenario.", build))
