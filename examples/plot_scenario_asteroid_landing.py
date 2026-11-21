#!/usr/bin/env python3
"""Upstream `scenarioAsteroidLanding` figure from
`scenarioAsteroidLanding.csv`: the hub's inertial velocity."""

from orbit_plotting import NANO2SEC, existing, run_plots


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, ["scenarioAsteroidLanding"]):
        time = table.time_ns * NANO2SEC
        velocity = table.vectors("hub_v_BN_N_m_per_s")
        figure, axis = plt.subplots()
        for index in range(3):
            axis.plot(time, velocity[:, index], "-", label=f"Vel [{index}]")
        axis.set_xlabel("Time [s]")
        axis.set_ylabel("Hub Velocity (Inertial) [m/s]")
        axis.legend()
        figures[stem + "_velocity"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the MuJoCo asteroid-landing scenario.", build))
