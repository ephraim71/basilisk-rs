#!/usr/bin/env python3
"""Upstream `scenarioAsteroidArrival` figure from `scenarioAsteroidArrival.csv`:
the Bennu-relative trajectory in the inertial x-z plane, with Bennu and the
desired circular capture orbit."""

import numpy as np
from orbit_plotting import existing, run_plots

DIAMETER = 2 * 245.03  # [m]
R_P = DIAMETER / 2.0 + 800  # [m] the capture orbit


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, ["scenarioAsteroidArrival"]):
        relative = table.vectors("r_BN_N_m") - table.vectors("bennu_r_N_m")
        figure = plt.figure(figsize=(5, 5))
        axis = figure.gca()
        axis.set_aspect("equal")
        axis.ticklabel_format(useOffset=False, style="sci")
        formatter = plt.FuncFormatter(lambda x, loc: "{:,}".format(int(x)))
        axis.get_yaxis().set_major_formatter(formatter)
        axis.get_xaxis().set_major_formatter(formatter)
        axis.add_artist(plt.Circle((0, 0), 0.5 * DIAMETER, color="#008800"))
        axis.plot(relative[:, 0], relative[:, 2], color="orangered", label="Simulated Flight")
        f = np.linspace(0, 2 * np.pi, 100)
        axis.plot(R_P * np.cos(f), R_P * np.sin(f), "--", color="#555555",
                  label="Desired Circ.Capture Orbit")
        axis.set_xlabel("X Distance, Inertial [m]")
        axis.set_ylabel("Z Distance, Inertial [m]")
        axis.legend(loc="upper right")
        axis.grid()
        figures[stem + "1"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the asteroid arrival scenario.", build))
