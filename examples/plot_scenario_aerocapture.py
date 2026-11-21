#!/usr/bin/env python3
"""Upstream `scenarioAerocapture` figures from `scenarioAerocapture<planet>.csv`:
the position, the orbital energy, the density, the altitude over velocity
and over time, named `scenarioAerocapture<n><planet>`."""

import numpy as np

from orbit_plotting import MU_EARTH, MU_MARS, NANO2MIN, existing, line_color, run_plots

PLANETS = {"Earth": (MU_EARTH, 6378136.6), "Mars": (MU_MARS, 3397.2 * 1000)}


def build(generated_dir, plt):
    figures = {}
    for planet, (mu, r_eq) in PLANETS.items():
        for _, table in existing(generated_dir, [f"scenarioAerocapture{planet}"]):
            minutes = table.time_ns * NANO2MIN
            r, v = table.vectors("r_BN_N_m"), table.vectors("v_BN_N_m_per_s")
            figure = plt.figure()
            axis = figure.gca()
            axis.ticklabel_format(useOffset=False, style="plain")
            for index in range(3):
                axis.plot(minutes, r[:, index] / 1000.0, color=line_color(index, 3, plt),
                          label=f"$r_{{BN,{index}}}$")
            axis.legend(loc="lower right")
            axis.set_xlabel("Time [min]")
            axis.set_ylabel("Inertial Position [km]")
            figures[f"scenarioAerocapture1{planet}"] = figure

            radius = np.linalg.norm(r, axis=1)
            speed = np.linalg.norm(v, axis=1)
            energy = 0.5 * speed**2 - mu / radius
            for number, x, y, xlabel, ylabel, color in (
                (2, minutes, energy / 1e6, "Time [min]", "Energy [km^2/s^2]", "#aa0000"),
                (3, minutes, table["neutralDensity_kg_per_m3"], "Time [min]", "Density in kg/m^3", None),
                (4, speed / 1e3, (radius - r_eq) / 1e3, "velocity [km/s]", "altitude [km]", None),
                (5, minutes, (radius - r_eq) / 1e3, "time [min]", "altitude [km]", None),
            ):
                figure = plt.figure()
                axis = figure.gca()
                axis.plot(x, y, color=color)
                axis.set_xlabel(xlabel)
                axis.set_ylabel(ylabel)
                if number != 3:
                    axis.grid()
                figures[f"scenarioAerocapture{number}{planet}"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the aerocapture scenario.", build))
