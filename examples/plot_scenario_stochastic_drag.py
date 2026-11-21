#!/usr/bin/env python3
"""Upstream `scenarioStochasticDrag` figures from `scenarioStochasticDrag.csv`
and `scenarioStochasticDrag_igbm.csv`: the orbit, the altitude, the
stochastic and exponential densities, their ratio and the drag force."""

import numpy as np

from orbit_plotting import MU_EARTH, NANO2HOUR, REQ_EARTH, elements, existing, rv2elem, run_plots


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, ["scenarioStochasticDrag",
                                                "scenarioStochasticDrag_igbm"]):
        hours = table.time_ns * NANO2HOUR
        r, v = table.vectors("r_BN_N_m"), table.vectors("v_BN_N_m_per_s")
        drag = table.vectors("force_S_N")
        oe0 = rv2elem(MU_EARTH, r[0], v[0])
        radius, angle = [], []
        for oe in elements(MU_EARTH, r, v):
            radius.append(oe["rmag"])
            angle.append(oe["f"] + oe["omega"] - oe0["omega"])
        radius, angle = np.array(radius), np.array(angle)

        figure, axis = plt.subplots()
        axis.axis("equal")
        axis.add_artist(plt.Circle((0, 0), REQ_EARTH / 1000.0, color="#008800"))
        axis.plot(radius * np.cos(angle) / 1000.0, radius * np.sin(angle) / 1000.0, color="#aa0000",
                  linewidth=1.0)
        axis.set_xlabel("$i_e$ Cord. [km]")
        axis.set_ylabel("$i_p$ Cord. [km]")
        figures[stem + "_orbit"] = figure

        altitude = (radius - REQ_EARTH) / 1000.0
        figure, axis = plt.subplots()
        axis.ticklabel_format(useOffset=False, style="plain")
        axis.plot(hours, altitude)
        axis.set_xlabel("$t$ [h]")
        axis.set_ylabel("Altitude [km]")
        figures[stem + "_altitude"] = figure

        density = table["neutralDensity_kg_per_m3"]
        exponential = table["neutralDensityExponential_kg_per_m3"]
        figure, axis = plt.subplots()
        axis.semilogy(altitude[1:], density[1:], label="Stochastic")
        axis.semilogy(altitude[1:], exponential[1:], label="Exponential")
        axis.legend(loc="upper right")
        axis.set_xlabel("Altitude [km]")
        axis.set_ylabel("$\\rho$ [kg/m$^3$]")
        figures[stem + "_density"] = figure

        figure, axis = plt.subplots()
        axis.plot(hours[1:], (density / exponential)[1:] - 1.0)
        axis.set_xlabel("Time [hr]")
        axis.set_ylabel(r"$(\rho_{stoch} / \rho_{exp}) - 1$ [-]")
        figures[stem + "_densityDiff"] = figure

        figure, axis = plt.subplots()
        axis.semilogy(hours[1:], np.linalg.norm(drag, axis=1)[1:])
        axis.set_xlabel("$t$ [hr]")
        axis.set_ylabel("$|F_{drag}|$ [N]")
        figures[stem + "_drag"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the MuJoCo stochastic drag scenario.", build))
