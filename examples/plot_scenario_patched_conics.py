#!/usr/bin/env python3
"""Upstream `scenarioPatchedConics` figures from `scenarioPatchedConics.csv`:
the Earth departure, the Earth-relative radius, the heliocentric transfer,
the Jupiter arrival and the heliocentric radius of every phase."""

import math

import numpy as np

from orbit_plotting import (MU_EARTH, MU_SUN, NANO2HOUR, NANO2MIN, REQ_EARTH, REQ_JUPITER, REQ_SUN,
                            elements, existing, run_plots)

R_EARTH_ORBIT = 149598023.0 * 1000
R_JUPITER_ORBIT = 778298361.0 * 1000
POS_EARTH = np.array([0.0, -R_EARTH_ORBIT, 0.0])
POS_JUPITER = np.array([0.0, R_JUPITER_ORBIT, 0.0])
WEEK_NS = 7 * 24 * 3600 * 10**9


def phase_ends_ns():
    """Upstream's phase boundaries (ns): end of the Earth departure, of the
    weekly heliocentric cruise and of the 500 s approach."""
    r_leo = 7000.0 * 1000
    at = (R_EARTH_ORBIT + R_JUPITER_ORBIT) * 0.5
    v_pt = math.sqrt(2 * MU_SUN / R_EARTH_ORBIT - MU_SUN / at)
    t2 = int(math.pi / math.sqrt(MU_SUN / at**3) * 1e9 + 0.5)
    v_rel = v_pt - 29.7859 * 1000
    e_hyp = r_leo * v_rel**2 / MU_EARTH + 1
    n = math.sqrt(MU_EARTH / r_leo**3)
    simulation_time = int((math.acos(1 / e_hyp) + math.pi) / n * 1e9 + 0.5)
    climb = simulation_time + 110000 * 10**9
    return climb, climb + t2 - WEEK_NS, climb + t2 - WEEK_NS // 2


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, ["scenarioPatchedConics"]):
        time = table.time_ns
        r = table.vectors("r_BN_N_m")
        v = table.vectors("v_BN_N_m_per_s")
        earth_end, sun_end, switch_end = phase_ends_ns()
        earth = time <= earth_end
        sun = (time > earth_end) & (time <= sun_end)
        switch = (time > sun_end) & (time <= switch_end)
        jupiter = time > switch_end

        r_e, v_e = r[earth] - POS_EARTH, v[earth]
        oes = elements(MU_EARTH, r_e, v_e)
        figure = plt.figure(1)
        axis = figure.gca()
        axis.axis("equal")
        axis.axis([-20000, 50000, -10000, 10000])
        axis.add_artist(plt.Circle((0, 0), REQ_EARTH / 1000, color="#008800"))
        radius = np.array([oe["rmag"] for oe in oes])
        angle = np.array([oe["f"] + oe["omega"] for oe in oes])
        axis.plot(radius * np.cos(angle) / 1000, radius * np.sin(angle) / 1000, color="#aa0000",
                  linewidth=3.0, label="Simulated Flight")
        f = np.linspace(0, 2 * np.pi, 100)
        axis.plot(7000 * np.cos(f), 7000 * np.sin(f), "--", color="#555555", label="Orbit Track")
        axis.set_xlabel("Earth velocity direction [km]")
        axis.set_ylabel("Sunward direction [km]")
        axis.legend(loc="upper right")
        axis.grid()
        figures["scenarioPatchedConics1"] = figure

        figure = plt.figure(2)
        axis = figure.gca()
        count = min(300, len(r_e))
        axis.plot(time[:count] * NANO2MIN, np.linalg.norm(r_e[:count], axis=1) / 1000, color="#aa0000")
        axis.set_xlabel("Time [min]")
        axis.set_ylabel("Earth Relative Radius [km]")
        figures["scenarioPatchedConics2"] = figure

        figure = plt.figure(3)
        axis = figure.gca()
        axis.add_artist(plt.Circle((0, 0), REQ_SUN / 1000 / 149598000, color="#008800"))
        helio = elements(MU_SUN, r[sun], v[sun])
        radius = np.array([oe["rmag"] for oe in helio]) / 1000 / 149598000
        angle = np.array([oe["f"] for oe in helio])
        axis.plot(radius * np.cos(angle), radius * np.sin(angle), color="#008800", linewidth=3.0,
                  label="Simulated Flight")
        axis.legend(loc="lower left")
        axis.grid()
        axis.set_xlabel("Y axis - Sunward Direction [AU]")
        axis.set_ylabel("X axis - Velocity Direction [AU]")
        figures["scenarioPatchedConics3"] = figure

        figure = plt.figure(4, figsize=(5, 5))
        axis = figure.gca()
        axis.axis([-20000000, 20000000, -20000000, 20000000])
        axis.set_aspect("equal")
        axis.add_artist(plt.Circle((0, 0), REQ_JUPITER / 1000, color="#008800"))
        r_j = r[jupiter] - POS_JUPITER
        axis.plot(r_j[:, 0] / 1000, r_j[:, 1] / 1000, color="orangered", label="Simulated Flight")
        axis.set_xlabel("Jupiter velocity direction [km]")
        axis.set_ylabel("Anti-Sunward direction [km]")
        figures["scenarioPatchedConics4"] = figure

        figure = plt.figure(5)
        axis = figure.gca()
        for mask, color, label in ((earth, "#aa0000", "Earth Centered"), (sun, "#008800", "Heliocentric transfer"),
                                   (switch, "#555555", "Time Switch"), (jupiter, "orangered", "Jupiter Centered")):
            if mask.any():
                axis.plot(time[mask] * NANO2HOUR, np.linalg.norm(r[mask], axis=1) / 1000, color=color, label=label)
        axis.set_yscale("log")
        axis.legend(loc="lower right")
        axis.set_xlabel("Time [h]")
        axis.set_ylabel("Heliocentric Radius [km]")
        figures["scenarioPatchedConics5"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the patched conics scenario.", build))
