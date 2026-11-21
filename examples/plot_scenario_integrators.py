#!/usr/bin/env python3
"""Upstream `scenarioIntegrators` figure from `scenarioIntegrators<case>.csv`:
each integrator's orbit in the initial perifocal frame over the true orbit."""

import math

import numpy as np

from orbit_plotting import MU_EARTH, REQ_EARTH, elements, existing, line_color, rv2elem, run_plots

CASES = ("rk4", "rkf45", "rkf78", "euler", "rk2", "rk3", "bogackiShampine")


def build(generated_dir, plt):
    tables = existing(generated_dir, [f"scenarioIntegrators{case}" for case in CASES])
    if not tables:
        return {}
    first = tables[0][1]
    oe0 = rv2elem(MU_EARTH, first.vectors("r_BN_N_m")[0], first.vectors("v_BN_N_m_per_s")[0])
    b = oe0["a"] * math.sqrt(1 - oe0["e"] ** 2)
    p = oe0["a"] * (1 - oe0["e"] ** 2)
    figure = plt.figure(figsize=tuple(np.array((1.0, b / oe0["a"])) * 4.75), dpi=100)
    axis = figure.gca()
    axis.axis(np.array([-oe0["rApoap"], oe0["rPeriap"], -b, b]) / 1000 * 1.25)
    axis.add_artist(plt.Circle((0, 0), REQ_EARTH / 1000, color="#008800"))
    for index, (stem, table) in enumerate(tables):
        radius, angle = [], []
        for oe in elements(MU_EARTH, table.vectors("r_BN_N_m"), table.vectors("v_BN_N_m_per_s")):
            radius.append(oe["rmag"])
            angle.append(oe["f"] + oe["omega"] - oe0["omega"])
        radius, angle = np.array(radius), np.array(angle)
        axis.plot(radius * np.cos(angle) / 1000, radius * np.sin(angle) / 1000,
                  label=stem.removeprefix("scenarioIntegrators"), linewidth=3.0,
                  color=line_color(index, len(tables), plt))
    f = np.linspace(0, 2 * np.pi, 100)
    r = p / (1 + oe0["e"] * np.cos(f))
    axis.plot(r * np.cos(f) / 1000, r * np.sin(f) / 1000, "--", color="#555555")
    axis.set_xlabel("$i_e$ Cord. [km]")
    axis.set_ylabel("$i_p$ Cord. [km]")
    axis.legend(loc="lower right")
    axis.grid()
    axis.set_aspect("equal", adjustable="box")
    return {"scenarioIntegrators": figure}


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the integrator comparison orbits.", build))
