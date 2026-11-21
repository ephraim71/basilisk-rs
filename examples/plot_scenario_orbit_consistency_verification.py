#!/usr/bin/env python3
"""Upstream `scenarioOrbitConsistencyVerification` figures: the eccentricity
and inclination histories of the rotating and non-rotating Earth runs."""

import math

from orbit_plotting import MU_EARTH, NANO2SEC, PlotError, Table, elements, run_plots


def build(generated_dir, plt):
    runs = {}
    for case in ("Rotating", "NonRotating"):
        path = generated_dir / f"scenarioOrbitConsistencyVerification{case}.csv"
        if not path.is_file():
            raise PlotError(f"{path} is missing; run scenario_orbit_consistency_verification")
        table = Table(path)
        oes = elements(MU_EARTH, table.vectors("r_BN_N_m"), table.vectors("v_BN_N_m_per_s"))
        runs[case] = (table.time_ns * NANO2SEC / 86400.0, oes)
    figures = {}
    for key, label, value in (
        ("ecc", "eccentricity [-]", lambda oe: oe["e"]),
        ("inc", "inclination [deg]", lambda oe: math.degrees(oe["i"])),
    ):
        figure = plt.figure()
        axis = figure.gca()
        days, oes = runs["Rotating"]
        axis.plot(days, [value(oe) for oe in oes], color="tab:blue", label="Earth rotating (consistent)")
        days, oes = runs["NonRotating"]
        axis.plot(days, [value(oe) for oe in oes], color="tab:red", linestyle="--", label="Earth non-rotating")
        axis.set_xlabel("time [days]")
        axis.set_ylabel(label)
        axis.legend(loc="upper left")
        axis.grid(True)
        figures[f"scenarioOrbitConsistencyVerification_{key}"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the orbit consistency verification scenario.", build))
