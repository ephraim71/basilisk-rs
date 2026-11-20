#!/usr/bin/env python3
"""Figures of upstream scenarioFormationMeanOEFeedback.py from the Rust CSV:
the deputy-minus-chief J2 mean element differences in classical (figure 1)
and equinoctial (figure 2) elements, against time in orbits."""
from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from formation_plotting import classic_difference, equinoctial_difference, mean_osc_map
from orbit_plotting import MU_EARTH, NANO2SEC, PlotError, elements, existing, run_plots

STEM = "scenarioFormationMeanOEFeedback"
VARIANTS = ("1", "0")  # int(useClassicElem)
CHIEF_A = 11000.0e3  # upstream oe.a [m]


def build(generated_dir: Path, plt: Any) -> dict[str, Any]:
    figures: dict[str, Any] = {}
    orbit_period = 2.0 * math.pi / math.sqrt(MU_EARTH / CHIEF_A**3)
    for stem, table in existing(generated_dir, [STEM + variant for variant in VARIANTS]):
        variant = stem[len(STEM):]
        time = table.time_ns * NANO2SEC / orbit_period
        chief = elements(MU_EARTH, table.vectors("chief_r_BN_N_m"),
                         table.vectors("chief_v_BN_N_m_per_s"))
        deputy = elements(MU_EARTH, table.vectors("deputy_r_BN_N_m"),
                          table.vectors("deputy_v_BN_N_m_per_s"))
        if len(chief) < 2:
            raise PlotError(f"{stem}.csv needs at least two samples")
        chief_mean = [mean_osc_map(oe, -1.0) for oe in chief]
        deputy_mean = [mean_osc_map(oe, -1.0) for oe in deputy]

        for number, difference, labels in (
            ("1", classic_difference, ("da", "de", "di", "dOmega", "domega", "dM")),
            ("2", equinoctial_difference, ("da", "dP1", "dP2", "dQ1", "dQ2", "dl")),
        ):
            rows = [difference(c, d) for c, d in zip(chief_mean, deputy_mean, strict=True)]
            figure = plt.figure()
            axis = figure.gca()
            for index, label in enumerate(labels):
                axis.plot(time, [row[index] for row in rows], label=label)
            axis.legend()
            axis.set_xlabel("time [orbit]")
            axis.set_ylabel("mean orbital element difference")
            figures[f"{STEM}{number}{variant}"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(f"Plot {STEM} results.", build))
