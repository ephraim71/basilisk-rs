#!/usr/bin/env python3
"""Figures of upstream scenarioFormationReconfig.py from the Rust CSV: the
deputy-minus-chief osculating element differences (figure 1) and the
deputy's attitude error (figure 2), against time in orbits."""
from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from formation_plotting import classic_difference
from orbit_plotting import MU_EARTH, NANO2SEC, elements, existing, run_plots

STEM = "scenarioFormationReconfig"
VARIANTS = ("0", "1")  # int(useRefAttitude)
# Upstream's orbit period uses the deputy's semi-major axis (`oe2 = oe`).
DEPUTY_A = 11000.0e3 * (1.0 + 0.0003)


def build(generated_dir: Path, plt: Any) -> dict[str, Any]:
    figures: dict[str, Any] = {}
    orbit_period = 2.0 * math.pi / math.sqrt(MU_EARTH / DEPUTY_A**3)
    for stem, table in existing(generated_dir, [STEM + variant for variant in VARIANTS]):
        variant = stem[len(STEM):]
        time = table.time_ns * NANO2SEC / orbit_period
        chief = elements(MU_EARTH, table.vectors("chief_r_BN_N_m"),
                         table.vectors("chief_v_BN_N_m_per_s"))
        deputy = elements(MU_EARTH, table.vectors("deputy_r_BN_N_m"),
                          table.vectors("deputy_v_BN_N_m_per_s"))
        rows = [classic_difference(c, d) for c, d in zip(chief, deputy, strict=True)]

        figure = plt.figure()
        axis = figure.gca()
        for index, label in enumerate(("da", "de", "di", "dOmega", "domega", "dM")):
            axis.plot(time, [row[index] for row in rows], label=label)
        axis.legend()
        axis.set_xlabel("time [orbit]")
        axis.set_ylabel("orbital element difference")
        figures[f"{STEM}1{variant}"] = figure

        figure = plt.figure()
        axis = figure.gca()
        sigma = table.vectors("sigma_BR")
        for index in range(3):
            axis.plot(time, sigma[:, index])
        axis.set_xlabel("time [orbit]")
        axis.set_ylabel("MRP Error")
        figures[f"{STEM}2{variant}"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(f"Plot {STEM} results.", build))
