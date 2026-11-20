#!/usr/bin/env python3
"""Figures of upstream BskSim scenario_RelativePointingFormation.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

import bsk_sim_plotting as bsk
from scenario_plotting import CsvTable

STEM = "scenario_RelativePointingFormation"


def build(plt: Any, table: CsvTable) -> dict[str, Any]:
    time = bsk.minutes(table)
    vec = lambda prefix: bsk.vectors(table, prefix)  # noqa: E731
    return {
        "attitude_error": bsk.plot_attitude_error(plt, time, vec("deputy_sigma_BR")),
        "relative_orbit": bsk.plot_rel_orbit(plt, time, vec("chief_r_BN_N_m"), vec("deputy_r_BN_N_m")),
        "sigma_RN": bsk.plot_sigma(plt, time, vec("deputy_sigma_RN")),
        "sigma_BN_deputy": bsk.plot_sigma(plt, time, vec("deputy_sigma_BN")),
        "sigma_BR_deputy": bsk.plot_sigma(plt, time, vec("deputy_sigma_BR")),
    }


if __name__ == "__main__":
    raise SystemExit(bsk.run(STEM, build))
