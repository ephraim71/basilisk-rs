#!/usr/bin/env python3
"""Figures of upstream scenarioSpiceSpacecraft.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from scenario_plotting import CsvTable, _plot_vector_history, _time_minutes, run_variant_plot

STEM = "scenarioSpiceSpacecraft"


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_minutes(table)
    attitude = plt.figure()
    _plot_vector_history(
        attitude.gca(), time, table, "sigma_BR",
        lambda index: rf"$\sigma_{index}$", r"Attitude Error $\sigma_{B/R}$",
    )
    # Upstream saves its figure 4 (the Hubble position) as figure 2.
    position = plt.figure()
    _plot_vector_history(
        position.gca(), time, table, "r_BN_N_m",
        lambda index: rf"$r_{{BN,{index}}}$", "Inertial Position [km]", scale=1.0e-3,
    )
    return {f"{STEM}1": attitude, f"{STEM}2": position}


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, ("",), build))
