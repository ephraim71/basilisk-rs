#!/usr/bin/env python3
"""Figure of upstream scenarioAttitudePrescribed.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from scenario_plotting import CsvTable, _plot_vector_history, _time_minutes, run_variant_plot

STEM = "scenarioAttitudePrescribed"
VARIANTS = ("0", "1")


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    figure = plt.figure()
    _plot_vector_history(
        figure.gca(),
        _time_minutes(table),
        table,
        "sigma_BN",
        lambda index: rf"$\sigma_{{{index}}}$",
        r"Inertial Attitude $\sigma_{B/N}$",
    )
    return {f"{STEM}1{variant}": figure}


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, VARIANTS, build))
