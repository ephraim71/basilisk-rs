#!/usr/bin/env python3
"""Figure of upstream scenarioSpacecraftLocation.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from scenario_plotting import CsvTable, _time_minutes, run_variant_plot

STEM = "scenarioSpacecraftLocation"


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    figure = plt.figure()
    axis = figure.gca()
    axis.plot(_time_minutes(table), table.column("hasAccess"))
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("Sat-Sat Access")
    return {f"{STEM}1": figure}


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, ("",), build))
