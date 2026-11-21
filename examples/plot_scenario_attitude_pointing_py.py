#!/usr/bin/env python3
"""Figures of upstream scenarioAttitudePointingPy.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from plot_scenario_attitude_pointing import build_figures
from scenario_plotting import CsvTable, run_variant_plot

STEM = "scenarioAttitudePointingPy"


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    return build_figures(plt, table, variant, STEM)


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, ("",), build))
