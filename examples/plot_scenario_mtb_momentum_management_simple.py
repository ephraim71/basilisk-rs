#!/usr/bin/env python3
"""Figures of upstream scenarioMtbMomentumManagementSimple.py from the Rust CSV.

The CSV has the columns of scenarioMtbMomentumManagement, so the figures are
those of plot_scenario_mtb_momentum_management.py.
"""
from __future__ import annotations

from typing import Any

from scenario_plotting import CsvTable, _build_mtb_momentum_management, run_variant_plot

STEM = "scenarioMtbMomentumManagementSimple"


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    figures = _build_mtb_momentum_management(plt, table)
    return {
        name.replace("scenarioMtbMomentumManagement", STEM): figure
        for name, figure in figures.items()
    }


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, ("",), build))
