#!/usr/bin/env python3
"""Figures of upstream scenarioAttitudeConstrainedManeuver.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from constrained_attitude_plotting import build as build_figures
from scenario_plotting import CsvTable, run_variant_plot

STEM = "scenarioAttitudeConstrainedManeuver"
VARIANTS = ("130703", "020700", "120701", "120702")


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    return build_figures(plt, table, STEM, variant)


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, VARIANTS, build))
