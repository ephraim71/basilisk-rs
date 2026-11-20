#!/usr/bin/env python3
"""Figure of upstream scenario_BasicOrbitMultiSat.py from the Rust CSVs: the
three spacecraft's inertial orbits (``spacecraft_orbits``), for each
environment the example ran (``scenario_BasicOrbitMultiSat<Environment>_sat<i>.csv``)."""
from __future__ import annotations

from pathlib import Path
from typing import Any

from multi_sat_plotting import orbit_figure, spacecraft_tables
from orbit_plotting import run_plots

STEM = "scenario_BasicOrbitMultiSat"
ENVIRONMENTS = ("Earth", "Mercury")


def build(generated_dir: Path, plt: Any) -> dict[str, Any]:
    figures: dict[str, Any] = {}
    for environment in ENVIRONMENTS:
        tables = spacecraft_tables(generated_dir, STEM + environment)
        if tables:
            figures[f"{STEM}{environment}_spacecraft_orbits"] = orbit_figure(plt, tables)
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(f"Plot {STEM} results.", build))
