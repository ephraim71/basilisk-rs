#!/usr/bin/env python3
"""Figure of upstream scenario_constellationFromTle.py from the Rust CSVs:
every spacecraft's inertial orbit (``spacecraft_orbits``), for each TLE file
the example ran (``scenario_constellationFromTle_<file stem>_sat<i>.csv``)."""
from __future__ import annotations

import re
from pathlib import Path
from typing import Any

from multi_sat_plotting import orbit_figure, spacecraft_tables
from orbit_plotting import run_plots

STEM = "scenario_constellationFromTle"


def build(generated_dir: Path, plt: Any) -> dict[str, Any]:
    pattern = re.compile(rf"^{STEM}_(.+)_sat\d+\.csv$")
    catalogs = sorted({
        match.group(1)
        for path in generated_dir.glob(f"{STEM}_*_sat*.csv")
        if (match := pattern.match(path.name))
    })
    figures: dict[str, Any] = {}
    for catalog in catalogs:
        tables = spacecraft_tables(generated_dir, f"{STEM}_{catalog}")
        figures[f"{STEM}_{catalog}_spacecraft_orbits"] = orbit_figure(plt, tables)
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(f"Plot {STEM} results.", build))
