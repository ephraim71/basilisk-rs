#!/usr/bin/env python3
"""Figures of upstream scenario_OpNavAttOD.py from the Rust CSV(s), as its
``pull_outputs`` draws them: the orbit and the measurements, the
measurement errors in the camera frame, the filter's percent errors and
state errors with their covariance, the post-fit residuals, and the
circles found against the truth."""
from __future__ import annotations

from pathlib import Path
from typing import Any

import opnav_plotting as opnav
from orbit_plotting import Table, run_plots

STEM = "scenario_OpNavAttOD"


def build(generated_dir: Path, plt: Any) -> dict[str, Any]:
    table = Table(generated_dir / f"{STEM}.csv")
    figures = opnav.od_figures(plt, table, "Circ", circles=True, valid_column="circles_valid")
    return {f"{STEM}_{name}": figure for name, figure in figures.items()}


if __name__ == "__main__":
    raise SystemExit(run_plots(f"Plot {STEM} results.", build))
