#!/usr/bin/env python3
"""Figures of upstream scenarioMagneticFieldWMM.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from scenario_plotting import (
    CsvTable,
    _norm,
    _plot_vector_history,
    _time_minutes,
    _vector_rows,
    run_variant_plot,
)

STEM = "scenarioMagneticFieldWMM"
VARIANTS = ("elliptical", "circular")
REACH_KM = (10000.0, 20000.0)


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_minutes(table)

    radius = plt.figure()
    axis = radius.gca()
    radii = [_norm(row) / 1000.0 for row in _vector_rows(table, "r_BN_N_m")]
    axis.plot(time, radii, color="#aa0000")
    if variant == "elliptical":
        for reach in REACH_KM:
            axis.plot(time, [reach] * len(time), color="#007700", dashes=[5, 5, 5, 5])
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("Radius [km]")
    axis.set_ylim(min(radii) * 0.9, max(radii) * 1.1)

    field = plt.figure()
    _plot_vector_history(
        field.gca(),
        time,
        table,
        "magField_N_T",
        lambda index: rf"$B\_N_{{{index}}}$",
        "Magnetic Field [nT]",
        scale=1.0e9,
    )
    return {f"{STEM}1{variant}": radius, f"{STEM}2{variant}": field}


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, VARIANTS, build))
