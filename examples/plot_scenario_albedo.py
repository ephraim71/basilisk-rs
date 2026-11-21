#!/usr/bin/env python3
"""Figures of upstream scenarioAlbedo.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from scenario_plotting import (
    CsvTable,
    _line_color,
    _norm,
    _time_seconds,
    _vector_rows,
    run_variant_plot,
)

STEM = "scenarioAlbedo"
VARIANTS = ("0101", "1000", "0100", "0110")


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_seconds(table)
    multiple_planet = variant[2] == "1"
    count = sum(1 for header in table.headers if header.startswith("albedo"))

    figure = plt.figure()
    axis = figure.gca()
    for index in range(count):
        axis.plot(
            time,
            table.column(f"albedo{index}"),
            linewidth=2,
            alpha=0.7,
            color=_line_color(index),
            label=rf"Albedo$_{{{index}}}$",
        )
        if not multiple_planet:
            axis.plot(
                time,
                table.column(f"css{index + 1}Signal"),
                "--",
                linewidth=1.5,
                color=_line_color(index),
                label=rf"CSS$_{{{index}}}$",
            )
    axis.legend(loc="upper right")
    axis.set_xlabel("Time [s]")
    axis.set_ylabel("Instrument's signal [-]" if not multiple_planet else "Albedo [-]")
    figures = {f"{STEM}1{variant}": figure}

    if multiple_planet:
        radius = plt.figure()
        axis = radius.gca()
        axis.plot(
            time,
            [_norm(row) / 1000.0 for row in _vector_rows(table, "r_BN_N_m")],
            color="#aa0000",
        )
        axis.set_xlabel("Time [s]")
        axis.set_ylabel("Radius [km]")
        figures[f"{STEM}2{variant}"] = radius
    return figures


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, VARIANTS, build))
