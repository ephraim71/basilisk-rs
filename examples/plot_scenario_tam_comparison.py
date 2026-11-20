#!/usr/bin/env python3
"""Figures of upstream scenarioTAMcomparison.py from the Rust CSV."""
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

STEM = "scenarioTAMcomparison"
VARIANTS = (
    "circularFalseTrueFalseFalse",
    "circularFalseTrueFalseTrue",
    "ellipticalTrueFalseTrueFalse",
    "ellipticalFalseFalseFalseFalse",
)


def _tam_figure(plt: Any, time: list[float], table: CsvTable, prefix: str) -> Any:
    figure = plt.figure()
    _plot_vector_history(
        figure.gca(),
        time,
        table,
        prefix,
        lambda index: rf"$TAM_{{{index}}}$",
        "Magnetic Field [nT] ",
        scale=1.0e9,
    )
    return figure


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_minutes(table)

    radius = plt.figure()
    axis = radius.gca()
    radii = [_norm(row) / 1000.0 for row in _vector_rows(table, "r_BN_N_m")]
    axis.plot(time, radii, color="#aa0000")
    if variant.startswith("elliptical"):
        for reach in (10000.0, 20000.0):
            axis.plot(time, [reach] * len(time), color="#007700", dashes=[5, 5, 5, 5])
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("Radius [km]")

    dipole = _tam_figure(plt, time, table, "tam1_S_T")
    if "tam3_S_T_0" in table.headers:
        _plot_vector_history(
            dipole.gca(),
            time,
            table,
            "tam3_S_T",
            lambda index: rf"$TAM_{{{index}}}$ (inner dipole)",
            "Magnetic Field [nT] ",
            scale=1.0e9,
        )
    wmm = _tam_figure(plt, time, table, "tam2_S_T")
    return {
        f"{STEM}1{variant}": radius,
        f"{STEM}2{variant}": dipole,
        f"{STEM}3{variant}": wmm,
    }


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, VARIANTS, build))
