#!/usr/bin/env python3
"""Figures of upstream scenarioMagneticFieldCenteredDipole.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from scenario_plotting import CsvTable, _plot_vector_history, _time_minutes, run_variant_plot

STEM = "scenarioMagneticFieldCenteredDipole"
VARIANTS = ("circularEarth", "ellipticalEarth", "ellipticalJupiter")


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_minutes(table)

    position = plt.figure()
    _plot_vector_history(
        position.gca(),
        time,
        table,
        "r_BN_N_m",
        lambda index: rf"$r_{{BN,{index}}}$",
        "Inertial Position [km]",
        scale=1.0e-3,
    )

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
    if "magField2_N_T_0" in table.headers:
        _plot_vector_history(
            field.gca(),
            time,
            table,
            "magField2_N_T",
            lambda index: rf"$B\_N_{{{index}}}$ (inner dipole)",
            "Magnetic Field [nT]",
            scale=1.0e9,
        )
    return {f"{STEM}1{variant}": position, f"{STEM}2{variant}": field}


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, VARIANTS, build))
