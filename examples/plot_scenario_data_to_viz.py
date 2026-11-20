#!/usr/bin/env python3
"""Figures of upstream scenarioDataToViz.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from scenario_plotting import CsvTable, _line_color, _norm, _time_hours, _vector_rows, run_variant_plot

STEM = "scenarioDataToViz"
VARIANTS = ("0", "1")


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_hours(table)
    norms = plt.figure()
    axis = norms.gca()
    for spacecraft in (1, 2):
        axis.plot(
            time,
            [_norm(sigma) for sigma in _vector_rows(table, f"sigma_B{spacecraft}N")],
            color=_line_color(spacecraft),
            label=rf"$|\sigma_{{B{spacecraft}/N}}|$",
        )
    axis.set_xlabel("Time [h]")
    axis.set_ylabel("MRP Norm")
    axis.legend(loc="lower right")

    relative = plt.figure()
    axis = relative.gca()
    rho = [
        [target[index] - servicer[index] for index in range(3)]
        for servicer, target in zip(
            _vector_rows(table, "r_B1N_N_m"), _vector_rows(table, "r_B2N_N_m"), strict=True
        )
    ]
    for index in range(3):
        axis.plot(time, [row[index] for row in rho], color=_line_color(index),
                  label=rf"$\rho_{{{index + 1}}}$")
    axis.legend(loc="lower right")
    axis.set_xlabel("Time [h]")
    axis.set_ylabel(r"$\rho_{S/T}$ (Inertial) [m]")
    return {f"{STEM}1{variant}": norms, f"{STEM}2{variant}": relative}


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, VARIANTS, build))
