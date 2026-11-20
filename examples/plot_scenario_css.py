#!/usr/bin/env python3
"""Figure of upstream scenarioCSS.py from the Rust CSV, for every variant."""
from __future__ import annotations

from typing import Any

from scenario_plotting import CsvTable, _line_color, _time_seconds, run_variant_plot

STEM = "scenarioCSS"
VARIANTS = ("0010", "0000", "0100", "0001", "1000")


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    figure = plt.figure(figsize=(6.4, 4.8))
    axis = figure.gca()
    time = _time_seconds(table)
    constellation = variant.startswith("1")
    for index in range(3):
        label = rf"CSS$_{{{index}}}$" if constellation else rf"CSS$_{{{index + 1}}}$"
        axis.plot(
            time,
            table.column(f"css{index + 1}Signal"),
            color=_line_color(index),
            label=label,
        )
    axis.legend(loc="lower right")
    axis.set_xlabel("Time [sec]")
    axis.set_ylabel("CSS Signals ")
    return {f"{STEM}{variant}": figure}


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, VARIANTS, build))
