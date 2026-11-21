#!/usr/bin/env python3
"""Figures of upstream scenarioTempMeasurementAttitude.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from scenario_plotting import CsvTable, _plot_vector_history, _time_minutes, run_variant_plot

STEM = "scenarioTempMeasurementAttitude"


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_minutes(table)
    figures = {}
    for number, (prefix, ylabel) in enumerate(
        (
            ("rwTemperature_C", "RW Temperature [ºC] "),
            ("tempMeasurement_C", "RW Temperature Measurement [ºC] "),
        ),
        start=1,
    ):
        figure = plt.figure()
        _plot_vector_history(
            figure.gca(),
            time,
            table,
            prefix,
            lambda index: rf"$T_{{RW,{index + 1}}}$",
            ylabel,
            legend="upper left",
        )
        figures[f"{STEM}{number}"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, ("",), build))
