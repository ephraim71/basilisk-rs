#!/usr/bin/env python3
"""Figure of upstream scenarioSensorThermal.py from the Rust CSV.

The CSV ends where the attitude diverges after the pointing flip (see the
scenario docs).
"""
from __future__ import annotations

from typing import Any

from scenario_plotting import CsvTable, _time_hours, run_variant_plot

STEM = "scenarioSensorThermal"


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    figure = plt.figure()
    axis = figure.gca()
    axis.plot(_time_hours(table), table.column("temperature_C"))
    axis.set_title("Sensor Temperature")
    axis.set_xlabel("Time [h]")
    axis.set_ylabel("Temperature [deg C]")
    axis.grid(True)
    return {"scenario_thermalSensor": figure}


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, ("",), build))
