#!/usr/bin/env python3
"""Figure of upstream scenarioPowerDemo.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from scenario_plotting import CsvTable, _scaled, _time_hours, run_variant_plot

STEM = "scenarioPowerDemo"


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_hours(table)
    figure = plt.figure()
    axis = figure.gca()
    axis.plot(
        time,
        _scaled(table.column("storageLevel_Ws"), 1.0 / 3600.0),
        label="Stored Power (W-Hr)",
    )
    axis.plot(time, table.column("currentNetPower_W"), label="Net Power (W)")
    axis.plot(time, table.column("panelPower_W"), label="Panel Power (W)")
    axis.plot(time, table.column("sinkPower_W"), label="Power Draw (W)")
    axis.set_xlabel("Time (Hr)")
    axis.set_ylabel("Power (W)")
    axis.grid(True)
    axis.legend()
    return {"scenario_powerDemo": figure}


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, ("",), build))
