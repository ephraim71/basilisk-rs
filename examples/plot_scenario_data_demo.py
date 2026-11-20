#!/usr/bin/env python3
"""Figures of upstream scenarioDataDemo.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from scenario_plotting import CsvTable, _scaled, _time_hours, run_variant_plot

STEM = "scenarioDataDemo"
BITS_PER_KB = 8e3


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_hours(table)

    bits = plt.figure()
    axis = bits.gca()
    for column, label in (
        ("storageLevel_bits", "Data Unit Total Storage Level (KB)"),
        ("storedData_bits_0", "Instrument 1 Partition Level (KB)"),
        ("storedData_bits_1", "Instrument 2 Partition Level (KB)"),
    ):
        axis.plot(time, _scaled(table.column(column), 1.0 / BITS_PER_KB), label=label)
    axis.set_xlabel("Time (Hr)")
    axis.set_ylabel("Data Stored (KB)")
    axis.grid(True)
    axis.legend()

    baud = plt.figure()
    axis = baud.gca()
    axis.plot(
        time,
        _scaled(table.column("currentNetBaud_bps"), 1.0 / BITS_PER_KB),
        label="Net Baud Rate (KB/s)",
    )
    axis.set_xlabel("Time (Hr)")
    axis.set_ylabel("Data Rate (KB/s)")
    axis.grid(True)
    axis.legend()

    return {"scenario_dataDemoBits": bits, "scenario_dataDemoBaud": baud}


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, ("",), build))
