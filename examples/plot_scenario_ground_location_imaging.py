#!/usr/bin/env python3
"""Figures of upstream scenarioGroundLocationImaging.py from the Rust CSV.

The CSV ends where the slew to Santiago diverges (see the scenario docs).
"""
from __future__ import annotations

from typing import Any

from scenario_plotting import (
    CsvTable,
    _norm,
    _scaled,
    _time_minutes,
    _vector_rows,
    run_variant_plot,
)

STEM = "scenarioGroundLocationImaging"
BITS_PER_KB = 8e3


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_minutes(table)

    attitude = plt.figure()
    axis = attitude.gca()
    axis.plot(time, [_norm(row) for row in _vector_rows(table, "sigma_BR")])
    axis.set_xlabel("Time [min]")
    axis.set_ylabel(r"Attitude Error Norm $|\sigma_{B/R}|$")
    axis.set_yscale("log")

    access = plt.figure()
    axis = access.gca()
    axis.plot(time, table.column("hasAccess"))
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("Imaging Target Access")

    device = plt.figure()
    axis = device.gca()
    axis.plot(time, table.column("deviceCmd"))
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("Device Status")

    storage = plt.figure()
    axis = storage.gca()
    for column, label in (
        ("storageLevel_bits", "Data Unit Total Storage Level (KB)"),
        ("storedData_bits_0", "Boulder Partition Level (KB)"),
        ("storedData_bits_1", "Santiago Partition Level (KB)"),
    ):
        axis.plot(time, _scaled(table.column(column), 1.0 / BITS_PER_KB), label=label)
    axis.set_xlabel("Time (min)")
    axis.set_ylabel("Data Stored (KB)")
    axis.grid(True)
    axis.legend()

    return {f"{STEM}1": attitude, f"{STEM}2": access, f"{STEM}3": device, f"{STEM}4": storage}


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, ("",), build))
