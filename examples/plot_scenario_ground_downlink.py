#!/usr/bin/env python3
"""Figures of upstream scenarioGroundDownlink.py from the Rust CSV.

The spacecraft position in the CSV is already relative to the Earth, which
upstream gets by subtracting the planet position. Upstream's ECI figure plots
the ground station's x component twice (x, x, z); here it is (x, y, z).
"""
from __future__ import annotations

import math
from typing import Any

from scenario_plotting import (
    CsvTable,
    _scaled,
    _time_hours,
    _vector_columns,
    run_variant_plot,
)

STEM = "scenarioGroundDownlink"
BITS_PER_KB = 8e3


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_hours(table)

    storage = plt.figure()
    axis = storage.gca()
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

    eci = plt.figure()
    axis = eci.add_subplot(1, 1, 1, projection="3d")
    spacecraft = [_scaled(column, 1e-3) for column in _vector_columns(table, "r_BN_N_m", 3)]
    station = [_scaled(column, 1e-3) for column in _vector_columns(table, "r_LP_N_m", 3)]
    axis.plot(*spacecraft, label="S/C Position")
    axis.plot(*station, label="Ground Station Position")
    axis.legend()

    access = table.column("hasAccess")
    azimuth = table.column("azimuth_rad")
    elevation = table.column("elevation_rad")
    passes = [index for index, value in enumerate(access) if value]
    polar = plt.figure()
    axis = polar.add_subplot(1, 1, 1, projection="polar")
    axis.plot(
        [azimuth[index] for index in passes],
        [90.0 - math.degrees(elevation[index]) for index in passes],
    )
    axis.set_title("Ground Pass Azimuth and Declination")

    angles = plt.figure()
    axis = angles.gca()
    axis.plot(time, [math.degrees(value) for value in azimuth], label="az")
    axis.plot(time, [math.degrees(value) for value in elevation], label="el")
    axis.legend()
    axis.grid(True)
    axis.set_ylabel("Angles (deg)")
    axis.set_xlabel("Time (hr)")

    slant_range = plt.figure()
    axis = slant_range.gca()
    axis.plot(time, _scaled(table.column("slantRange_m"), 1e-3))
    axis.plot(time, _scaled(access, 1000.0))
    axis.grid(True)
    axis.set_title("Slant Range, Access vs. Time")
    axis.set_ylabel("Slant Range (km)")
    axis.set_xlabel("Time (hr)")

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

    return {
        "scenarioGroundPassStorage": storage,
        "scenarioGroundPassECI": eci,
        "scenarioGroundPassPolar": polar,
        "scenarioGroundPassAngles": angles,
        "scenarioGroundPassRange": slant_range,
        "scenarioGroundPassBaud": baud,
    }


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, ("",), build))
