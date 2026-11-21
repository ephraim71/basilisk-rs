#!/usr/bin/env python3
"""Figures of upstream scenarioAttitudeFeedbackRW.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from scenario_plotting import (
    CsvTable,
    _plot_attitude_error,
    _plot_motor_torque,
    _plot_vector_history,
    _plot_wheel_speeds,
    _time_minutes,
    run_variant_plot,
)

STEM = "scenarioAttitudeFeedbackRW"
# `<useJitterSimple><useRWVoltageIO>`; upstream's own default first.
VARIANTS = ("01", "00", "10")


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_minutes(table)

    attitude = plt.figure()
    _plot_attitude_error(attitude.gca(), time, table)

    torque = plt.figure()
    _plot_motor_torque(torque.gca(), time, table, 3, grid=False)

    speed = plt.figure()
    _plot_wheel_speeds(speed.gca(), time, table, 3, grid=False)

    figures = {
        f"{STEM}1{variant}": attitude,
        f"{STEM}2{variant}": torque,
        f"{STEM}3{variant}": speed,
    }
    if variant.endswith("1"):
        voltage = plt.figure()
        _plot_vector_history(
            voltage.gca(),
            time,
            table,
            "rwVoltage_V",
            lambda index: rf"$V_{index}$",
            "RW Voltage (V)",
        )
        figures[f"{STEM}4{variant}"] = voltage
    return figures


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, VARIANTS, build))
