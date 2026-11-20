#!/usr/bin/env python3
"""Figures of upstream scenarioAttitudeFeedbackRWPower.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from scenario_plotting import (
    CsvTable,
    _plot_attitude_error,
    _plot_motor_torque,
    _plot_vector_history,
    _time_minutes,
    run_variant_plot,
)

STEM = "scenarioAttitudeFeedbackRWPower"
# `str(useRwPowerGeneration)`; upstream's own default first.
VARIANTS = ("True", "False")


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_minutes(table)

    attitude = plt.figure()
    _plot_attitude_error(attitude.gca(), time, table)

    torque = plt.figure()
    _plot_motor_torque(torque.gca(), time, table, 3, grid=False)

    power = plt.figure()
    _plot_vector_history(
        power.gca(),
        time,
        table,
        "rwPower_W",
        lambda index: rf"$p_{{rw,{index}}}$",
        "RW Power (W)",
    )

    battery = plt.figure()
    axis = battery.gca()
    axis.plot(time, table.column("storageLevel_Ws"))
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("Battery Storage (Ws)")

    return {
        f"{STEM}1{variant}": attitude,
        f"{STEM}2{variant}": torque,
        f"{STEM}3{variant}": power,
        f"{STEM}4{variant}": battery,
    }


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, VARIANTS, build))
