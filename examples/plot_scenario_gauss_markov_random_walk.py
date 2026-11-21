#!/usr/bin/env python3
"""Figure of upstream scenarioGaussMarkovRandomWalk.py from the Rust CSV."""
from __future__ import annotations

from typing import Any

from scenario_plotting import CsvTable, _time_seconds, run_variant_plot

STEM = "scenarioGaussMarkovRandomWalk"
WALK_BOUNDS = 3.0


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_seconds(table)
    figure = plt.figure(figsize=(12, 8))
    axis = figure.gca()
    axis.plot(
        time,
        table.column("AngVelPlatform1_rad_per_s_0"),
        label="IMU 1 (Bounded Random Walk)",
        alpha=0.7,
    )
    axis.plot(
        time,
        table.column("AngVelPlatform2_rad_per_s_0"),
        label="IMU 2 (Pure Gaussian)",
        alpha=0.7,
    )
    axis.axhline(y=WALK_BOUNDS, color="r", linestyle="--", alpha=0.3, label="IMU1 Bounds")
    axis.axhline(y=-WALK_BOUNDS, color="r", linestyle="--", alpha=0.3)
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Angular Velocity (rad/s)")
    axis.set_title("IMU Gyro Measurements: Random Walk vs Pure Gaussian")
    axis.legend()
    axis.grid(True)
    return {STEM: figure}


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, ("",), build))
