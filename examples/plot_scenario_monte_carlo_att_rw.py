#!/usr/bin/env python3
"""Figures of upstream scenarioMonteCarloAttRW.py from the Rust CSVs.

Upstream's data callback ``plotSim`` draws each run over the same five
figures. The Rust campaign writes each run's retained data to
``scenarioMonteCarloAttRW_run<i>.csv`` (case 1) or
``scenarioMonteCarloAttRW_IC<i>.csv`` (case 2, ``--ic``); this overlays them.
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Any

from scenario_plotting import (
    DEFAULT_GENERATED_DIR,
    RPM_RAD_PER_S,
    PlotError,
    _load_csv,
    _time_minutes,
)

STEM = "scenarioMonteCarloAttRW"


def main() -> int:
    parser = argparse.ArgumentParser(description=f"Plot {STEM} results.")
    parser.add_argument("--generated-dir", type=Path, default=DEFAULT_GENERATED_DIR)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--ic", action="store_true", help="plot the initial-condition runs (case 2)")
    parser.add_argument("--show", action="store_true")
    arguments = parser.parse_args()

    kind = "IC" if arguments.ic else "run"
    pattern = re.compile(rf"{STEM}_{kind}(\d+)\.csv$")
    paths = sorted(
        (int(match.group(1)), path)
        for path in arguments.generated_dir.glob(f"{STEM}_{kind}*.csv")
        if (match := pattern.search(path.name))
    )
    if not paths:
        print(f"error: no {STEM}_{kind}<i>.csv in {arguments.generated_dir}", file=sys.stderr)
        return 1
    try:
        tables = [(index, _load_csv(path)) for index, path in paths]
    except PlotError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1

    import matplotlib

    if not arguments.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figures: dict[str, Any] = {
        name: plt.figure()
        for name in ("AttitudeError", "RWMotorTorque", "RateTrackingError", "RWSpeed", "RWVoltage")
    }
    for run, table in tables:
        time = _time_minutes(table)
        for index in range(3):
            figures["AttitudeError"].gca().plot(
                time, table.column(f"sigma_BR_{index}"), label=rf"Run {run} $\sigma_{index}$")
            figures["RWMotorTorque"].gca().plot(
                time, table.column(f"motorTorque_Nm_{index}"), "--",
                label=rf"Run {run} $\hat u_{{s,{index}}}$")
            figures["RWMotorTorque"].gca().plot(
                time, table.column(f"rwTorque_Nm_{index}"), label=rf"Run {run} $u_{{s,{index}}}$")
            figures["RateTrackingError"].gca().plot(
                time, table.column(f"omega_BR_B_rad_per_s_{index}"),
                label=rf"Run {run} $\omega_{{BR,{index}}}$")
            figures["RWSpeed"].gca().plot(
                time, [value / RPM_RAD_PER_S for value in table.column(f"wheelSpeeds_rad_per_s_{index}")],
                label=rf"Run {run} $\Omega_{{{index}}}$")
            figures["RWVoltage"].gca().plot(
                time, table.column(f"rwVoltage_V_{index}"), label=rf"Run {run} $V_{{{index}}}$")
    labels = {
        "AttitudeError": r"Attitude Error $\sigma_{B/R}$",
        "RWMotorTorque": "RW Motor Torque (Nm)",
        "RateTrackingError": "Rate Tracking Error (rad/s) ",
        "RWSpeed": "RW Speed (RPM) ",
        "RWVoltage": "RW Voltage (V) ",
    }
    output_dir = arguments.output_dir or arguments.generated_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    for name, figure in figures.items():
        axis = figure.gca()
        axis.set_xlabel("Time [min]")
        axis.set_ylabel(labels[name])
        output = output_dir / f"{STEM}{'_IC' if arguments.ic else ''}_{name}.png"
        figure.tight_layout()
        figure.savefig(output, dpi=160)
        print(f"saved plot: {output}")
    if arguments.show:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
