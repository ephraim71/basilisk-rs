#!/usr/bin/env python3
"""Figures of upstream BskSim scenario_AddRWFault.py from the Rust CSVs.

The fault markers of the friction figure come from
``scenario_AddRWFaultLog.csv`` (upstream's ``RWFaultLog``).
"""
from __future__ import annotations

import csv
from pathlib import Path
from typing import Any

import bsk_sim_plotting as bsk
from scenario_plotting import DEFAULT_GENERATED_DIR, CsvTable

STEM = "scenario_AddRWFault"


def fault_times(generated_dir: Path) -> list[float]:
    path = generated_dir / f"{STEM}Log.csv"
    if not path.exists():
        return []
    with path.open(encoding="utf-8") as handle:
        return [float(row["time_min"]) for row in csv.DictReader(handle)]


def build(plt: Any, table: CsvTable, generated_dir: Path = DEFAULT_GENERATED_DIR) -> dict[str, Any]:
    table = bsk.drop_first(table)
    time = bsk.minutes(table)
    return {
        "attitudeErrorNorm": bsk.plot_attitude_error(plt, time, bsk.vectors(table, "sigma_BR")),
        "rateError": bsk.plot_rate_error(plt, time, bsk.vectors(table, "omega_BR_B_rad_per_s")),
        "RWSpeeds": bsk.plot_rw_speeds(plt, time, bsk.vectors(table, "wheelSpeeds_rad_per_s", 4), 4),
        "RWFriction": bsk.plot_rw_friction(
            plt, time, bsk.vectors(table, "rwFriction_Nm", 4), 4, fault_times(generated_dir)
        ),
    }


if __name__ == "__main__":
    import sys

    directory = DEFAULT_GENERATED_DIR
    if "--generated-dir" in sys.argv:
        directory = Path(sys.argv[sys.argv.index("--generated-dir") + 1])
    raise SystemExit(bsk.run(STEM, lambda plt, table: build(plt, table, directory)))
