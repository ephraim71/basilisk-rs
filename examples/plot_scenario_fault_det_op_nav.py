#!/usr/bin/env python3
"""Figures of upstream scenario_faultDetOpNav.py from the Rust CSV: the
faults detected with the validity of both measurements, the filter's
percent errors at the merged measurements, and its state errors with their
covariance."""
from __future__ import annotations

from pathlib import Path
from typing import Any

import opnav_plotting as opnav
from orbit_plotting import Table, run_plots

STEM = "scenario_faultDetOpNav"


def build(generated_dir: Path, plt: Any) -> dict[str, Any]:
    table = Table(generated_dir / f"{STEM}.csv")
    time = table.time_ns
    error, covar = opnav.filter_errors(table)
    figures = {}
    figures.update(opnav.plot_faults(plt, time, table["opnav_faultDetected"], table["primary_valid"],
                                     table["secondary_valid"]))
    figures.update(opnav.nav_percentages(
        plt, time, opnav.columns(table, "r_BN_N_m", 3), opnav.columns(table, "v_BN_N_m_per_s", 3),
        opnav.columns(table, "filter_state", 6), opnav.np.nan_to_num(covar), table["opnav_valid"], "Fault"))
    figures.update(opnav.plot_state_covar(plt, time, error, covar))
    return {f"{STEM}_{name}": figure for name, figure in figures.items()}


if __name__ == "__main__":
    raise SystemExit(run_plots(f"Plot {STEM} results.", build))
