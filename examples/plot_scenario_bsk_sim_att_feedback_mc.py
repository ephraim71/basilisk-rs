#!/usr/bin/env python3
"""Figure of upstream scenarioBskSimAttFeedbackMC.py from the Rust CSVs: its
data callback ``displayPlots`` draws every run's ``attGuidMsg.sigma_BR``
components on one figure. The Rust campaign writes each run's retained data
to ``scenarioBskSimAttFeedbackMC_run<i>.csv``; this overlays them."""
from __future__ import annotations

import re
from pathlib import Path
from typing import Any

from orbit_plotting import NANO2SEC, Table, run_plots

STEM = "scenarioBskSimAttFeedbackMC"


def build(generated_dir: Path, plt: Any) -> dict[str, Any]:
    pattern = re.compile(rf"^{STEM}_run(\d+)\.csv$")
    runs = sorted(
        (int(match.group(1)), path)
        for path in generated_dir.glob(f"{STEM}_run*.csv")
        if (match := pattern.match(path.name))
    )
    if not runs:
        return {}
    figure = plt.figure(1)
    axis = figure.gca()
    for _, path in runs:
        table = Table(path)
        time = table.time_ns * NANO2SEC
        sigma = table.vectors("sigma_BR")
        axis.plot(time, sigma[:, 0], time, sigma[:, 1], time, sigma[:, 2])
    axis.set_xlabel("Time [s]")
    axis.set_ylabel(r"Attitude Error $\sigma_{B/R}$")
    return {f"{STEM}_sigma_BR": figure}


if __name__ == "__main__":
    raise SystemExit(run_plots(f"Plot {STEM} results.", build))
