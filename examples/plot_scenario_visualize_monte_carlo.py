#!/usr/bin/env python3
"""Figures of upstream scenarioVisualizeMonteCarlo.py from the Rust CSVs.

Upstream's ``MonteCarloPlotter`` shows each component of
``attGuidMsg.sigma_BR`` and ``attGuidMsg.omega_BR_B`` with every run, coloured
by run number, highlights the runs entered by number, and titles the page
with the number of runs. ``scenario_visualize_monte_carlo`` writes each
variable and component as ``MonteCarloPlots_<variable>_<component>.csv``
(``time_ns`` and a ``run_<i>`` column per run); this draws them with
matplotlib, the runs of ``MONTE_CARLO_RUNS`` (comma-separated, e.g.
``1,3``) highlighted. Bokeh's interactive page has no counterpart here.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any

from orbit_plotting import NANO2SEC, Table, run_plots

VARIABLES = ("attGuidMsg.sigma_BR", "attGuidMsg.omega_BR_B")


def build(generated_dir: Path, plt: Any) -> dict[str, Any]:
    import matplotlib

    highlighted = {
        int(run) for run in os.environ.get("MONTE_CARLO_RUNS", "").split(",") if run.strip()
    }
    figures: dict[str, Any] = {}
    for variable in VARIABLES:
        for component in (1, 2, 3):
            name = f"MonteCarloPlots_{variable}_{component}"
            path = generated_dir / f"{name}.csv"
            if not path.is_file():
                continue
            table = Table(path)
            runs = sorted(int(column[4:]) for column in table.columns if column.startswith("run_"))
            colors = matplotlib.colormaps["viridis"].resampled(max(len(runs), 2))
            # Upstream fades the background lines with the number of runs.
            opacity = max(0.1, min(0.8, 10.0 / max(len(runs), 1)))
            figure = plt.figure()
            axis = figure.gca()
            time = table.time_ns * NANO2SEC
            for order, run in enumerate(runs):
                emphasis = run in highlighted
                axis.plot(time, table[f"run_{run}"], color=colors(order),
                          alpha=1.0 if emphasis else opacity, linewidth=3 if emphasis else 2,
                          label=f"run {run}" if emphasis else None)
            if highlighted:
                axis.legend()
            axis.set_title(f"{variable} - component {component}")
            axis.set_xlabel("Time [s]")
            axis.set_ylabel(variable)
            figure.suptitle(f"Monte Carlo Visualization: {len(runs)} runs")
            figures[name] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot scenarioVisualizeMonteCarlo results.", build))
