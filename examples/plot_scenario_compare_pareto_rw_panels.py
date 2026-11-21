#!/usr/bin/env python3
"""Upstream `scenarioCompareParetoRwPanels` figures from
`scenarioCompareParetoRwPanels.csv`: every configuration's final attitude
and hub position error against wall-clock (`_pareto`, `_paretoPosition`;
color and marker by integrator, filled BSM, open MuJoCo) and each engine's
non-dominated frontier (`_frontier`, `_frontierPosition`)."""

from dyncomp_plotting import pareto_figures
from orbit_plotting import existing, run_plots

STEM = "scenarioCompareParetoRwPanels"


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        figures.update(pareto_figures(plt, table, STEM, "Final hub\nposition error [m]"))
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(__doc__, build))
