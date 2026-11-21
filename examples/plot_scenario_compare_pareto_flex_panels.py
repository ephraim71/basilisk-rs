#!/usr/bin/env python3
"""Upstream `scenarioCompareParetoFlexPanels` figures from
`scenarioCompareParetoFlexPanels.csv`: the work-precision scatter and
frontier of the stability-limited 16-segment arrays, for the attitude
(`_pareto`, `_frontier`) and the hub position (`_paretoPosition`,
`_frontierPosition`)."""

from dyncomp_plotting import pareto_figures
from orbit_plotting import existing, run_plots

STEM = "scenarioCompareParetoFlexPanels"


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        figures.update(pareto_figures(plt, table, STEM, "Final hub position error [m]"))
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(__doc__, build))
