#!/usr/bin/env python3
"""Upstream `scenarioDragDeorbit` figures from `scenarioDragDeorbit<model>0.csv`:
the orbit, the altitude, the density over altitude and the drag force, named
`scenarioDragDeorbit<model><n>` as upstream. `--variant all` plots both models;
the default is the MSIS run."""

from scenario_plotting import _build_drag_deorbit, run_variant_plot


def build(plt, table, variant):
    model = variant[:-1]  # drop upstream's useWind digit
    return {
        name.replace("scenarioDragDeorbitmsis", f"scenarioDragDeorbit{model}"): figure
        for name, figure in _build_drag_deorbit(plt, table).items()
    }


if __name__ == "__main__":
    raise SystemExit(
        run_variant_plot(
            "Plot the drag deorbit scenario.",
            "scenarioDragDeorbit",
            ("msis0", "exponential0"),
            build,
        )
    )
