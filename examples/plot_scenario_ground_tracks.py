#!/usr/bin/env python3
"""Upstream `scenarioGroundTracks` figure from `scenarioGroundTracks.csv`: the
orbit radius of each of the five spacecraft."""

import numpy as np

from orbit_plotting import NANO2HOUR, existing, run_plots

# Upstream `ModelTag`s (without `bsk-`) and `scLineColor`.
ORBITS = (
    ("LEO", "red"),
    ("GEO", "blue"),
    ("GTO", "cyan"),
    ("Hyperbolic", "yellow"),
    ("Molniya", "green"),
)


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, ["scenarioGroundTracks"]):
        figure = plt.figure()
        axis = figure.gca()
        axis.ticklabel_format(useOffset=False, style="plain")
        hours = table.time_ns * NANO2HOUR
        for label, color in ORBITS:
            radii = np.linalg.norm(table.vectors(f"{label}_r_BN_N_m"), axis=1)
            axis.semilogy(hours, radii / 1000.0, color=color, label=f"bsk-{label}")
        axis.legend(loc="lower right")
        axis.set_xlabel("Time [hours]")
        axis.set_ylabel("Orbit Radius [km]")
        figures[f"{stem}1"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(__doc__.splitlines()[0], build))
