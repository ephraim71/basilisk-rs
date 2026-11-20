#!/usr/bin/env python3
"""Upstream `scenarioGroundMapping` figures from
`scenarioGroundMapping<True|False>[Step<ms>ms].csv` and its `...Points.csv`:
the attitude error, the mapped points with the planet-fixed spacecraft path,
and the number of stored points.

With upstream's 5 s step the CSV ends where the attitude loop diverges (see
the scenario docs); `--step 0.5` gives the converged mapping.
"""

import numpy as np

from orbit_plotting import NANO2MIN, Table, run_plots

STEM = "scenarioGroundMapping"


def build(generated_dir, plt):
    figures = {}
    for path in sorted(generated_dir.glob(f"{STEM}*.csv")):
        if path.stem.endswith("Points"):
            continue
        points_path = path.with_name(f"{path.stem}Points.csv")
        if not points_path.is_file():
            continue
        table = Table(path)
        points = Table(points_path)
        suffix = path.stem[len(STEM):]
        time = table.time_ns * NANO2MIN

        figure = plt.figure()
        axis = figure.gca()
        axis.plot(time, np.linalg.norm(table.vectors("sigma_BR"), axis=1))
        axis.set_xlabel("Time [min]")
        axis.set_ylabel(r"Attitude Error Norm $|\sigma_{B/R}|$")
        axis.set_yscale("log")
        figures[f"{STEM}1{suffix}"] = figure

        figure = plt.figure()
        axis = figure.add_subplot(projection="3d")
        path_km = table.vectors("r_BP_P_m") / 1000
        axis.plot(path_km[:, 0], path_km[:, 1], path_km[:, 2], ".")
        mapped = points["mapped"] > 0.5
        points_km = points.vectors("r_LP_P_m")[mapped] / 1000
        axis.plot(points_km[:, 0], points_km[:, 1], points_km[:, 2], ".", c="g")
        axis.set_xlabel("X [km]")
        axis.set_ylabel("Y [km]")
        axis.set_zlabel("Z [km]")
        figures[f"{STEM}2{suffix}"] = figure

        figure = plt.figure()
        axis = figure.gca()
        axis.plot(time, table["storedPoints"])
        axis.set_xlabel("Time [min]")
        axis.set_ylabel("Stored Points")
        figures[f"{STEM}3{suffix}"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(__doc__.splitlines()[0], build))
