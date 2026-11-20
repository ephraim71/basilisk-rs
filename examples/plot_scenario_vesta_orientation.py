#!/usr/bin/env python3
"""Figures of upstream scenarioVestaOrientation.py from the Rust CSV files."""
from __future__ import annotations

import csv
from pathlib import Path
from typing import Any

import numpy as np

from orbit_plotting import NANO2SEC, PlotError, Table, line_color, run_plots

STEM = "scenarioVestaOrientation"
# Vesta's reference radius [m] (VESTA20H.txt header) and upstream's steps [s].
VESTA_RADIUS_M = 265000.0
REFERENCE_DT_S = 1.0


def query_counts(generated_dir: Path) -> dict[str, int]:
    path = generated_dir / f"{STEM}Queries.csv"
    try:
        with path.open("r", encoding="utf-8", newline="") as handle:
            return {row["run"]: int(row["orientation_queries"]) for row in csv.DictReader(handle)}
    except (OSError, KeyError, ValueError) as error:
        raise PlotError(f"cannot read {path}: {error}") from error


def plot_trajectory(plt: Any, positions: np.ndarray) -> Any:
    """Upstream plotTrajectory: the reference path with Vesta to scale."""
    figure = plt.figure(figsize=(7, 7))
    axis = figure.add_subplot(projection="3d")
    p = positions / 1e3
    axis.plot(p[:, 0], p[:, 1], p[:, 2], color=line_color(1, 2, plt), lw=1.5,
              label="spacecraft trajectory")
    u, v = np.mgrid[0:2 * np.pi:40j, 0:np.pi:20j]
    r = VESTA_RADIUS_M / 1e3
    axis.plot_surface(r * np.cos(u) * np.sin(v), r * np.sin(u) * np.sin(v), r * np.cos(v),
                      color="0.7", alpha=0.5, linewidth=0)
    axis.set_xlabel("x  [km]")
    axis.set_ylabel("y  [km]")
    axis.set_zlabel("z  [km]")
    axis.set_box_aspect((1, 1, 1))
    axis.legend(loc="upper left")
    return figure


def plot_error_over_time(plt: Any, times: np.ndarray, exact_error: np.ndarray, exact_queries: int,
                         one_error: np.ndarray, one_queries: int) -> Any:
    """Upstream plotErrorOverTime: both coarse runs have the same error."""
    figure, axis = plt.subplots(figsize=(8, 6))
    axis.plot(times / 3600.0, exact_error, color=line_color(0, 2, plt), lw=3.5, alpha=0.7,
              zorder=4, label=f"exact SPICE every sub-step ({exact_queries} queries)")
    axis.plot(times / 3600.0, one_error, color=line_color(1, 2, plt), lw=1.8, ls="--", zorder=5,
              label=f"velocity extrapolation ({one_queries} SPICE query)")
    axis.set_xlim(0, times[-1] / 3600.0)
    axis.set_ylim(bottom=0.0)
    axis.set_xlabel("time  [hr]")
    axis.set_ylabel(f"orbit error vs. {REFERENCE_DT_S:g} s reference  [m]")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="upper left")
    return figure


def build(generated_dir: Path, plt: Any) -> dict[str, Any]:
    reference = Table(generated_dir / f"{STEM}Reference.csv")
    exact = Table(generated_dir / f"{STEM}Exact.csv")
    one = Table(generated_dir / f"{STEM}Extrapolated.csv")
    queries = query_counts(generated_dir)

    reference_time = reference.time_ns * NANO2SEC
    reference_position = reference.vectors("r_BN_N_m")
    times = exact.time_ns * NANO2SEC
    reference_at_coarse = np.vstack(
        [np.interp(times, reference_time, reference_position[:, i]) for i in range(3)]
    ).T
    exact_error = np.linalg.norm(exact.vectors("r_BN_N_m") - reference_at_coarse, axis=1)
    one_error = np.linalg.norm(one.vectors("r_BN_N_m") - reference_at_coarse, axis=1)

    plt.rcParams.update({"font.size": 13, "lines.linewidth": 2.0, "lines.markersize": 8})
    return {
        f"{STEM}1": plot_trajectory(plt, reference_position),
        f"{STEM}2": plot_error_over_time(plt, times, exact_error, queries["Exact"], one_error,
                                         queries["Extrapolated"]),
    }


if __name__ == "__main__":
    raise SystemExit(run_plots(f"Plot {STEM} results.", build))
