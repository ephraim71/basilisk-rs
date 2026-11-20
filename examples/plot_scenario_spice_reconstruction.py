#!/usr/bin/env python3
"""Figures of upstream scenarioSpiceReconstruction.py from the Rust CSVs:
the cislunar flyby, the accuracy-versus-cost sweep and the errors over time.

``scenarioSpiceReconstruction.csv`` holds the exact-SPICE reference
trajectory and the Moon's, ``...Sweep.csv`` one row per run (``kind`` 0 a
reconstruction grid, 1 the classic spacecraft, 2 the exact reference) and
``...Errors.csv`` the errors against the reference over time.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from scenario_plotting import (
    DEFAULT_GENERATED_DIR,
    PlotError,
    _cross,
    _dot,
    _line_color,
    _load_csv,
    _unit,
    _vector_rows,
)

STEM = "scenarioSpiceReconstruction"
C_RECON, C_CLASSIC, C_EXACT = _line_color(0), _line_color(1), _line_color(2)


def plot_trajectory(plt, table):
    """Figure 1: both Earth-centred paths in the spacecraft's orbit plane
    (upstream projects on the path's two principal directions; the plane
    here is the mean angular-momentum plane), closest approach marked."""
    spacecraft = _vector_rows(table, "r_BN_N_m")
    moon = _vector_rows(table, "moon_r_N_m")
    momentum = [0.0, 0.0, 0.0]
    for first, second in zip(spacecraft, spacecraft[1:]):
        momentum = [a + b for a, b in zip(momentum, _cross(first, second))]
    normal = _unit(momentum)
    e1 = _unit(_cross(_cross(normal, spacecraft[0]), normal))
    e2 = _cross(normal, e1)
    project = lambda points: ([_dot(p, e1) / 1e3 for p in points], [_dot(p, e2) / 1e3 for p in points])
    sc_x, sc_y = project(spacecraft)
    moon_x, moon_y = project(moon)
    distances = [sum((a - b) ** 2 for a, b in zip(s, m)) for s, m in zip(spacecraft, moon)]
    closest = distances.index(min(distances))
    figure = plt.figure(figsize=(8, 7))
    axis = figure.gca()
    axis.plot(sc_x, sc_y, color=C_RECON, lw=2.0, label="spacecraft trajectory", zorder=4)
    half = len(moon_x) // 2
    axis.plot(moon_x[half:], moon_y[half:], color="0.55", lw=1.5, ls="--", label="Moon trajectory")
    axis.plot(0, 0, "o", color="0.2", ms=12, label="Earth")
    axis.plot(sc_x[0], sc_y[0], "o", color=C_RECON, ms=8)
    axis.plot(moon_x[closest], moon_y[closest], "o", color="0.35", ms=16, label="Moon at closest approach")
    axis.plot(sc_x[closest], sc_y[closest], "o", color=C_EXACT, ms=9, markeredgecolor="k")
    axis.plot([sc_x[closest], moon_x[closest]], [sc_y[closest], moon_y[closest]], color="k", lw=1.0, ls=":")
    axis.set_xlabel("in-plane distance from Earth  [km]")
    axis.set_ylabel("in-plane distance from Earth  [km]")
    axis.set_aspect("equal")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="lower left")
    return figure


def plot_accuracy_vs_cost(plt, table):
    """Figure 2: final-position error against Moon queries, with the
    integrator floor and its free zone."""
    rows = list(zip(table.column("kind"), table.column("moon_queries"), table.column("error_m")))
    floor = table.column("integrator_floor_m")[0]
    by_queries = {}
    for kind, queries, error in rows:
        if kind == 0 and (queries not in by_queries or error < by_queries[queries]):
            by_queries[queries] = error
    queries = sorted(by_queries)
    errors = [by_queries[q] for q in queries]
    classic = next((q, e) for kind, q, e in rows if kind == 1)
    exact = next(q for kind, q, _ in rows if kind == 2)
    plot_floor = 1e-6
    top = max(classic[1], floor, max(errors)) * 3.0
    figure = plt.figure(figsize=(8, 6))
    axis = figure.gca()
    axis.set_xscale("log")
    axis.set_yscale("log")
    axis.set_ylim(plot_floor, top)
    axis.axhspan(plot_floor, floor, color="0.85", alpha=0.6, zorder=0)
    axis.axhline(floor, color="k", lw=1.5, label=f"integrator error floor ({floor:.0f} m)")
    axis.plot(queries, [max(e, plot_floor) for e in errors], "-o", color=C_RECON,
              label="Moon reconstruction (cubic Hermite)")
    axis.plot(classic[0], max(classic[1], plot_floor), "s", color=C_CLASSIC, ms=12,
              label="classic spacecraft + gravityEffector")
    axis.plot(exact, plot_floor, "D", color=C_EXACT, ms=10, markeredgecolor="k",
              label=f"exact SPICE reference ({exact:.0f} queries, zero error)")
    axis.set_xlabel("Moon SPICE queries over the run  (compute cost)")
    axis.set_ylabel("orbit final-position error vs. exact-SPICE reference  [m]")
    axis.grid(True, which="both", alpha=0.3)
    axis.legend(loc="upper center")
    return figure


def plot_error_over_time(plt, table):
    """Figure 3: the errors against the exact reference over time."""
    hours = [t * 1e-9 / 3600.0 for t in table.column("time_ns")]
    clip = lambda values: [max(v, 1e-6) for v in values]
    floor = clip(table.column("integrator_error_m"))
    recon = clip(table.column("reconstruction_error_m"))
    classic = clip(table.column("classic_error_m"))
    figure = plt.figure(figsize=(8, 6))
    axis = figure.gca()
    axis.fill_between(hours, 1e-6, floor, color="0.85", alpha=0.6, zorder=0)
    axis.semilogy(hours, floor, color="k", lw=1.5, label="integrator error (perfect ephemeris)")
    axis.semilogy(hours, recon, color=C_RECON,
                  label=f"reconstruction, {table.column('reconstruction_moon_queries')[0]:.0f} Moon queries")
    axis.semilogy(hours, classic, color=C_CLASSIC, ls="--",
                  label=f"classic spacecraft + gravityEffector ({table.column('classic_moon_queries')[0]:.0f} queries)")
    axis.set_ylim(1e-6, max(max(floor), max(recon), max(classic)) * 3.0)
    axis.set_xlabel("time  [hr]")
    axis.set_ylabel("position error vs. exact SPICE  [m]")
    axis.grid(True, which="both", alpha=0.3)
    axis.legend(loc="lower right")
    return figure


def main() -> int:
    parser = argparse.ArgumentParser(description=f"Plot {STEM} results.")
    parser.add_argument("--generated-dir", type=Path, default=DEFAULT_GENERATED_DIR)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--show", action="store_true")
    arguments = parser.parse_args()
    try:
        trajectory = _load_csv(arguments.generated_dir / f"{STEM}.csv")
        sweep = _load_csv(arguments.generated_dir / f"{STEM}Sweep.csv")
        errors = _load_csv(arguments.generated_dir / f"{STEM}Errors.csv")
    except PlotError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1

    import matplotlib

    if not arguments.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figures = {
        f"{STEM}1": plot_trajectory(plt, trajectory),
        f"{STEM}2": plot_accuracy_vs_cost(plt, sweep),
        f"{STEM}3": plot_error_over_time(plt, errors),
    }
    output_dir = arguments.output_dir or arguments.generated_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    for name, figure in figures.items():
        output = output_dir / f"{name}.png"
        figure.tight_layout()
        figure.savefig(output, dpi=160)
        print(f"saved plot: {output}")
    if arguments.show:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
