"""Shared figure helpers of the dynamics-engine comparison plot scripts.

The counterpart of upstream ``examples/dynamicsComparison/_comparePlots.py``:
each figure has one axis; a scalar comparison gives an overlay figure and,
with MuJoCo data, a difference figure (``<name>_difference``); a vector
comparison gives the pair per component (``<name>_x``,
``<name>_xDifference``, ...). BSM curves are thick translucent underlays,
MuJoCo curves thin lines on top. Also the work-precision (Pareto) figures of
``scenarioCompareParetoRwPanels`` and ``scenarioCompareParetoFlexPanels``.

The ``plot_scenario_compare_*.py`` scripts read the CSV files the Rust
examples write to ``examples/output/scenarios`` (columns ``bsm_*`` and, with
the ``mujoco`` feature, ``mujoco_*``) and use ``orbit_plotting.run_plots``
for the command line.
"""

from __future__ import annotations

from typing import Any

import numpy as np

# Paul Tol high-contrast palette (upstream COLOR_BSM, COLOR_MUJOCO, COLOR_DIFF).
COLOR_BSM = "#004488"
COLOR_MUJOCO = "#BB5566"
COLOR_DIFF = "#DDAA33"
AXES = ("x", "y", "z")

R2D = 180.0 / np.pi


def has(table: Any, column: str) -> bool:
    """Whether the CSV has ``column`` (the MuJoCo ones need the feature)."""
    return column in table.columns


def _plot_both(axis: Any, time: np.ndarray, y_bsm: np.ndarray, y_mj: np.ndarray | None,
               bsm_label: str, mj_label: str) -> None:
    axis.plot(time, y_bsm, lw=4, alpha=0.4, color=COLOR_BSM, label=bsm_label)
    if y_mj is not None:
        axis.plot(time, y_mj, lw=1.3, color=COLOR_MUJOCO, label=mj_label)


def _unit_of(label: str) -> str:
    if "[" in label and label.rstrip().endswith("]"):
        return label[label.rindex("[") + 1:label.rindex("]")]
    return ""


def scalar_comparison(plt: Any, name: str, time: np.ndarray, y_bsm: np.ndarray,
                      y_mj: np.ndarray | None, ylabel: str, xlabel: str = "Time [s]",
                      diff_scale: float = 1.0, diff_unit: str | None = None,
                      log_diff: bool = False, figsize=(7.0, 2.6)) -> dict[str, Any]:
    """Upstream ``scalarComparison``."""
    figures = {}
    figure, axis = plt.subplots(figsize=figsize)
    _plot_both(axis, time, y_bsm, y_mj, "BSM", "MuJoCo")
    axis.set_xlabel(xlabel)
    axis.set_ylabel(ylabel)
    if y_mj is not None:
        axis.legend(loc="best")
    figures[name] = figure
    if y_mj is not None:
        difference = np.asarray(y_bsm) - np.asarray(y_mj)
        figure, axis = plt.subplots(figsize=figsize)
        if log_diff:
            axis.semilogy(time, np.maximum(np.abs(difference) * diff_scale, 1e-16), color=COLOR_DIFF)
        else:
            axis.plot(time, difference * diff_scale, color=COLOR_DIFF)
        unit = diff_unit if diff_unit is not None else _unit_of(ylabel)
        axis.set_ylabel("Difference" + (f" [{unit}]" if unit else ""))
        axis.set_xlabel(xlabel)
        figures[name + "_difference"] = figure
    return figures


def component_comparison(plt: Any, name: str, time: np.ndarray, v_bsm: np.ndarray,
                         v_mj: np.ndarray | None, quantity: str, unit: str,
                         xlabel: str = "Time [s]", scale: float = 1.0,
                         figsize=(4.8, 2.4)) -> dict[str, Any]:
    """Upstream ``componentComparison``: overlay and difference per component."""
    figures = {}
    for column in range(3):
        component = f"{name}_{AXES[column]}"
        figure, axis = plt.subplots(figsize=figsize)
        _plot_both(axis, time, v_bsm[:, column] * scale, None if v_mj is None else v_mj[:, column] * scale,
                   "BSM", "MuJoCo")
        axis.set_xlabel(xlabel)
        axis.set_ylabel(f"{quantity}$_{AXES[column]}$ [{unit}]")
        if v_mj is not None:
            axis.legend(loc="best", fontsize=8)
        figures[component] = figure
        if v_mj is not None:
            figure, axis = plt.subplots(figsize=figsize)
            axis.plot(time, (v_bsm[:, column] - v_mj[:, column]) * scale, color=COLOR_DIFF)
            axis.set_xlabel(xlabel)
            axis.set_ylabel(f"Difference [{unit}]")
            figures[component + "Difference"] = figure
    return figures


def vectors_or_none(table: Any, prefix: str) -> np.ndarray | None:
    """The ``<prefix>_0..2`` columns, or ``None`` when absent."""
    return table.vectors(prefix) if has(table, f"{prefix}_0") else None


# --- Work-precision (Pareto) figures ----------------------------------------

# Upstream INTEGRATOR_STYLE: (color, marker, label), indexed as the Rust CSV's
# `integrator` column (Euler, RK2, RK4, RKF45, RKF78).
TOL_COLORS = ("#004488", "#BB5566", "#DDAA33")
INTEGRATOR_STYLE = (
    (TOL_COLORS[0], "o", "Euler"),
    (TOL_COLORS[1], "s", "RK2"),
    (TOL_COLORS[2], "^", "RK4"),
    (TOL_COLORS[0], "D", "RKF45 (adaptive)"),
    (TOL_COLORS[1], "v", "RKF78 (adaptive)"),
)


def pareto_rows(table: Any) -> list[dict[str, float]]:
    """The CSV rows as dictionaries."""
    names = list(table.columns)
    count = len(table[names[0]])
    return [{name: float(table[name][i]) for name in names} for i in range(count)]


def pareto_frontier(rows: list[dict[str, float]], key: str) -> list[dict[str, float]]:
    """Upstream ``paretoFrontier``: the non-dominated rows by wall-clock."""
    frontier, best = [], np.inf
    for row in sorted(rows, key=lambda r: (r["wall_s"], r[key])):
        if row[key] < best:
            frontier.append(row)
            best = row[key]
    return frontier


def _metric(row: dict[str, float], key: str) -> float:
    return row[key] * R2D if key == "error_rad" else row[key]


def pareto_scatter(plt: Any, rows: list[dict[str, float]], key: str, ylabel: str) -> Any:
    """Upstream ``_paretoScatter``: color by integrator, filled BSM, open MuJoCo."""
    figure, axis = plt.subplots(figsize=(8, 4))
    seen = set()
    for row in rows:
        color, marker, label = INTEGRATOR_STYLE[int(row["integrator"])]
        engine = "BSM" if row["engine"] == 0 else "mujoco"
        legend = None
        if (row["integrator"], engine) not in seen:
            legend = f"{label} ({engine})"
            seen.add((row["integrator"], engine))
        axis.scatter(row["wall_s"], _metric(row, key), marker=marker, s=45,
                     facecolors=color if engine == "BSM" else "none", edgecolors=color, label=legend)
    axis.set_xscale("log")
    axis.set_yscale("log")
    axis.set_xlabel("Wall-clock time [s]")
    axis.set_ylabel(ylabel)
    axis.grid(True, which="both", alpha=0.3)
    handles, labels = axis.get_legend_handles_labels()
    if handles:
        axis.legend(handles, labels, fontsize=7, ncol=1, loc="center left", bbox_to_anchor=(1.02, 0.5))
    return figure


def pareto_frontier_plot(plt: Any, rows: list[dict[str, float]], key: str, ylabel: str) -> Any:
    """Upstream ``_paretoFrontierPlot``."""
    figure, axis = plt.subplots(figsize=(8, 4))
    for engine, label, color in ((0, "Backsubstitution (BSM)", TOL_COLORS[0]),
                                 (1, "MuJoCo", TOL_COLORS[1])):
        engine_rows = [row for row in rows if row["engine"] == engine]
        if not engine_rows:
            continue
        frontier = pareto_frontier(engine_rows, key)
        axis.plot([r["wall_s"] for r in frontier], [_metric(r, key) for r in frontier],
                  "-o", color=color, label=label)
    axis.set_xscale("log")
    axis.set_yscale("log")
    axis.set_xlabel("Wall-clock time [s]")
    axis.set_ylabel(ylabel)
    axis.grid(True, which="both", alpha=0.3)
    handles, labels = axis.get_legend_handles_labels()
    if handles:
        axis.legend(handles, labels, loc="center left", bbox_to_anchor=(1.02, 0.5))
    return figure


def pareto_figures(plt: Any, table: Any, stem: str, position_label: str) -> dict[str, Any]:
    """Upstream ``plotResults`` of both Pareto scenarios."""
    rows = pareto_rows(table)
    attitude = "Attitude error [deg]"
    return {
        f"{stem}_pareto": pareto_scatter(plt, rows, "error_rad", attitude),
        f"{stem}_frontier": pareto_frontier_plot(plt, rows, "error_rad", attitude),
        f"{stem}_paretoPosition": pareto_scatter(plt, rows, "position_error_m", position_label),
        f"{stem}_frontierPosition": pareto_frontier_plot(plt, rows, "position_error_m", position_label),
    }
