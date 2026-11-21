#!/usr/bin/env python3
"""Upstream `scenarioLambertSolver` figures from `scenarioLambertSolver.csv`
(truth and navigation, 10 s) and `scenarioLambertSolverFsw.csv` (flight
software, 30 s, up to the maneuver): position and velocity with the target,
the maneuver position r1 and velocity v1, the delta-v command, the solver's
x, iteration count and error, and the delta-v convergence flag."""

import numpy as np

from orbit_plotting import NANO2SEC, PlotError, Table, run_plots

R_EARTH = 6378 * 1e3
TARGET = np.array([-(R_EARTH + 200 * 1e3), 0.0, 0.0])


def three_axes(plt, title):
    figure, axes = plt.subplots(3, sharex=True, figsize=(12, 6))
    axes[0].set_title(title)
    axes[2].set_xlabel("Time [sec]")
    return figure, axes


def build(generated_dir, plt):
    truth_path = generated_dir / "scenarioLambertSolver.csv"
    fsw_path = generated_dir / "scenarioLambertSolverFsw.csv"
    if not truth_path.is_file() or not fsw_path.is_file():
        raise PlotError("scenarioLambertSolver CSVs are missing; run scenario_lambert_solver")
    truth, fsw = Table(truth_path), Table(fsw_path)
    time = truth.time_ns * NANO2SEC
    # Upstream appends a NaN at the final time to the FSW histories.
    time_fsw = np.append(fsw.time_ns * NANO2SEC, time[-1])

    def fsw_column(name):
        return np.append(fsw[name], np.nan)

    def fsw_vectors(prefix):
        return np.vstack([fsw.vectors(prefix), np.full(3, np.nan)])

    figures = {}
    figure, axes = three_axes(plt, "Spacecraft Position")
    for index in range(3):
        axes[index].plot(time, truth[f"nav_r_BN_N_m_{index}"], "k*", markersize=2,
                         label="measurement" if index == 0 else None)
        axes[index].plot(time, truth[f"r_BN_N_m_{index}"], label="truth" if index == 0 else None)
        axes[index].plot(time[-1], TARGET[index], "rx", label="target" if index == 0 else None)
        axes[index].set_ylabel(f"${{}}^Nr_{{BN_{index + 1}}}$ [m]")
    axes[0].legend(loc="upper right")
    figures["scenarioLambertSolver1"] = figure

    figure, axes = three_axes(plt, "Spacecraft Velocity")
    for index in range(3):
        axes[index].plot(time, truth[f"nav_v_BN_N_m_per_s_{index}"], "k*", markersize=2,
                         label="measurement" if index == 0 else None)
        axes[index].plot(time, truth[f"v_BN_N_m_per_s_{index}"], label="truth" if index == 0 else None)
        axes[index].set_ylabel(f"${{}}^Nv_{{BN_{index + 1}}}$ [m/s]")
    axes[0].legend(loc="upper right")
    figures["scenarioLambertSolver2"] = figure

    for number, prefix, title, unit in (
        (3, "r1_N_m", "Expected Spacecraft Position at Maneuver Time", "${{}}^Nr_{{1,{i}}}$ [m]"),
        (4, "v1_N_m_per_s", "Lambert Solution Velocity at Maneuver Time", "${{}}^Nv_{{1,{i}}}$ [m/s]"),
        (5, "dvInrtlCmd_m_per_s", "Delta-V Command", r"${{}}^N\Delta v_{{{i}}}$ [m/s]"),
    ):
        figure, axes = three_axes(plt, title)
        values = fsw_vectors(prefix)
        for index in range(3):
            axes[index].plot(time_fsw, values[:, index], ".")
            axes[index].set_ylabel(unit.format(i=index + 1))
        figures[f"scenarioLambertSolver{number}"] = figure

    for number, name, label in (
        (6, "x", "x [-]"),
        (7, "numIter", "number of iterations [-]"),
        (8, "errX", "Error in x [-]"),
        (9, "failedDvSolutionConvergence", "Failed DV convergence [-]"),
    ):
        figure = plt.figure(figsize=(12, 3))
        axis = figure.gca()
        axis.plot(time_fsw, fsw_column(name), ".")
        axis.set_xlabel("Time [sec]")
        axis.set_ylabel(label)
        figures[f"scenarioLambertSolver{number}"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the Lambert solver scenario.", build))
