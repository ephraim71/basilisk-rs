"""Upstream ``examples/MultiSatBskSim/plottingMultiSat/BSK_MultiSatPlotting.py``
for the ``plot_multi_sat_*.py`` scripts.

Each function draws one of upstream's MultiSat figures on a new figure of
``plt`` from arrays the scripts read out of the Rust CSV files (see
``examples/support/multi_sat/logs.rs`` for the columns), and returns it.
The scripts collect the figures under upstream's names (``<scenario>_<figure>``)
and ``orbit_plotting.run_plots`` saves them. ``spacecraft_tables`` finds the
per-spacecraft CSV files of one scenario run.
"""
from __future__ import annotations

import math
import re
from pathlib import Path
from typing import Any

import numpy as np

from orbit_plotting import Table, line_color

M2KM = 1.0 / 1000.0
NANO2MIN = 1.0 / 60.0e9
NANO2SEC = 1.0e-9
RPM = 2.0 * math.pi / 60.0
COLOR_X = "dodgerblue"


def spacecraft_tables(generated_dir: Path, stem: str) -> list[Table]:
    """The ``<stem>_sat<i>.csv`` tables in spacecraft order."""
    pattern = re.compile(rf"^{re.escape(stem)}_sat(\d+)\.csv$")
    indexed = sorted(
        (int(match.group(1)), path)
        for path in generated_dir.glob(f"{stem}_sat*.csv")
        if (match := pattern.match(path.name))
    )
    return [Table(path) for _, path in indexed]


def count_columns(table: Table, prefix: str) -> int:
    """The number of ``<prefix>_<i>`` columns."""
    count = 0
    while f"{prefix}_{count}" in table.columns:
        count += 1
    return count


def columns(table: Table, prefix: str) -> np.ndarray:
    """The ``<prefix>_<i>`` columns as one array, a column per index."""
    return np.column_stack([table[f"{prefix}_{index}"] for index in range(count_columns(table, prefix))])


def _components(plt: Any, time: np.ndarray, data: np.ndarray, label: str, ylabel: str) -> Any:
    figure = plt.figure()
    axis = figure.gca()
    for index in range(3):
        axis.plot(time, data[:, index], color=line_color(index, 3, plt), label=label.format(index + 1))
    axis.legend(loc="lower right")
    axis.set_xlabel("Time [min]")
    axis.set_ylabel(ylabel)
    return figure


def plot_attitude(plt: Any, time: np.ndarray, sigma_bn: np.ndarray) -> Any:
    """Upstream ``plot_attitude``."""
    return _components(plt, time, sigma_bn, r"$\sigma_{}$", r"Attitude $\sigma_{B/N}$")


def plot_attitude_error(plt: Any, time: np.ndarray, sigma_br: np.ndarray) -> Any:
    """Upstream ``plot_attitude_error``."""
    return _components(plt, time, sigma_br, r"$\sigma_{}$", r"Attitude Tracking Error $\sigma_{B/R}$")


def plot_attitude_reference(plt: Any, time: np.ndarray, sigma_rn: np.ndarray) -> Any:
    """Upstream ``plot_attitude_reference``."""
    return _components(plt, time, sigma_rn, r"$\sigma_{}$", r"Attitude Reference $\sigma_{R/N}$")


def plot_rate(plt: Any, time: np.ndarray, omega_bn: np.ndarray) -> Any:
    """Upstream ``plot_rate``."""
    return _components(plt, time, omega_bn, r"$\omega_{{BN,{}}}$", "Angular Rate (rad/s)")


def plot_rate_error(plt: Any, time: np.ndarray, omega_br: np.ndarray) -> Any:
    """Upstream ``plot_rate_error``."""
    return _components(plt, time, omega_br, r"$\omega_{{BR,{}}}$", "Rate Tracking Error (rad/s)")


def plot_rate_reference(plt: Any, time: np.ndarray, omega_rn: np.ndarray) -> Any:
    """Upstream ``plot_rate_reference``."""
    return _components(plt, time, omega_rn, r"$\omega_{{RN,{}}}$", "Reference Rate (rad/s)")


def plot_rw_motor_torque(plt: Any, time: np.ndarray, requested: np.ndarray, applied: np.ndarray) -> Any:
    """Upstream ``plot_rw_motor_torque``: requested (dashed) and applied."""
    figure = plt.figure()
    axis = figure.gca()
    count = requested.shape[1]
    for index in range(count):
        color = line_color(index, count, plt)
        axis.plot(time, requested[:, index], "--", color=color, label=rf"$\hat u_{{s,{index + 1}}}$")
        axis.plot(time, applied[:, index], color=color, label=rf"$u_{{s,{index + 1}}}$")
    axis.legend(loc="lower right")
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("RW Motor Torque (Nm)")
    return figure


def plot_rw_speeds(plt: Any, time: np.ndarray, speeds: np.ndarray) -> Any:
    """Upstream ``plot_rw_speeds`` (RPM)."""
    figure = plt.figure()
    axis = figure.gca()
    count = speeds.shape[1]
    for index in range(count):
        axis.plot(time, speeds[:, index] / RPM, color=line_color(index, count, plt),
                  label=rf"$\Omega_{{{index + 1}}}$")
    axis.legend(loc="lower right")
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("RW Speed (RPM) ")
    return figure


def plot_power(plt: Any, time: np.ndarray, net: np.ndarray, supply: np.ndarray, sink: np.ndarray) -> Any:
    """Upstream ``plot_power``."""
    figure = plt.figure()
    axis = figure.gca()
    axis.plot(time, net, label="Net Power")
    axis.plot(time, supply, label="Panel Power")
    axis.plot(time, sink, label="Power Draw")
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("Power [W]")
    axis.grid(True)
    axis.legend(loc="lower right")
    return figure


def plot_fuel(plt: Any, time: np.ndarray, fuel: np.ndarray) -> Any:
    """Upstream ``plot_fuel``."""
    figure = plt.figure()
    axis = figure.gca()
    axis.plot(time, fuel)
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("Stored Fuel Mass [kg]")
    return figure


def plot_thrust_percentage(plt: Any, time: np.ndarray, factors: list[np.ndarray]) -> Any:
    """Upstream ``plot_thrust_percentage``."""
    figure = plt.figure()
    axis = figure.gca()
    for index, factor in enumerate(factors):
        axis.plot(time, factor, color=line_color(index, len(factors), plt),
                  label=rf"$F_{{thr,{index + 1}}}$")
    axis.legend(loc="lower right")
    axis.set_ylim([0, 1.1])
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("Thrust Percentage")
    return figure


def plot_orbits(plt: Any, positions: list[np.ndarray]) -> Any:
    """Upstream ``plot_orbits``: the inertial orbits in 3D, ±8000 km."""
    figure = plt.figure(figsize=(6, 5))
    axis = figure.add_subplot(projection="3d")
    for index, r in enumerate(positions):
        axis.plot(r[:, 0] * M2KM, r[:, 1] * M2KM, r[:, 2] * M2KM, label=f"Spacecraft {index}",
                  c=line_color(index, len(positions), plt))
    axis.set_xlim3d(-8000, 8000)
    axis.set_ylim3d(-8000, 8000)
    axis.set_zlim3d(-8000, 8000)
    axis.scatter(0, 0, 0, c=COLOR_X)
    axis.set_title("Spacecraft Inertial Orbits")
    axis.set_xlabel("x [km]")
    axis.set_ylabel("y [km]")
    axis.set_zlabel("z [km]")
    axis.legend(loc=2)
    return figure


def plot_relative_orbits(plt: Any, relative: list[np.ndarray]) -> Any:
    """Upstream ``plot_relative_orbits``: Hill-frame positions in 3D."""
    figure = plt.figure(figsize=(6, 5))
    axis = figure.add_subplot(projection="3d")
    for index, r in enumerate(relative):
        axis.plot(r[:, 0] * M2KM, r[:, 1] * M2KM, r[:, 2] * M2KM, label=f"Spacecraft {index}",
                  c=line_color(index, len(relative), plt))
    last = relative[-1]
    axis.set_box_aspect((np.ptp(last[:, 0]), np.ptp(last[:, 1]), np.ptp(last[:, 2])))
    axis.scatter(0, 0, 0, c=COLOR_X)
    axis.set_title("Spacecraft Relative Orbits in Hill Frame")
    axis.set_xlabel("$i_r$ [km]")
    axis.set_ylabel(r"$i_{\theta}$ [km]")
    axis.set_zlabel("$i_h$ [km]")
    axis.legend(loc=2)
    return figure


def plot_orbital_element_differences(plt: Any, time: np.ndarray, oed: np.ndarray) -> Any:
    """Upstream ``plot_orbital_element_differences``."""
    figure = plt.figure()
    axis = figure.gca()
    for index, label in enumerate(("da", "de", "di", "dOmega", "domega", "dM")):
        axis.plot(time, oed[:, index], label=label)
    axis.legend()
    axis.set_xlabel("time [orbit]")
    axis.set_ylabel("Orbital Element Difference")
    return figure


def orbit_figure(plt: Any, tables: list[Table]) -> Any:
    """``plot_orbits`` of every spacecraft's navigation position."""
    return plot_orbits(plt, [table.vectors("r_BN_N_m") for table in tables])
