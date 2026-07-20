from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Any, Callable, Sequence

from scenario_csv_tools import (
    DEFAULT_GENERATED_DIR,
    DEFAULT_REFERENCE_ZIP,
    SCENARIOS,
    CsvTable,
    CsvToolError,
    load_scenario_pair,
)


COLORS = ("#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b")
EARTH_EQUATORIAL_RADIUS_M = 6_378_136.6


def _times_seconds(table: CsvTable) -> list[float]:
    return [value * 1.0e-9 for value in table.column("time_ns")]


def _values(table: CsvTable, column: str, scale: float = 1.0) -> list[float]:
    return [value * scale for value in table.column(column)]


def _norm(table: CsvTable, columns: Sequence[str], scale: float = 1.0) -> list[float]:
    vectors = [table.column(column) for column in columns]
    return [
        math.sqrt(sum(component[index] ** 2 for component in vectors)) * scale
        for index in range(len(table.rows))
    ]


def _pair_lines(
    axis: Any,
    generated: CsvTable,
    reference: CsvTable,
    columns: Sequence[str],
    labels: Sequence[str],
    *,
    scale: float = 1.0,
) -> None:
    for index, (column, label) in enumerate(zip(columns, labels, strict=True)):
        color = COLORS[index % len(COLORS)]
        axis.plot(
            _times_seconds(generated),
            _values(generated, column, scale),
            color=color,
            linestyle="-",
            label=f"{label} Rust",
        )
        axis.plot(
            _times_seconds(reference),
            _values(reference, column, scale),
            color=color,
            linestyle="--",
            label=f"{label} C++",
        )


def _derived_pair(
    axis: Any,
    generated: CsvTable,
    reference: CsvTable,
    generated_values: Sequence[float],
    reference_values: Sequence[float],
    label: str,
) -> None:
    axis.plot(
        _times_seconds(generated),
        generated_values,
        "-",
        color=COLORS[0],
        label=f"{label} Rust",
    )
    axis.plot(
        _times_seconds(reference),
        reference_values,
        "--",
        color=COLORS[0],
        label=f"{label} C++",
    )


def _component_errors(
    axis: Any,
    generated: CsvTable,
    reference: CsvTable,
    columns: Sequence[str],
    labels: Sequence[str],
    *,
    scale: float = 1.0,
) -> None:
    times = _times_seconds(generated)
    for index, (column, label) in enumerate(zip(columns, labels, strict=True)):
        actual = generated.column(column)
        expected = reference.column(column)
        errors = [
            (left - right) * scale for left, right in zip(actual, expected, strict=True)
        ]
        axis.plot(times, errors, color=COLORS[index % len(COLORS)], label=label)


def _finish_axis(axis: Any, title: str, ylabel: str | None = None) -> None:
    axis.set_title(title)
    if ylabel:
        axis.set_ylabel(ylabel)
    axis.grid(True, alpha=0.3)
    axis.legend(fontsize=8, ncol=2)


def _build_basic_orbit(plt: Any, generated: CsvTable, reference: CsvTable) -> Any:
    figure, axes = plt.subplots(2, 2, figsize=(14, 9))
    position = [f"r_BN_N_m_{index}" for index in range(3)]
    velocity = [f"v_BN_N_m_per_s_{index}" for index in range(3)]
    labels = ("x", "y", "z")

    _pair_lines(axes[0, 0], generated, reference, position, labels, scale=1.0e-3)
    _finish_axis(axes[0, 0], "Inertial Position", "km")
    _pair_lines(axes[0, 1], generated, reference, velocity, labels, scale=1.0e-3)
    _finish_axis(axes[0, 1], "Inertial Velocity", "km/s")

    axes[1, 0].plot(
        _values(generated, position[0], 1.0e-3),
        _values(generated, position[1], 1.0e-3),
        "-",
        label="Rust",
    )
    axes[1, 0].plot(
        _values(reference, position[0], 1.0e-3),
        _values(reference, position[1], 1.0e-3),
        "--",
        label="C++",
    )
    axes[1, 0].set_aspect("equal", adjustable="box")
    axes[1, 0].set_xlabel("x [km]")
    _finish_axis(axes[1, 0], "Inertial Orbit Projection", "y [km]")

    position_errors = [
        math.sqrt(
            sum(
                (generated.column(column)[row] - reference.column(column)[row]) ** 2
                for column in position
            )
        )
        for row in range(len(generated.rows))
    ]
    axes[1, 1].plot(
        _times_seconds(generated), position_errors, color=COLORS[3], label="|Δr|"
    )
    axes[1, 1].set_xlabel("time [s]")
    _finish_axis(axes[1, 1], "Rust − C++ Position Error", "m")
    return figure


def _build_css(plt: Any, generated: CsvTable, reference: CsvTable) -> Any:
    figure, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    columns = ("css1Signal", "css2Signal", "css3Signal")
    labels = ("CSS 1", "CSS 2", "CSS 3")
    _pair_lines(axes[0], generated, reference, columns, labels)
    _finish_axis(axes[0], "Coarse Sun Sensor Signals", "signal")
    _component_errors(axes[1], generated, reference, columns, labels)
    axes[1].set_xlabel("time [s]")
    _finish_axis(axes[1], "Rust − C++ Signal Error", "signal")
    return figure


def _build_drag_deorbit(plt: Any, generated: CsvTable, reference: CsvTable) -> Any:
    figure, axes = plt.subplots(2, 2, figsize=(14, 9))
    position = [f"r_BN_N_m_{index}" for index in range(3)]
    force = [f"forceExternal_B_N_{index}" for index in range(3)]
    reference_altitude = [
        radius - EARTH_EQUATORIAL_RADIUS_M * 1.0e-3
        for radius in _norm(reference, position, 1.0e-3)
    ]
    generated_altitude = [
        (radius - EARTH_EQUATORIAL_RADIUS_M) * 1.0e-3
        for radius in _norm(generated, position)
    ]
    _derived_pair(
        axes[0, 0],
        generated,
        reference,
        generated_altitude,
        reference_altitude,
        "altitude",
    )
    _finish_axis(axes[0, 0], "Deorbit Altitude", "km")

    _derived_pair(
        axes[0, 1],
        generated,
        reference,
        generated.column("neutralDensity_kg_per_m3"),
        reference.column("neutralDensity_kg_per_m3"),
        "density",
    )
    axes[0, 1].set_yscale("log")
    _finish_axis(axes[0, 1], "MSIS Neutral Density", "kg/m³")

    _derived_pair(
        axes[1, 0],
        generated,
        reference,
        _norm(generated, force),
        _norm(reference, force),
        "|F|",
    )
    axes[1, 0].set_xlabel("time [s]")
    _finish_axis(axes[1, 0], "Drag Force Magnitude", "N")

    axes[1, 1].plot(
        _values(generated, position[0], 1.0e-3),
        _values(generated, position[1], 1.0e-3),
        "-",
        label="Rust",
    )
    axes[1, 1].plot(
        _values(reference, position[0], 1.0e-3),
        _values(reference, position[1], 1.0e-3),
        "--",
        label="C++",
    )
    axes[1, 1].set_aspect("equal", adjustable="box")
    axes[1, 1].set_xlabel("x [km]")
    _finish_axis(axes[1, 1], "Orbit Projection", "y [km]")
    return figure


def _build_attitude_feedback_rw(
    plt: Any, generated: CsvTable, reference: CsvTable
) -> Any:
    figure, axes = plt.subplots(3, 2, figsize=(15, 13), sharex=True)
    xyz = ("x", "y", "z")
    sigma = [f"sigma_BR_{index}" for index in range(3)]
    omega = [f"omega_BR_B_rad_per_s_{index}" for index in range(3)]
    requested = [f"motorTorque_Nm_{index}" for index in range(3)]
    actual = [f"rwTorque_Nm_{index}" for index in range(3)]
    speeds = [f"wheelSpeeds_rad_per_s_{index}" for index in range(3)]
    voltage = [f"rwVoltage_V_{index}" for index in range(3)]

    _pair_lines(axes[0, 0], generated, reference, sigma, xyz)
    _finish_axis(axes[0, 0], "MRP Tracking Error", "MRP")
    _pair_lines(axes[0, 1], generated, reference, omega, xyz)
    _finish_axis(axes[0, 1], "Body-Rate Tracking Error", "rad/s")
    _pair_lines(axes[1, 0], generated, reference, requested, ("RW 1", "RW 2", "RW 3"))
    _finish_axis(axes[1, 0], "Requested Wheel Torque", "N·m")
    _pair_lines(axes[1, 1], generated, reference, actual, ("RW 1", "RW 2", "RW 3"))
    _finish_axis(axes[1, 1], "Applied Wheel Torque", "N·m")
    _pair_lines(axes[2, 0], generated, reference, speeds, ("RW 1", "RW 2", "RW 3"))
    axes[2, 0].set_xlabel("time [s]")
    _finish_axis(axes[2, 0], "Wheel Speeds", "rad/s")
    _pair_lines(axes[2, 1], generated, reference, voltage, ("RW 1", "RW 2", "RW 3"))
    axes[2, 1].set_xlabel("time [s]")
    _finish_axis(axes[2, 1], "Wheel Voltages", "V")
    return figure


def _build_mtb_momentum_management(
    plt: Any, generated: CsvTable, reference: CsvTable
) -> Any:
    figure, axes = plt.subplots(3, 2, figsize=(15, 13), sharex=True)
    sigma = [f"sigma_BR_{index}" for index in range(3)]
    omega = [f"omega_BR_B_rad_per_s_{index}" for index in range(3)]
    requested = [f"motorTorque_Nm_{index}" for index in range(4)]
    speeds = [f"wheelSpeeds_rad_per_s_{index}" for index in range(4)]
    dipoles = [f"mtbDipoleCmds_A_m2_{index}" for index in range(4)]
    xyz = ("x", "y", "z")
    wheels = ("RW 1", "RW 2", "RW 3", "RW 4")

    _pair_lines(axes[0, 0], generated, reference, sigma, xyz)
    _finish_axis(axes[0, 0], "MRP Tracking Error", "MRP")
    _pair_lines(axes[0, 1], generated, reference, omega, xyz)
    _finish_axis(axes[0, 1], "Body-Rate Tracking Error", "rad/s")
    _pair_lines(axes[1, 0], generated, reference, requested, wheels)
    _finish_axis(axes[1, 0], "Requested Wheel Torque", "N·m")
    _pair_lines(axes[1, 1], generated, reference, speeds, wheels)
    _finish_axis(axes[1, 1], "Wheel Speeds", "rad/s")

    field_columns = [f"magField_N_T_{index}" for index in range(3)]
    tam_columns = [f"tam_B_T_{index}" for index in range(3)]
    for index, (columns, label) in enumerate(
        ((field_columns, "|B inertial|"), (tam_columns, "|TAM body|"))
    ):
        color = COLORS[index]
        axes[2, 0].plot(
            _times_seconds(generated),
            _norm(generated, columns),
            "-",
            color=color,
            label=f"{label} Rust",
        )
        axes[2, 0].plot(
            _times_seconds(reference),
            _norm(reference, columns),
            "--",
            color=color,
            label=f"{label} C++",
        )
    axes[2, 0].set_xlabel("time [s]")
    _finish_axis(axes[2, 0], "Magnetic and TAM Field Magnitudes", "T")

    _pair_lines(
        axes[2, 1], generated, reference, dipoles, ("MTB 1", "MTB 2", "MTB 3", "MTB 4")
    )
    axes[2, 1].set_xlabel("time [s]")
    _finish_axis(axes[2, 1], "MTB Dipole Commands", "A·m²")
    return figure


BUILDERS: dict[str, Callable[[Any, CsvTable, CsvTable], Any]] = {
    "basic-orbit": _build_basic_orbit,
    "css": _build_css,
    "drag-deorbit": _build_drag_deorbit,
    "attitude-feedback-rw": _build_attitude_feedback_rw,
    "mtb-momentum-management": _build_mtb_momentum_management,
}


def run_plot(scenario_key: str) -> int:
    scenario = SCENARIOS[scenario_key]
    parser = argparse.ArgumentParser(
        description=f"Plot basilisk-rs and Basilisk C++ results for {scenario_key}."
    )
    parser.add_argument(
        "--generated-dir",
        type=Path,
        default=DEFAULT_GENERATED_DIR,
        help="generated CSV directory",
    )
    parser.add_argument(
        "--reference-zip",
        type=Path,
        default=DEFAULT_REFERENCE_ZIP,
        help="Basilisk C++ reference ZIP",
    )
    parser.add_argument("--output", type=Path, help="output PNG path")
    parser.add_argument(
        "--show", action="store_true", help="show the interactive plot window"
    )
    arguments = parser.parse_args()

    try:
        generated, reference = load_scenario_pair(
            scenario, arguments.generated_dir, arguments.reference_zip
        )
        if generated.headers != reference.headers:
            raise CsvToolError(
                "generated and reference CSV schemas differ; run the comparator for details"
            )
    except CsvToolError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1

    import matplotlib

    if not arguments.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    try:
        figure = BUILDERS[scenario_key](plt, generated, reference)
    except (CsvToolError, ValueError) as error:
        print(f"error: cannot plot {scenario_key}: {error}", file=sys.stderr)
        return 1

    figure.suptitle(f"{scenario_key}: basilisk-rs vs Basilisk C++", fontsize=14)
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))
    output = (
        arguments.output
        or arguments.generated_dir / f"{Path(scenario.filename).stem}.png"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=160)
    print(f"saved plot: {output}")
    if arguments.show:
        plt.show()
    else:
        plt.close(figure)
    return 0
