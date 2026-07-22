from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, Sequence


DEFAULT_GENERATED_DIR = Path("examples/output/scenarios")
EARTH_EQUATORIAL_RADIUS_M = 6_378_136.6
EARTH_MU_M3_PER_S2 = 3.986_004_36e14
RPM_RAD_PER_S = 2.0 * math.pi / 60.0
COLORS = ("#1f77b4", "#ff7f0e", "#2ca02c", "#d62728")


class PlotError(RuntimeError):
    pass


@dataclass(frozen=True)
class CsvTable:
    headers: tuple[str, ...]
    rows: tuple[dict[str, float], ...]

    def column(self, name: str) -> list[float]:
        if name not in self.headers:
            raise PlotError(f"CSV has no column named {name!r}")
        return [row[name] for row in self.rows]


@dataclass(frozen=True)
class PlotSpec:
    filename: str
    builder: Callable[[Any, CsvTable], dict[str, Any]]


def _load_csv(path: Path) -> CsvTable:
    try:
        with path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            if reader.fieldnames is None:
                raise PlotError(f"{path}: CSV is empty")
            headers = tuple(header.strip() for header in reader.fieldnames)
            if len(headers) != len(set(headers)):
                raise PlotError(f"{path}: CSV contains duplicate headers")

            rows: list[dict[str, float]] = []
            for line_number, raw_row in enumerate(reader, start=2):
                if None in raw_row:
                    raise PlotError(f"{path}:{line_number}: too many fields")
                try:
                    rows.append(
                        {
                            header: float(raw_row[header])
                            for header in headers
                            if raw_row[header] is not None
                        }
                    )
                except (TypeError, ValueError) as error:
                    raise PlotError(
                        f"{path}:{line_number}: invalid numeric value"
                    ) from error
    except OSError as error:
        raise PlotError(f"cannot read CSV {path}: {error}") from error

    if not rows:
        raise PlotError(f"{path}: CSV has no data rows")
    return CsvTable(headers=headers, rows=tuple(rows))


def _scaled(values: Iterable[float], scale: float) -> list[float]:
    return [value * scale for value in values]


def _time_seconds(table: CsvTable) -> list[float]:
    return _scaled(table.column("time_ns"), 1.0e-9)


def _time_minutes(table: CsvTable) -> list[float]:
    return _scaled(table.column("time_ns"), 1.0 / 60.0e9)


def _time_hours(table: CsvTable) -> list[float]:
    return _scaled(table.column("time_ns"), 1.0 / 3_600.0e9)


def _vector_columns(table: CsvTable, prefix: str, count: int) -> list[list[float]]:
    return [table.column(f"{prefix}_{index}") for index in range(count)]


def _vector_rows(table: CsvTable, prefix: str, count: int = 3) -> list[tuple[float, ...]]:
    columns = _vector_columns(table, prefix, count)
    return [tuple(column[row] for column in columns) for row in range(len(table.rows))]


def _norm(vector: Sequence[float]) -> float:
    return math.sqrt(sum(component * component for component in vector))


def _dot(left: Sequence[float], right: Sequence[float]) -> float:
    return sum(a * b for a, b in zip(left, right, strict=True))


def _cross(left: Sequence[float], right: Sequence[float]) -> tuple[float, float, float]:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _unit(vector: Sequence[float]) -> tuple[float, ...]:
    magnitude = _norm(vector)
    if magnitude == 0.0:
        raise PlotError("cannot normalize a zero vector")
    return tuple(component / magnitude for component in vector)


def _line_color(index: int) -> str:
    return COLORS[index % len(COLORS)]


def _build_css(plt: Any, table: CsvTable) -> dict[str, Any]:
    figure = plt.figure(figsize=(6.4, 4.8))
    axis = figure.gca()
    time = _time_seconds(table)
    for index in range(3):
        axis.plot(
            time,
            table.column(f"css{index + 1}Signal"),
            color=_line_color(index),
            label=rf"CSS$_{{{index + 1}}}$",
        )
    axis.legend(loc="lower right")
    axis.set_xlabel("Time [sec]")
    axis.set_ylabel("CSS Signals ")
    return {"scenarioCSS0010": figure}


def _drag_perifocal_basis(
    position: Sequence[float], velocity: Sequence[float]
) -> tuple[tuple[float, ...], tuple[float, ...], float, float]:
    radius = _norm(position)
    speed_squared = _dot(velocity, velocity)
    angular_momentum = _cross(position, velocity)
    angular_momentum_unit = _unit(angular_momentum)
    eccentricity_vector = tuple(
        component / EARTH_MU_M3_PER_S2 - position_component / radius
        for component, position_component in zip(
            _cross(velocity, angular_momentum), position, strict=True
        )
    )
    eccentricity = _norm(eccentricity_vector)
    periapsis_unit = _unit(eccentricity_vector)
    transverse_unit = _unit(_cross(angular_momentum_unit, periapsis_unit))
    semimajor_axis = 1.0 / (2.0 / radius - speed_squared / EARTH_MU_M3_PER_S2)
    return periapsis_unit, transverse_unit, semimajor_axis, eccentricity


def _build_drag_deorbit(plt: Any, table: CsvTable) -> dict[str, Any]:
    position = _vector_rows(table, "r_BN_N_m")
    velocity = _vector_rows(table, "v_BN_N_m_per_s")
    drag_force = _vector_rows(table, "forceExternal_B_N")
    density = table.column("neutralDensity_kg_per_m3")
    time_hours = _time_hours(table)

    periapsis, transverse, semimajor_axis, eccentricity = _drag_perifocal_basis(
        position[0], velocity[0]
    )
    orbit_x = [_dot(vector, periapsis) / 1_000.0 for vector in position]
    orbit_y = [_dot(vector, transverse) / 1_000.0 for vector in position]
    radii_km = [_norm(vector) / 1_000.0 for vector in position]
    altitude_km = [
        radius - EARTH_EQUATORIAL_RADIUS_M / 1_000.0 for radius in radii_km
    ]

    figures: dict[str, Any] = {}

    orbit_figure = plt.figure(
        figsize=(4.75, 4.75 * math.sqrt(1.0 - eccentricity * eccentricity)),
        dpi=100,
    )
    orbit_axis = orbit_figure.gca()
    semiminor_axis = semimajor_axis * math.sqrt(1.0 - eccentricity * eccentricity)
    apoapsis = semimajor_axis * (1.0 + eccentricity)
    periapsis_radius = semimajor_axis * (1.0 - eccentricity)
    orbit_axis.axis(
        [
            -apoapsis / 1_000.0 * 1.25,
            periapsis_radius / 1_000.0 * 1.25,
            -semiminor_axis / 1_000.0 * 1.25,
            semiminor_axis / 1_000.0 * 1.25,
        ]
    )
    orbit_axis.set_aspect("equal", adjustable="box")
    orbit_axis.add_artist(
        plt.Circle(
            (0.0, 0.0), EARTH_EQUATORIAL_RADIUS_M / 1_000.0, color="#008800"
        )
    )
    orbit_axis.plot(orbit_x, orbit_y, color="#aa0000", linewidth=1.0)
    orbit_axis.set_xlabel(r"$i_e$ Cord. [km]")
    orbit_axis.set_ylabel(r"$i_p$ Cord. [km]")
    figures["scenarioDragDeorbitmsis1"] = orbit_figure

    altitude_figure = plt.figure()
    altitude_axis = altitude_figure.gca()
    altitude_axis.ticklabel_format(useOffset=False, style="plain")
    altitude_axis.plot(time_hours, altitude_km)
    altitude_axis.set_xlabel(r"$t$ [h]")
    altitude_axis.set_ylabel("Alt. [km]")
    figures["scenarioDragDeorbitmsis2"] = altitude_figure

    density_figure = plt.figure()
    density_axis = density_figure.gca()
    density_axis.semilogy(altitude_km, density)
    density_axis.set_xlabel("Alt. [km]")
    density_axis.set_ylabel(r"$\rho$ [kg/m$^2$]")
    figures["scenarioDragDeorbitmsis3"] = density_figure

    force_figure = plt.figure()
    force_axis = force_figure.gca()
    force_axis.semilogy(time_hours, [_norm(vector) for vector in drag_force])
    force_axis.set_xlabel(r"$t$ [hr]")
    force_axis.set_ylabel(r"$|F_drag|$ [N]")
    figures["scenarioDragDeorbitmsis4"] = force_figure
    return figures


def _plot_attitude_error(axis: Any, time: Sequence[float], table: CsvTable) -> None:
    for index, values in enumerate(_vector_columns(table, "sigma_BR", 3)):
        axis.plot(
            time,
            values,
            color=_line_color(index),
            label=rf"$\sigma_{index}$",
        )
    axis.legend(loc="lower right")
    axis.set_xlabel("Time [min]")
    axis.set_ylabel(r"Attitude Error $\sigma_{B/R}$")


def _plot_motor_torque(
    axis: Any,
    time: Sequence[float],
    table: CsvTable,
    wheel_count: int,
    *,
    grid: bool,
) -> None:
    requested = _vector_columns(table, "motorTorque_Nm", wheel_count)
    actual = _vector_columns(table, "rwTorque_Nm", wheel_count)
    for index, (requested_values, actual_values) in enumerate(
        zip(requested, actual, strict=True)
    ):
        color = _line_color(index)
        axis.plot(
            time,
            requested_values,
            "--",
            color=color,
            label=rf"$\hat u_{{s,{index}}}$",
        )
        axis.plot(
            time,
            actual_values,
            color=color,
            label=rf"$u_{{s,{index}}}$",
        )
    axis.legend(loc="lower right")
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("RW Motor Torque [Nm]" if grid else "RW Motor Torque (Nm)")
    axis.grid(grid)


def _plot_wheel_speeds(
    axis: Any,
    time: Sequence[float],
    table: CsvTable,
    wheel_count: int,
    *,
    grid: bool,
) -> None:
    for index, values in enumerate(
        _vector_columns(table, "wheelSpeeds_rad_per_s", wheel_count)
    ):
        axis.plot(
            time,
            _scaled(values, 1.0 / RPM_RAD_PER_S),
            color=_line_color(index),
            label=rf"$\Omega_{index}$",
        )
    axis.legend(loc="lower right")
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("RW Speed [RPM] " if grid else "RW Speed (RPM) ")
    axis.grid(grid)


def _build_attitude_feedback_rw(plt: Any, table: CsvTable) -> dict[str, Any]:
    time = _time_minutes(table)

    attitude_figure = plt.figure()
    _plot_attitude_error(attitude_figure.gca(), time, table)

    torque_figure = plt.figure()
    _plot_motor_torque(torque_figure.gca(), time, table, 3, grid=False)

    speed_figure = plt.figure()
    _plot_wheel_speeds(speed_figure.gca(), time, table, 3, grid=False)

    voltage_figure = plt.figure()
    voltage_axis = voltage_figure.gca()
    for index, values in enumerate(_vector_columns(table, "rwVoltage_V", 3)):
        voltage_axis.plot(
            time,
            values,
            color=_line_color(index),
            label=rf"$V_{index}$",
        )
    voltage_axis.legend(loc="lower right")
    voltage_axis.set_xlabel("Time [min]")
    voltage_axis.set_ylabel("RW Voltage (V)")

    return {
        "scenarioAttitudeFeedbackRW101": attitude_figure,
        "scenarioAttitudeFeedbackRW201": torque_figure,
        "scenarioAttitudeFeedbackRW301": speed_figure,
        "scenarioAttitudeFeedbackRW401": voltage_figure,
    }


def _plot_three_axis_field(
    axis: Any,
    time: Sequence[float],
    table: CsvTable,
    prefix: str,
    label_prefix: str,
) -> None:
    for index, values in enumerate(_vector_columns(table, prefix, 3)):
        axis.plot(
            time,
            _scaled(values, 1.0e9),
            color=_line_color(index),
            label=rf"${label_prefix}_{{{index}}}$",
        )
    axis.legend(loc="lower right")
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("Magnetic Field [nT]")
    axis.grid(True)


def _build_mtb_momentum_management(plt: Any, table: CsvTable) -> dict[str, Any]:
    time = _time_minutes(table)

    attitude_figure = plt.figure()
    attitude_axis = attitude_figure.gca()
    _plot_attitude_error(attitude_axis, time, table)
    attitude_axis.grid(True)

    torque_figure = plt.figure()
    _plot_motor_torque(torque_figure.gca(), time, table, 4, grid=True)

    speed_figure = plt.figure()
    _plot_wheel_speeds(speed_figure.gca(), time, table, 4, grid=True)

    magnetic_figure = plt.figure()
    _plot_three_axis_field(
        magnetic_figure.gca(), time, table, "magField_N_T", r"B\_N"
    )

    tam_sensor_figure = plt.figure()
    _plot_three_axis_field(
        tam_sensor_figure.gca(), time, table, "tam_S_T", r"TAM\_S"
    )

    tam_body_figure = plt.figure()
    _plot_three_axis_field(
        tam_body_figure.gca(), time, table, "tam_B_T", r"TAM\_B"
    )

    dipole_figure = plt.figure()
    dipole_axis = dipole_figure.gca()
    for index, values in enumerate(
        _vector_columns(table, "mtbDipoleCmds_A_m2", 4)
    ):
        dipole_axis.plot(
            time,
            values,
            color=_line_color(index),
            label=rf"$MTB\_T_{{{index}}}$",
        )
    dipole_axis.legend(loc="lower right")
    dipole_axis.set_xlabel("Time [min]")
    dipole_axis.set_ylabel("Torque Rod Dipoles [A-m2]")
    dipole_axis.grid(True)

    return {
        "scenarioMtbMomentumManagement1": attitude_figure,
        "scenarioMtbMomentumManagement2": torque_figure,
        "scenarioMtbMomentumManagement3": speed_figure,
        "scenarioMtbMomentumManagement4": magnetic_figure,
        "scenarioMtbMomentumManagement5": tam_sensor_figure,
        "scenarioMtbMomentumManagement6": tam_body_figure,
        "scenarioMtbMomentumManagement7": dipole_figure,
    }


PLOTS: dict[str, PlotSpec] = {
    "css": PlotSpec("scenarioCSS0010.csv", _build_css),
    "drag-deorbit": PlotSpec("scenarioDragDeorbitmsis0.csv", _build_drag_deorbit),
    "attitude-feedback-rw": PlotSpec(
        "scenarioAttitudeFeedbackRW01.csv", _build_attitude_feedback_rw
    ),
    "mtb-momentum-management": PlotSpec(
        "scenarioMtbMomentumManagement.csv", _build_mtb_momentum_management
    ),
}


def run_plot(scenario_key: str) -> int:
    try:
        spec = PLOTS[scenario_key]
    except KeyError:
        print(f"error: unknown plot scenario {scenario_key!r}", file=sys.stderr)
        return 2

    parser = argparse.ArgumentParser(description=f"Plot {scenario_key} results.")
    parser.add_argument(
        "--generated-dir",
        type=Path,
        default=DEFAULT_GENERATED_DIR,
        help=f"CSV directory (default: {DEFAULT_GENERATED_DIR})",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="PNG directory (default: generated CSV directory)",
    )
    parser.add_argument(
        "--show", action="store_true", help="show the interactive plot windows"
    )
    arguments = parser.parse_args()

    try:
        table = _load_csv(arguments.generated_dir / spec.filename)
    except PlotError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1

    import matplotlib

    if not arguments.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    try:
        figures = spec.builder(plt, table)
    except (PlotError, ValueError) as error:
        print(f"error: cannot plot {scenario_key}: {error}", file=sys.stderr)
        return 1

    output_dir = arguments.output_dir or arguments.generated_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    for name, figure in figures.items():
        output = output_dir / f"{name}.png"
        figure.tight_layout()
        figure.savefig(output, dpi=160)
        print(f"saved plot: {output}")

    if arguments.show:
        plt.show()
    else:
        for figure in figures.values():
            plt.close(figure)
    return 0
