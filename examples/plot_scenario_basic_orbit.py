#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np


MU_EARTH_M3PS2 = 3.986_004_36e14
EARTH_EQUATORIAL_RADIUS_M = 6_378_136.6
SEMIMAJOR_AXIS_M = 7_000_000.0
ECCENTRICITY = 0.0001
INCLINATION_RAD = math.radians(33.3)
RAAN_RAD = math.radians(48.2)
ARGUMENT_OF_PERIAPSIS_RAD = math.radians(347.8)
INITIAL_TRUE_ANOMALY_RAD = math.radians(85.3)

CSV_FILENAME = "scenarioBasicOrbitLEO0Earth.csv"
FIGURE_STEMS = (
    "scenarioBasicOrbit1LEO0Earth",
    "scenarioBasicOrbit2LEO0Earth",
    "scenarioBasicOrbit3LEO0Earth",
)
CSV_COLUMNS = (
    "time_ns",
    "r_BN_N_m_0",
    "r_BN_N_m_1",
    "r_BN_N_m_2",
    "v_BN_N_m_per_s_0",
    "v_BN_N_m_per_s_1",
    "v_BN_N_m_per_s_2",
)


def _load_csv(path: Path) -> tuple[np.ndarray, np.ndarray]:
    try:
        with path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.reader(handle)
            header = next(reader)
            missing = [column for column in CSV_COLUMNS if column not in header]
            if missing:
                raise ValueError(f"missing columns: {', '.join(missing)}")
            indices = [header.index(column) for column in CSV_COLUMNS]
            rows = [
                [float(row[index]) for index in indices]
                for row in reader
                if row and any(value.strip() for value in row)
            ]
    except StopIteration as error:
        raise ValueError(f"{path} is empty") from error
    except OSError as error:
        raise ValueError(f"cannot read {path}: {error}") from error
    except (IndexError, ValueError) as error:
        raise ValueError(f"cannot parse {path}: {error}") from error

    if not rows:
        raise ValueError(f"{path} has no data rows")
    data = np.asarray(rows, dtype=float)
    if not np.isfinite(data).all():
        raise ValueError(f"{path} contains a non-finite value")
    if np.any(np.diff(data[:, 0]) <= 0.0):
        raise ValueError(f"{path} timestamps must be strictly increasing")
    return data[:, 0] * 1.0e-9, data[:, 1:4]


def _perifocal_basis() -> tuple[np.ndarray, np.ndarray]:
    sin_i, cos_i = math.sin(INCLINATION_RAD), math.cos(INCLINATION_RAD)
    sin_raan, cos_raan = math.sin(RAAN_RAD), math.cos(RAAN_RAD)
    sin_aop = math.sin(ARGUMENT_OF_PERIAPSIS_RAD)
    cos_aop = math.cos(ARGUMENT_OF_PERIAPSIS_RAD)
    periapsis = np.array(
        [
            cos_raan * cos_aop - sin_raan * sin_aop * cos_i,
            sin_raan * cos_aop + cos_raan * sin_aop * cos_i,
            sin_aop * sin_i,
        ]
    )
    transverse = np.array(
        [
            -cos_raan * sin_aop - sin_raan * cos_aop * cos_i,
            -sin_raan * sin_aop + cos_raan * cos_aop * cos_i,
            cos_aop * sin_i,
        ]
    )
    return periapsis, transverse


def _eccentric_anomaly_from_true(true_anomaly_rad: float) -> float:
    return 2.0 * math.atan2(
        math.sqrt(1.0 - ECCENTRICITY) * math.sin(true_anomaly_rad / 2.0),
        math.sqrt(1.0 + ECCENTRICITY) * math.cos(true_anomaly_rad / 2.0),
    )


def _true_anomaly_from_mean(mean_anomaly_rad: float) -> float:
    eccentric_anomaly_rad = mean_anomaly_rad
    for _ in range(200):
        correction = (
            eccentric_anomaly_rad
            - ECCENTRICITY * math.sin(eccentric_anomaly_rad)
            - mean_anomaly_rad
        ) / (1.0 - ECCENTRICITY * math.cos(eccentric_anomaly_rad))
        eccentric_anomaly_rad -= correction
        if abs(correction) <= 1.0e-13:
            return 2.0 * math.atan2(
                math.sqrt(1.0 + ECCENTRICITY)
                * math.sin(eccentric_anomaly_rad / 2.0),
                math.sqrt(1.0 - ECCENTRICITY)
                * math.cos(eccentric_anomaly_rad / 2.0),
            )
    raise ValueError("Kepler equation did not converge")


def _position_from_true_anomaly(
    true_anomaly_rad: float,
    periapsis: np.ndarray,
    transverse: np.ndarray,
) -> np.ndarray:
    semilatus_rectum_m = SEMIMAJOR_AXIS_M * (1.0 - ECCENTRICITY**2)
    radius_m = semilatus_rectum_m / (
        1.0 + ECCENTRICITY * math.cos(true_anomaly_rad)
    )
    return radius_m * (
        math.cos(true_anomaly_rad) * periapsis
        + math.sin(true_anomaly_rad) * transverse
    )


def _line_color(index: int, matplotlib: Any, plt: Any) -> Any:
    normalization = matplotlib.colors.Normalize(vmin=0, vmax=4)
    return plt.get_cmap("gist_earth")(normalization(index + 1))


def _configure_plot_style(matplotlib: Any) -> None:
    matplotlib.rc("figure", facecolor="white", figsize=(5.75, 2.5))
    matplotlib.rc("xtick", labelsize=9)
    matplotlib.rc("ytick", labelsize=9)
    matplotlib.rc("axes", labelsize=10)
    matplotlib.rc("legend", fontsize=9, loc="lower right")
    matplotlib.rc("figure", autolayout=True, max_open_warning=30)


def _build_figures(
    time_s: np.ndarray,
    position_m: np.ndarray,
    matplotlib: Any,
    plt: Any,
) -> tuple[Any, Any, Any]:
    mean_motion_radps = math.sqrt(MU_EARTH_M3PS2 / SEMIMAJOR_AXIS_M**3)
    orbit_period_s = 2.0 * math.pi / mean_motion_radps
    time_orbits = time_s / orbit_period_s
    periapsis, transverse = _perifocal_basis()

    figure_position = plt.figure(1)
    axis_position = figure_position.gca()
    axis_position.ticklabel_format(useOffset=False, style="plain")
    for index in range(3):
        axis_position.plot(
            time_orbits,
            position_m[:, index] / 1000.0,
            color=_line_color(index, matplotlib, plt),
            label=f"$r_{{BN,{index}}}$",
        )
    axis_position.legend(loc="lower right")
    axis_position.set_xlabel("Time [orbits]")
    axis_position.set_ylabel("Inertial Position [km]")

    semiminor_axis_m = SEMIMAJOR_AXIS_M * math.sqrt(1.0 - ECCENTRICITY**2)
    periapsis_radius_m = SEMIMAJOR_AXIS_M * (1.0 - ECCENTRICITY)
    apoapsis_radius_m = SEMIMAJOR_AXIS_M * (1.0 + ECCENTRICITY)
    figure_orbit = plt.figure(
        2,
        figsize=(4.75, semiminor_axis_m / SEMIMAJOR_AXIS_M * 4.75),
        dpi=100,
    )
    axis_orbit = figure_orbit.gca()
    axis_orbit.axis(
        np.array(
            [-apoapsis_radius_m, periapsis_radius_m, -semiminor_axis_m, semiminor_axis_m]
        )
        / 1000.0
        * 1.25
    )
    axis_orbit.add_artist(
        plt.Circle(
            (0.0, 0.0),
            EARTH_EQUATORIAL_RADIUS_M / 1000.0,
            color="#008800",
        )
    )
    axis_orbit.plot(
        position_m @ periapsis / 1000.0,
        position_m @ transverse / 1000.0,
        color="#aa0000",
        linewidth=3.0,
    )
    full_true_anomaly_rad = np.linspace(0.0, 2.0 * math.pi, 100)
    semilatus_rectum_m = SEMIMAJOR_AXIS_M * (1.0 - ECCENTRICITY**2)
    full_radius_m = semilatus_rectum_m / (
        1.0 + ECCENTRICITY * np.cos(full_true_anomaly_rad)
    )
    axis_orbit.plot(
        full_radius_m * np.cos(full_true_anomaly_rad) / 1000.0,
        full_radius_m * np.sin(full_true_anomaly_rad) / 1000.0,
        "--",
        color="#555555",
    )
    axis_orbit.set_xlabel("$i_e$ Cord. [km]")
    axis_orbit.set_ylabel("$i_p$ Cord. [km]")
    axis_orbit.grid()

    initial_eccentric_anomaly_rad = _eccentric_anomaly_from_true(
        INITIAL_TRUE_ANOMALY_RAD
    )
    initial_mean_anomaly_rad = initial_eccentric_anomaly_rad - ECCENTRICITY * math.sin(
        initial_eccentric_anomaly_rad
    )
    analytical_position_m = np.array(
        [
            _position_from_true_anomaly(
                _true_anomaly_from_mean(
                    initial_mean_anomaly_rad + mean_motion_radps * elapsed_s
                ),
                periapsis,
                transverse,
            )
            for elapsed_s in time_s
        ]
    )
    trajectory_difference_m = position_m - analytical_position_m

    figure_difference = plt.figure(3)
    axis_difference = figure_difference.gca()
    axis_difference.ticklabel_format(useOffset=False, style="plain")
    for index in range(3):
        axis_difference.plot(
            time_orbits,
            trajectory_difference_m[:, index],
            color=_line_color(index, matplotlib, plt),
            label=rf"$\Delta r_{{BN,{index}}}$",
        )
    axis_difference.legend(loc="lower right")
    axis_difference.set_xlabel("Time [orbits]")
    axis_difference.set_ylabel("Trajectory Differences [m]")
    return figure_position, figure_orbit, figure_difference


def _parser() -> argparse.ArgumentParser:
    default_generated_dir = Path(__file__).resolve().parent / "output/scenarios"
    parser = argparse.ArgumentParser(description="Plot the basic LEO orbit scenario.")
    parser.add_argument(
        "--generated-dir",
        type=Path,
        default=default_generated_dir,
        help="directory containing the generated scenario CSV",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="directory for saved SVG figures (defaults to --generated-dir)",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="save the three SVG figures",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="do not open the plot windows",
    )
    return parser


def main() -> int:
    arguments = _parser().parse_args()
    csv_path = arguments.generated_dir / CSV_FILENAME
    output_dir = arguments.output_dir or arguments.generated_dir
    try:
        time_s, position_m = _load_csv(csv_path)
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1

    import matplotlib

    if arguments.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    _configure_plot_style(matplotlib)
    plt.close("all")
    try:
        figures = _build_figures(time_s, position_m, matplotlib, plt)
        if arguments.save or arguments.output_dir is not None:
            output_dir.mkdir(parents=True, exist_ok=True)
            for stem, figure in zip(FIGURE_STEMS, figures, strict=True):
                output_path = output_dir / f"{stem}.svg"
                figure.savefig(output_path, transparent=True)
                print(f"saved {output_path}")
    except (OSError, ValueError) as error:
        print(f"error: cannot plot {csv_path}: {error}", file=sys.stderr)
        plt.close("all")
        return 1

    if not arguments.no_show:
        plt.show()
    plt.close("all")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
