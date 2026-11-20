"""Shared plumbing for the orbit-scenario plot scripts.

Each ``plot_scenario_<name>.py`` reads the CSV files its Rust example wrote to
``examples/output/scenarios`` and redraws upstream's figures with matplotlib,
saving them as PNG files named as upstream names them. This module holds what
those scripts share: CSV loading, the two-body element conversion upstream's
plots use (``orbitalMotion.rv2elem``), line colors and the command line.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Any, Callable

import numpy as np

DEFAULT_GENERATED_DIR = Path(__file__).resolve().parent / "output/scenarios"

# astroConstants (km based), in SI.
MU_EARTH = 398600.436e9
MU_MARS = 42828.314e9
MU_SUN = 132712440023.310e9
MU_MOON = 4902.799e9
MU_JUPITER = 126712767.881e9
REQ_EARTH = 6378.1366e3
REQ_MARS = 3396.19e3
REQ_SUN = 695000.0e3
REQ_MOON = 1737.4e3
REQ_JUPITER = 71492.0e3
AU = 149597870.693e3
NANO2SEC = 1e-9
NANO2MIN = 1.0 / 60.0e9
NANO2HOUR = 1.0 / 3600.0e9


class PlotError(RuntimeError):
    pass


class Table:
    """The columns of one scenario CSV, as float arrays."""

    def __init__(self, path: Path):
        try:
            with path.open("r", encoding="utf-8", newline="") as handle:
                reader = csv.reader(handle)
                header = next(reader)
                rows = [[float(value) for value in row] for row in reader if row]
        except (OSError, StopIteration, ValueError) as error:
            raise PlotError(f"cannot read {path}: {error}") from error
        if not rows:
            raise PlotError(f"{path} has no data rows")
        data = np.asarray(rows, dtype=float)
        self.path = path
        self.columns = {name.strip(): data[:, index] for index, name in enumerate(header)}

    def __getitem__(self, name: str) -> np.ndarray:
        try:
            return self.columns[name]
        except KeyError as error:
            raise PlotError(f"{self.path} has no column {name!r}") from error

    def vectors(self, prefix: str) -> np.ndarray:
        return np.column_stack([self[f"{prefix}_{index}"] for index in range(3)])

    @property
    def time_ns(self) -> np.ndarray:
        return self["time_ns"]


def rv2elem(mu: float, r: np.ndarray, v: np.ndarray) -> dict[str, float]:
    """Upstream ``orbitalMotion.rv2elem`` for one state (elliptic and
    hyperbolic orbits): ``a``, ``e``, ``i``, ``Omega``, ``omega``, ``f``,
    ``rmag``, ``rPeriap`` and ``rApoap``."""
    r = np.asarray(r, dtype=float)
    v = np.asarray(v, dtype=float)
    rmag = float(np.linalg.norm(r))
    h = np.cross(r, v)
    hmag = float(np.linalg.norm(h))
    p = hmag * hmag / mu
    node = np.cross([0.0, 0.0, 1.0], h)
    n_hat = node / np.linalg.norm(node) if np.linalg.norm(node) > 1e-12 else np.array([1.0, 0.0, 0.0])
    e_vec = r * (np.dot(v, v) / mu - 1.0 / rmag) - v * (np.dot(r, v) / mu)
    e = float(np.linalg.norm(e_vec))
    e_hat = e_vec / e if e > 1e-12 else n_hat
    alpha = 2.0 / rmag - np.dot(v, v) / mu
    a = 1.0 / alpha if abs(alpha) > 1e-12 else 0.0
    h_hat = h / hmag
    raan = math.atan2(np.cross([1.0, 0.0, 0.0], n_hat)[2], n_hat[0]) % (2 * math.pi)
    argp = math.atan2(np.dot(h_hat, np.cross(n_hat, e_hat)), np.dot(n_hat, e_hat)) % (2 * math.pi)
    f = math.atan2(np.dot(h_hat, np.cross(e_hat, r / rmag)), np.dot(e_hat, r / rmag)) % (2 * math.pi)
    return {
        "a": a,
        "e": e,
        "i": math.acos(max(-1.0, min(1.0, h[2] / hmag))),
        "Omega": raan,
        "omega": argp,
        "f": f,
        "rmag": rmag,
        "rPeriap": p / (1.0 + e),
        "rApoap": p / (1.0 - e) if abs(alpha) > 1e-12 else 0.0,
    }


def elements(mu: float, positions: np.ndarray, velocities: np.ndarray) -> list[dict[str, float]]:
    return [rv2elem(mu, r, v) for r, v in zip(positions, velocities, strict=True)]


def line_color(index: int, count: int, plt: Any) -> Any:
    """Upstream ``simHelpers.getLineColor``."""
    import matplotlib

    normalization = matplotlib.colors.Normalize(vmin=0, vmax=count + 1)
    return plt.get_cmap("gist_earth")(normalization(index + 1))


def plot_components(axis: Any, time: np.ndarray, values: np.ndarray, label: str, plt: Any,
                    legend: str = "lower right") -> None:
    """One line per column of ``values``; ``label`` is a format string of
    ``i``, the column index, such as ``"$r_{{BN,{i}}}$"``."""
    for index in range(values.shape[1]):
        axis.plot(
            time,
            values[:, index],
            color=line_color(index, values.shape[1], plt),
            label=label.format(i=index),
        )
    axis.legend(loc=legend)


def perifocal_orbit(axis: Any, mu: float, oe0: dict[str, float], positions: np.ndarray,
                    velocities: np.ndarray, planet_radius: float, planet_color: str,
                    plt: Any) -> None:
    """Upstream's orbit plot: the trajectory in the initial perifocal frame,
    the planet, and the initial osculating orbit dashed."""
    b = oe0["a"] * math.sqrt(1 - oe0["e"] ** 2)
    p = oe0["a"] * (1 - oe0["e"] ** 2)
    axis.axis(np.array([-oe0["rApoap"], oe0["rPeriap"], -b, b]) / 1000 * 1.25)
    axis.add_artist(plt.Circle((0, 0), planet_radius / 1000, color=planet_color))
    radius, angle = [], []
    for oe in elements(mu, positions, velocities):
        radius.append(oe["rmag"])
        angle.append(oe["f"] + oe["omega"] - oe0["omega"])
    radius, angle = np.array(radius), np.array(angle)
    axis.plot(radius * np.cos(angle) / 1000, radius * np.sin(angle) / 1000, color="#aa0000", linewidth=3.0)
    f = np.linspace(0, 2 * np.pi, 100)
    r = p / (1 + oe0["e"] * np.cos(f))
    axis.plot(r * np.cos(f) / 1000, r * np.sin(f) / 1000, "--", color="#555555")
    axis.set_xlabel("$i_e$ Cord. [km]")
    axis.set_ylabel("$i_p$ Cord. [km]")
    axis.grid()


def run_plots(description: str, build: Callable[[Path, Any], dict[str, Any]]) -> int:
    """Parse ``--generated-dir``, ``--output-dir`` and ``--show``, build the
    figures and save each as ``<name>.png``."""
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--generated-dir", type=Path, default=DEFAULT_GENERATED_DIR,
                        help=f"CSV directory (default: {DEFAULT_GENERATED_DIR})")
    parser.add_argument("--output-dir", type=Path, help="PNG directory (default: --generated-dir)")
    parser.add_argument("--show", action="store_true", help="show the plot windows")
    arguments = parser.parse_args()

    import matplotlib

    if not arguments.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    try:
        figures = build(arguments.generated_dir, plt)
    except PlotError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    if not figures:
        print(f"error: no CSV files found in {arguments.generated_dir}", file=sys.stderr)
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
    plt.close("all")
    return 0


def existing(generated_dir: Path, stems: list[str]) -> list[tuple[str, Table]]:
    """The tables of the ``stems`` whose CSV exists."""
    return [
        (stem, Table(generated_dir / f"{stem}.csv"))
        for stem in stems
        if (generated_dir / f"{stem}.csv").is_file()
    ]
