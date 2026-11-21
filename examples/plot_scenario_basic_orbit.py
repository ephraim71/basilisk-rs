#!/usr/bin/env python3
"""Upstream `scenarioBasicOrbit` figures from the `scenario_basic_orbit` CSVs.

For every variant whose CSV exists (`scenarioBasicOrbit<orbit><sh><planet>.csv`):
figure 1, the inertial position components over the orbits; without spherical
harmonics, figure 2, the orbit in the initial perifocal frame, and figure 3, the
difference from the Kepler solution; with them, figure 2, the semi-major axis.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

from orbit_plotting import (
    DEFAULT_GENERATED_DIR,
    MU_EARTH,
    MU_MARS,
    NANO2SEC,
    REQ_EARTH,
    REQ_MARS,
    PlotError,
    Table,
    elements,
    perifocal_orbit,
    plot_components,
    rv2elem,
)

VARIANTS = ("LEO0Earth", "GTO0Earth", "GEO0Earth", "LEO1Earth", "LEO0Mars", "LEO1Mars")


def _kepler_positions(mu: float, oe: dict[str, float], times_s: np.ndarray) -> np.ndarray:
    e = oe["e"]
    e0 = 2 * math.atan2(math.sqrt(1 - e) * math.sin(oe["f"] / 2), math.sqrt(1 + e) * math.cos(oe["f"] / 2))
    m0 = e0 - e * math.sin(e0)
    n = math.sqrt(mu / oe["a"] ** 3)
    p = oe["a"] * (1 - e * e)
    ci, si = math.cos(oe["i"]), math.sin(oe["i"])
    cO, sO = math.cos(oe["Omega"]), math.sin(oe["Omega"])
    positions = []
    for t in times_s:
        m = m0 + n * t
        ecc = m
        for _ in range(200):
            step = (ecc - e * math.sin(ecc) - m) / (1 - e * math.cos(ecc))
            ecc -= step
            if abs(step) < 1e-13:
                break
        f = 2 * math.atan2(math.sqrt(1 + e) * math.sin(ecc / 2), math.sqrt(1 - e) * math.cos(ecc / 2))
        r = p / (1 + e * math.cos(f))
        theta = oe["omega"] + f
        positions.append(r * np.array([
            cO * math.cos(theta) - sO * math.sin(theta) * ci,
            sO * math.cos(theta) + cO * math.sin(theta) * ci,
            math.sin(theta) * si,
        ]))
    return np.array(positions)


def build(table: Table, variant: str, plt) -> dict:
    mars = variant.endswith("Mars")
    spherical_harmonics = variant[3] == "1"
    mu, radius = (MU_MARS, REQ_MARS) if mars else (MU_EARTH, REQ_EARTH)
    r = table.vectors("r_BN_N_m")
    v = table.vectors("v_BN_N_m_per_s")
    time_s = table.time_ns * NANO2SEC
    oe0 = rv2elem(mu, r[0], v[0])
    period = 2 * math.pi / math.sqrt(mu / oe0["a"] ** 3)
    orbits = time_s / period
    figures = {}

    figure = plt.figure()
    axis = figure.gca()
    axis.ticklabel_format(useOffset=False, style="plain")
    plot_components(axis, orbits, r / 1000.0, "$r_{{BN,{i}}}$", plt)
    axis.set_xlabel("Time [orbits]")
    axis.set_ylabel("Inertial Position [km]")
    figures[f"scenarioBasicOrbit1{variant}"] = figure

    if not spherical_harmonics:
        b = oe0["a"] * math.sqrt(1 - oe0["e"] ** 2)
        figure = plt.figure(figsize=tuple(np.array((1.0, b / oe0["a"])) * 4.75), dpi=100)
        perifocal_orbit(figure.gca(), mu, oe0, r, v, radius, "#884400" if mars else "#008800", plt)
        figures[f"scenarioBasicOrbit2{variant}"] = figure

        figure = plt.figure()
        axis = figure.gca()
        axis.ticklabel_format(useOffset=False, style="plain")
        difference = r - _kepler_positions(mu, oe0, time_s)
        plot_components(axis, orbits, difference, r"$\Delta r_{{BN,{i}}}$", plt)
        axis.set_xlabel("Time [orbits]")
        axis.set_ylabel("Trajectory Differences [m]")
        figures[f"scenarioBasicOrbit3{variant}"] = figure
    else:
        figure = plt.figure()
        axis = figure.gca()
        axis.ticklabel_format(useOffset=False, style="plain")
        axis.plot(orbits, [oe["a"] / 1000.0 for oe in elements(mu, r, v)], color="#aa0000")
        axis.set_xlabel("Time [orbits]")
        axis.set_ylabel("SMA [km]")
        figures[f"scenarioBasicOrbit2{variant}"] = figure
    return figures


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot the basic orbit scenario variants.")
    parser.add_argument("--generated-dir", type=Path, default=DEFAULT_GENERATED_DIR,
                        help="directory containing the generated scenario CSVs")
    parser.add_argument("--output-dir", type=Path,
                        help="directory for saved SVG figures (defaults to --generated-dir)")
    parser.add_argument("--save", action="store_true", help="save the SVG figures")
    parser.add_argument("--no-show", action="store_true", help="do not open the plot windows")
    arguments = parser.parse_args()

    import matplotlib

    if arguments.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output_dir = arguments.output_dir or arguments.generated_dir
    found = False
    try:
        for variant in VARIANTS:
            path = arguments.generated_dir / f"scenarioBasicOrbit{variant}.csv"
            if not path.is_file():
                continue
            found = True
            for name, figure in build(Table(path), variant, plt).items():
                if arguments.save or arguments.output_dir is not None:
                    output_dir.mkdir(parents=True, exist_ok=True)
                    output = output_dir / f"{name}.svg"
                    figure.savefig(output, transparent=True)
                    print(f"saved {output}")
    except PlotError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    if not found:
        print(f"error: no scenarioBasicOrbit CSVs in {arguments.generated_dir}", file=sys.stderr)
        return 1
    if not arguments.no_show:
        plt.show()
    plt.close("all")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
