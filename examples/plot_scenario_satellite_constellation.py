#!/usr/bin/env python3
"""Upstream `scenarioSatelliteConstellation` figure from
`scenarioSatelliteConstellation.csv`: every satellite's ground track,
coloured by orbit plane, from its first (green) to its last (red) sample.

As upstream's `rv2latlon`, the Earth turns at `OMEGA_EARTH` from the
inertial frame at `t = 0`."""

import math

import numpy as np

from orbit_plotting import NANO2SEC, existing, line_color, run_plots

STEM = "scenarioSatelliteConstellation"
# Upstream `planetEphemeris.OMEGA_EARTH` [rad/s].
OMEGA_EARTH = 2 * math.pi / 23.9345 / 3600.0


def latitude_longitude(times_s, r_n):
    """Upstream `rv2latlon`: degrees, about the uniformly turning Earth."""
    theta = times_s * OMEGA_EARTH
    x = np.cos(theta) * r_n[:, 0] + np.sin(theta) * r_n[:, 1]
    y = -np.sin(theta) * r_n[:, 0] + np.cos(theta) * r_n[:, 1]
    latitude = np.degrees(np.arcsin(r_n[:, 2] / np.linalg.norm(r_n, axis=1)))
    longitude = np.degrees(np.arctan2(y, x))
    return latitude, longitude


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        count = sum(1 for name in table.columns if name.endswith("_r_BN_N_m_0"))
        times_s = table.time_ns * NANO2SEC
        positions = [table.vectors(f"sat{index}_r_BN_N_m") for index in range(count)]
        # Group the satellites by orbit plane (their angular momentum).
        normals = []
        planes = []
        for r_n in positions:
            normal = np.cross(r_n[0], r_n[1])
            normal /= np.linalg.norm(normal)
            for plane, known in enumerate(normals):
                if normal @ known > 1 - 1e-6:
                    planes.append(plane)
                    break
            else:
                normals.append(normal)
                planes.append(len(normals) - 1)

        figure = plt.figure()
        axis = figure.gca()
        for r_n, plane in zip(positions, planes):
            latitude, longitude = latitude_longitude(times_s, r_n)
            color = line_color(plane, max(len(normals), 3), plt)
            axis.scatter(longitude, latitude, s=5, color=[color])
            axis.scatter(longitude[0], latitude[0], s=20, c="#00A300")
            axis.scatter(longitude[-1], latitude[-1], s=20, c="#A30000")
        axis.set_xlim([-180, 180])
        axis.set_ylim([-90, 90])
        axis.set_xticks(range(-180, 181, 30))
        axis.set_yticks(range(-90, 91, 15))
        axis.set_xlabel("longitude (degrees)")
        axis.set_ylabel("lattitude (degrees)")
        figures[STEM] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(__doc__.splitlines()[0], build))
