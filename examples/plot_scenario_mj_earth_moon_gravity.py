#!/usr/bin/env python3
"""Upstream `scenarioMJEarthMoonGravity` figures from
`scenarioMJEarthMoonGravity_spice.csv` and `..._planetEphemeris.csv`: the
trajectory in the orbital plane with the 400 km bound, the Earth-Moon
geometry, and the osculating elements with the lunar perturbation."""

import numpy as np

from orbit_plotting import MU_EARTH, MU_MOON, NANO2HOUR, REQ_EARTH, elements, existing, run_plots

MINIMUM_SAFE_ALTITUDE_M = 400_000.0
STEMS = {"scenarioMJEarthMoonGravity_spice": "SPICE",
         "scenarioMJEarthMoonGravity_planetEphemeris": "PlanetEphemeris"}


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, list(STEMS)):
        label = STEMS[stem]
        hours = table.time_ns * NANO2HOUR
        r, v = table.vectors("hub_r_BN_N_m"), table.vectors("hub_v_BN_N_m_per_s")
        moon = table.vectors("moon_r_N_m") - table.vectors("earth_r_N_m")
        oe = elements(MU_EARTH, r, v)
        a = np.array([item["a"] for item in oe])
        e = np.array([item["e"] for item in oe])
        radius = np.array([item["rmag"] for item in oe])
        angle = np.array([item["omega"] + item["f"] for item in oe])
        to_spacecraft = moon - r
        third_body = np.linalg.norm(
            MU_MOON * to_spacecraft / np.linalg.norm(to_spacecraft, axis=1)[:, None] ** 3
            - MU_MOON * moon / np.linalg.norm(moon, axis=1)[:, None] ** 3, axis=1)

        figure, axis = plt.subplots(figsize=(6.5, 6.5))
        axis.add_patch(plt.Circle((0.0, 0.0), REQ_EARTH / 1000.0, color="#2a6fbb", alpha=0.8,
                                  label="Earth"))
        axis.add_patch(plt.Circle((0.0, 0.0), (REQ_EARTH + MINIMUM_SAFE_ALTITUDE_M) / 1000.0,
                                  color="#e9c46a", fill=False, linestyle="--", linewidth=2.0,
                                  label=r"Minimum radius ($R_E+400$ km)"))
        axis.plot(radius / 1000.0 * np.cos(angle), radius / 1000.0 * np.sin(angle), color="#bb3e03",
                  linewidth=1.0, label="MuJoCo hub")
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel(r"$r_x$ [km]")
        axis.set_ylabel(r"$r_y$ [km]")
        axis.set_title(f"MuJoCo trajectory in the orbital plane ({label})")
        axis.legend(loc="best")
        axis.grid(True, alpha=0.3)
        figures[stem + "_orbit"] = figure

        figure, axes = plt.subplots(1, 2, figsize=(10.0, 4.5))
        axes[0].plot(moon[:, 0] / 1.0e6, moon[:, 1] / 1.0e6, color="#6a4c93", linewidth=2.0,
                     label="Moon trajectory")
        axes[0].scatter(moon[0, 0] / 1.0e6, moon[0, 1] / 1.0e6, color="#2a9d8f", marker="o",
                        label="Start", zorder=3)
        axes[0].scatter(moon[-1, 0] / 1.0e6, moon[-1, 1] / 1.0e6, color="#e76f51", marker="x",
                        label="End", zorder=3)
        axes[0].scatter(0.0, 0.0, color="#2a6fbb", marker="o", label="Earth", zorder=3)
        axes[0].set_aspect("equal", adjustable="datalim")
        axes[0].set_xlabel(r"Earth-Moon $x$ [$10^3$ km]")
        axes[0].set_ylabel(r"Earth-Moon $y$ [$10^3$ km]")
        axes[0].set_title("Earth-centered geometry")
        axes[0].legend(loc="best")
        axes[0].grid(True, alpha=0.3)
        change = (moon - moon[0]) / 1000.0
        axes[1].plot(change[:, 0], change[:, 1], color="#6a4c93", linewidth=2.0)
        axes[1].scatter(change[0, 0], change[0, 1], color="#2a9d8f", marker="o", zorder=3)
        axes[1].scatter(change[-1, 0], change[-1, 1], color="#e76f51", marker="x", zorder=3)
        axes[1].set_aspect("equal", adjustable="datalim")
        axes[1].set_xlabel(r"$\Delta r_x$ [km]")
        axes[1].set_ylabel(r"$\Delta r_y$ [km]")
        axes[1].set_title("Lunar motion from initial position")
        axes[1].grid(True, alpha=0.3)
        figure.suptitle(f"Earth-Moon geometry over 6 hours ({label})")
        figures[stem + "_moonGeometry"] = figure

        figure, axes = plt.subplots(3, 1, figsize=(7.0, 7.0), sharex=True)
        axes[0].plot(hours, a - a[0], color="#005f73")
        axes[0].set_ylabel(r"$a-a_0$ [m]")
        axes[1].plot(hours, e, color="#ca6702")
        axes[1].set_ylabel("Eccentricity [-]")
        axes[2].plot(hours, third_body, color="#6a4c93")
        axes[2].set_ylabel(r"$|\mathbf{a}_{Moon,3B}|$ [m/s$^2$]")
        axes[2].set_xlabel("Time [h]")
        for axis in axes:
            axis.grid(True, alpha=0.3)
        figure.suptitle(f"Osculating orbit and lunar perturbation ({label})")
        figures[stem + "_diagnostics"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the MuJoCo Earth-Moon gravity scenario.", build))
