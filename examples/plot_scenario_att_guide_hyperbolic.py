#!/usr/bin/env python3
"""Figures of upstream scenarioAttGuideHyperbolic.py from the Rust CSV."""
from __future__ import annotations

import math
from typing import Any

import numpy as np

from orbit_plotting import MU_EARTH, REQ_EARTH, rv2elem
from plot_scenario_attitude_guidance import plot_attitude_error_norm
from scenario_plotting import (
    CsvTable,
    _plot_vector_history,
    _time_minutes,
    _vector_rows,
    run_variant_plot,
)

STEM = "scenarioAttGuideHyperbolic"
VARIANTS = ("0", "1")

# Upstream's hyperbola: a [m], e, omega [rad].
SEMI_MAJOR_AXIS_M = -150000.0 * 1000.0
ECCENTRICITY = 1.5
ARG_PERIAPSIS_RAD = math.radians(347.8)


def plot_orbit(plt: Any, table: CsvTable) -> Any:
    """Upstream plot_orbit: the flown arc and the hyperbola, in the
    perifocal frame."""
    figure = plt.figure(figsize=(4.75, 4.75), dpi=100)
    axis = figure.gca()
    axis.add_artist(plt.Circle((0, 0), REQ_EARTH / 1000, color="#008800"))
    radius, angle = [], []
    for position, velocity in zip(
        _vector_rows(table, "r_BN_N_m"), _vector_rows(table, "v_BN_N_m_per_s"), strict=True
    ):
        oe = rv2elem(MU_EARTH, np.array(position), np.array(velocity))
        radius.append(oe["rmag"])
        angle.append(oe["f"] + oe["omega"] - ARG_PERIAPSIS_RAD)
    radius, angle = np.array(radius), np.array(angle)
    axis.plot(radius * np.cos(angle) / 1000, radius * np.sin(angle) / 1000,
              color="#aa0000", linewidth=3.0, label="Simulated Flight")
    axis.axis(np.array([-1, 1, -1, 1]) * 1.25 * np.amax(radius) / 1000)
    p = SEMI_MAJOR_AXIS_M * (1 - ECCENTRICITY * ECCENTRICITY)
    half = 0.5 * (np.pi - 2 * np.arcsin(1 / ECCENTRICITY)) * 1.01
    f = np.linspace(np.pi - half, -np.pi + half, 100)
    r = p / (1 + ECCENTRICITY * np.cos(f))
    axis.plot(r * np.cos(f) / 1000, r * np.sin(f) / 1000, "--", color="#555555",
              label="Orbit Track")
    axis.set_xlabel("$i_e$ Cord. [km]")
    axis.set_ylabel("$i_p$ Cord. [km]")
    axis.legend(loc="lower left")
    axis.grid()
    return figure


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_minutes(table)

    attitude = plot_attitude_error_norm(plt, time, table)

    torque = plt.figure()
    _plot_vector_history(
        torque.gca(),
        time,
        table,
        "torqueRequestBody_Nm",
        lambda index: rf"$L_{{r,{index}}}$",
        "Control Torque $L_r$ [Nm]",
    )

    rate = plt.figure()
    _plot_vector_history(
        rate.gca(),
        time,
        table,
        "omega_BR_B_rad_per_s",
        lambda index: rf"$\omega_{{BR,{index}}}$",
        "Rate Tracking Error [rad/s] ",
    )

    return {
        f"{STEM}1{variant}": attitude,
        f"{STEM}2{variant}": torque,
        f"{STEM}3{variant}": rate,
        f"{STEM}4{variant}": plot_orbit(plt, table),
    }


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, VARIANTS, build))
