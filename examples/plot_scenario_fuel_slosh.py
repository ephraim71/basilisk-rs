#!/usr/bin/env python3
"""Upstream `scenarioFuelSlosh` figures from `scenarioFuelSlosh<setup>.csv`:
the orbit, the relative variation of the orbital and rotational energy and
angular momentum, and (with damping) the particle displacements."""

import numpy as np

from orbit_plotting import MU_EARTH, NANO2SEC, REQ_EARTH, existing, rv2elem, run_plots

STEM = "scenarioFuelSlosh"


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, [f"{STEM}{setup}" for setup in (1, 2, 3)]):
        setup = stem[-1]
        seconds = table.time_ns * NANO2SEC
        positions = table.vectors("r_BN_N_m")
        velocities = table.vectors("v_BN_N_m_per_s")

        figure = plt.figure(figsize=(5, 5))
        axis = figure.gca()
        omega0 = rv2elem(MU_EARTH, positions[0], velocities[0])["omega"]
        radius, angle = [], []
        for r, v in zip(positions, velocities, strict=True):
            oe = rv2elem(MU_EARTH, r, v)
            radius.append(oe["rmag"])
            angle.append(oe["f"] + oe["omega"] - omega0)
        radius, angle = np.array(radius), np.array(angle)
        axis.add_artist(plt.Circle((0, 0), REQ_EARTH / 1000, color="#008800"))
        axis.plot(radius * np.cos(angle) / 1000, radius * np.sin(angle) / 1000)
        axis.set_xlim([-7000, 7000])
        axis.set_ylim([-7000, 7000])
        axis.set_xlabel("X (km)")
        axis.set_ylabel("Y (km)")
        figures[f"{STEM}Orbit{setup}"] = figure

        for name, column, ylabel in (
            ("OAM", "totOrbAngMomPntN_N_kg_m2_per_s", "Relative Orbital Angular Momentum Variation"),
            ("OE", "totOrbEnergy_J", "Relative Orbital Energy Variation"),
            ("RAM", "totRotAngMomPntC_N_kg_m2_per_s", "Relative Rotational Angular Momentum Variation"),
            ("RE", "totRotEnergy_J", "Relative Rotational Energy Variation"),
        ):
            figure = plt.figure(figsize=(5, 4))
            axis = figure.gca()
            values = table.vectors(column) if column.endswith("per_s") else table[column][:, None]
            for index in range(values.shape[1]):
                axis.plot(seconds, (values[:, index] - values[0, index]) / values[0, index])
            axis.set_xlabel("Time (s)")
            axis.set_ylabel(ylabel)
            figures[f"{STEM}{name}{setup}"] = figure

        if "damperRho1_m" in table.columns and setup == "3":
            figure = plt.figure(figsize=(5, 4))
            axis = figure.gca()
            for index in (1, 2, 3):
                axis.plot(seconds, table[f"damperRho{index}_m"])
            axis.legend(["Particle 1", "Particle 2", "Particle 3"], loc="lower right")
            axis.set_xlabel("Time (s)")
            axis.set_ylabel("Displacement (m)")
            figures[f"{STEM}ParticleMotion"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the fuel slosh scenario.", build))
