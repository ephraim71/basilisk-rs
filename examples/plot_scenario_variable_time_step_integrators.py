#!/usr/bin/env python3
"""Upstream `scenarioVariableTimeStepIntegrators` figures: the orbits of the
tight-tolerance runs (`scenarioVariableTimeStepIntegrators<case>TightTol.csv`)
in Earth radii, and the position error histories of the accuracy comparison
(`scenarioVariableTimeStepIntegratorsAccuracy.csv`)."""

import numpy as np

from orbit_plotting import MU_EARTH, REQ_EARTH, Table, elements, existing, line_color, rv2elem, run_plots

CASES = ("rk4", "rkf45", "rkf78")
ACCURACY_LABELS = (
    "RKF45, default tolerances",
    "RKF45, tightened tolerances",
    "RKF78, default tolerances",
    "RKF78, tightened tolerances",
    "RKF78, default tolerances, 10 min task",
)


def build(generated_dir, plt):
    figures = {}
    tables = existing(generated_dir, [f"scenarioVariableTimeStepIntegrators{c}TightTol" for c in CASES])
    if tables:
        figure = plt.figure()
        axis = figure.gca()
        axis.axis([-50, 10, -20, 20])
        axis.set_aspect("equal")
        axis.add_artist(plt.Circle((0, 0), 1.0, color="#008800"))
        oe0 = None
        for stem, table in tables:
            positions, velocities = table.vectors("r_BN_N_m"), table.vectors("v_BN_N_m_per_s")
            if oe0 is None:
                oe0 = rv2elem(MU_EARTH, positions[0], velocities[0])
            case = stem.removeprefix("scenarioVariableTimeStepIntegrators").removesuffix("TightTol")
            radius, angle = [], []
            for oe in elements(MU_EARTH, positions, velocities):
                radius.append(oe["rmag"] / REQ_EARTH)
                angle.append(oe["f"] + oe["omega"] - oe0["omega"])
            radius, angle = np.array(radius), np.array(angle)
            axis.plot(radius * np.cos(angle), radius * np.sin(angle), linewidth=3.0, label=case,
                      color=line_color(CASES.index(case), len(CASES), plt))
        p = oe0["a"] * (1 - oe0["e"] ** 2)
        f = np.linspace(0, 2 * np.pi, 100)
        r = p / (1 + oe0["e"] * np.cos(f)) / REQ_EARTH
        axis.plot(r * np.cos(f), r * np.sin(f), "--", color="#555555")
        axis.set_xlabel("$i_e$ Cord. [DU]")
        axis.set_ylabel("$i_p$ Cord. [DU]")
        axis.legend(loc="lower right")
        axis.grid()
        figures["scenarioVariableTimeStepIntegrators"] = figure

    path = generated_dir / "scenarioVariableTimeStepIntegratorsAccuracy.csv"
    if path.is_file():
        table = Table(path)
        figure = plt.figure(figsize=(8, 5))
        axis = figure.gca()
        for index, label in enumerate(ACCURACY_LABELS):
            rows = table["case"] == index
            axis.semilogy(table["time_s"][rows] / 3600.0, np.maximum(table["position_error_m"][rows], 1e-6),
                          color=line_color(index, len(ACCURACY_LABELS), plt), label=label)
        axis.set_xlabel("Time [h]")
        axis.set_ylabel("Position Error [m]")
        axis.set_ylim(1e-6, 1e12)
        axis.legend(loc="upper left", ncol=2, fontsize="small")
        axis.grid()
        figures["scenarioVariableTimeStepIntegratorsAccuracy"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the variable time step integrator scenario.", build))
