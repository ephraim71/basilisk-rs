#!/usr/bin/env python3
"""Upstream `scenarioFormationFlyingWithDrag` figures from
`scenarioFormationFlyingWithDrag*.csv`: the chief's inertial path, its drag,
the deputy's control force, the chief's elements, the element differences
and the deputy's Hill-frame path."""

import math

import numpy as np

from orbit_plotting import MU_EARTH, NANO2HOUR, PlotError, Table, run_plots

BLUE, RED = "#004488", "#BB5566"
STEMS = [
    "scenarioFormationFlyingWithDrag",
    "scenarioFormationFlyingWithDrag_noDrag",
    "scenarioFormationFlyingWithDrag_noControl",
    "scenarioFormationFlyingWithDrag_noIntegralControl",
    "scenarioFormationFlyingWithDrag_noDrag_noControl",
    "scenarioFormationFlyingWithDrag_noDrag_noIntegralControl",
    "scenarioFormationFlyingWithDrag_noControl_noIntegralControl",
    "scenarioFormationFlyingWithDrag_noDrag_noControl_noIntegralControl",
]
# Upstream `targetOeDiffPayload` and `kProp`.
TARGET = (0.0, 0.0005, 0.0, 0.0, 0.0, 0.0)
K_PROP = (1e5, 1e5, 0, 0, 10, 10)
LABELS = ("a [m]", "e [-]", "i [deg]", r"$\Omega$ [deg]", r"$\omega$ [deg]", r"$M$ [deg]")


def elements_with_mean_anomaly(r, v):
    """Upstream `rv2elemVector`: a, e and the angles in degrees, the last the
    mean anomaly."""
    rows = []
    for position, velocity in zip(r, v):
        h = np.cross(position, velocity)
        radius = np.linalg.norm(position)
        energy = 0.5 * velocity @ velocity - MU_EARTH / radius
        a = -MU_EARTH / (2 * energy)
        e_vec = np.cross(velocity, h) / MU_EARTH - position / radius
        e = np.linalg.norm(e_vec)
        i = math.acos(h[2] / np.linalg.norm(h))
        node = np.cross([0.0, 0.0, 1.0], h)
        raan = math.atan2(node[1], node[0]) % (2 * math.pi)
        omega = math.acos(np.clip(node @ e_vec / (np.linalg.norm(node) * e), -1, 1))
        if e_vec[2] < 0:
            omega = 2 * math.pi - omega
        f = math.acos(np.clip(e_vec @ position / (e * radius), -1, 1))
        if position @ velocity < 0:
            f = 2 * math.pi - f
        big_e = 2 * math.atan(math.sqrt((1 - e) / (1 + e)) * math.tan(f / 2))
        mean = (big_e - e * math.sin(big_e)) % (2 * math.pi)
        rows.append((a, e, math.degrees(i), math.degrees(raan), math.degrees(omega), math.degrees(mean)))
    return np.array(rows)


def hill(rc, vc, rd, vd):
    """Upstream `deputyToHillFrame`: the deputy's Hill-frame position."""
    r_hat = rc / np.linalg.norm(rc, axis=1, keepdims=True)
    h = np.cross(rc, vc)
    h_hat = h / np.linalg.norm(h, axis=1, keepdims=True)
    y_hat = np.cross(h_hat, r_hat)
    dcm = np.stack([r_hat, y_hat, h_hat], axis=1)
    return np.einsum("nij,nj->ni", dcm, rd - rc)


def gradient(axis, x, y):
    from matplotlib.collections import LineCollection

    points = np.column_stack([x, y]).reshape(-1, 1, 2)
    lines = LineCollection(np.concatenate([points[:-1], points[1:]], axis=1), cmap="viridis",
                           linewidth=2.0)
    lines.set_array(np.linspace(0.0, 1.0, len(x)))
    axis.add_collection(lines)
    axis.autoscale()


def build(generated_dir, plt):
    figures = {}
    for stem in STEMS:
        path = generated_dir / f"{stem}.csv"
        if not path.is_file():
            continue
        table = Table(path)
        hours = table.time_ns * NANO2HOUR
        rc, vc = table.vectors("chief_r_BN_N_m"), table.vectors("chief_v_BN_N_m_per_s")
        rd, vd = table.vectors("deputy_r_BN_N_m"), table.vectors("deputy_v_BN_N_m_per_s")

        figure, axis = plt.subplots(figsize=[5, 5])
        gradient(axis, rc[:, 0] / 1000.0, rc[:, 1] / 1000.0)
        axis.set_xlabel(r"${}^\mathcal{N} {x}$ [km]")
        axis.set_ylabel(r"${}^\mathcal{N} {y}$ [km]")
        axis.axis("equal")
        figures[stem + "_chiefInertial"] = figure

        if "chief_drag_force_S_N_0" in table.columns:
            figure, axis = plt.subplots(figsize=[8, 4])
            axis.plot(hours, np.linalg.norm(table.vectors("chief_drag_force_S_N"), axis=1), color=BLUE)
            axis.set_ylabel("Chief Drag [m/s$^2$]")
            axis.set_xlabel("Time [hr]")
            figures[stem + "_chiefDrag"] = figure
        controlled = "deputy_control_force_S_N_0" in table.columns
        if controlled:
            figure, axis = plt.subplots(figsize=[8, 4])
            axis.plot(hours, np.linalg.norm(table.vectors("deputy_control_force_S_N"), axis=1),
                      label="Feedback", color=RED)
            axis.legend(loc="upper right")
            axis.set_xlabel("Time [hr]")
            axis.set_ylabel("Control [m/s$^2$]")
            figures[stem + "_control"] = figure

        oe_chief = elements_with_mean_anomaly(rc, vc)
        oe_deputy = elements_with_mean_anomaly(rd, vd)
        oe_diff = oe_deputy.copy()
        oe_diff[:, :2] -= oe_chief[:, :2]
        oe_diff[:, 2:] = ((oe_deputy[:, 2:] - oe_chief[:, 2:] + 180.0) % 360.0) - 180.0
        for name, values, is_diff in (("Chief", oe_chief, False), ("Diff", oe_diff, True)):
            figure, axes = plt.subplots(nrows=2, ncols=3, sharex=True, figsize=[10, 6])
            for index, axis in enumerate(axes.flat):
                axis.plot(hours, values[:, index], color=BLUE)
                axis.set_ylabel((r"$\Delta$" if is_diff else "") + LABELS[index])
                axis.ticklabel_format(useOffset=False, axis="y")
                if is_diff and controlled and K_PROP[index] != 0:
                    target = TARGET[index] if index < 2 else math.degrees(TARGET[index])
                    axis.axhline(target, ls="--", color=RED)
            for axis in axes[-1]:
                axis.set_xlabel("Time [hr]")
            figures[stem + f"_oe{name}"] = figure

        position = hill(rc, vc, rd, vd)
        figure, axis = plt.subplots(figsize=[4, 4.8])
        gradient(axis, position[:, 0] / 1000.0, position[:, 1] / 1000.0)
        axis.set_xlabel(r"${}^\mathcal{H} {x}$ [km]")
        axis.set_ylabel(r"${}^\mathcal{H} {y}$ [km]")
        axis.axis("equal")
        figures[stem + "_hillXY"] = figure
    if not figures:
        raise PlotError(f"no scenarioFormationFlyingWithDrag CSV in {generated_dir}")
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the MuJoCo formation-flying-with-drag scenario.", build))
