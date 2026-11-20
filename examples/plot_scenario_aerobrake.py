#!/usr/bin/env python3
"""Upstream `scenarioAerobrake` figures from `scenarioAerobrake_<model>_<ctrl|free>.csv`,
every case found overlaid: the orbit trace, the altitude, the apoapsis
altitude, panel 1's deflection, the velocity pointing error and the density."""

import numpy as np

from orbit_plotting import existing, run_plots

EARTH_RADIUS = 6378.1363e3
MU = 0.3986004415e15
CASES = (
    ("branched_ctrl", "branched + ctrl", "C0", "-"),
    ("branched_free", "branched + free", "C1", "--"),
    ("legacy_ctrl", "legacy + ctrl", "C2", "-."),
    ("legacy_free", "legacy + free", "C3", ":"),
)


def mrp_to_dcm(sigma):
    s2 = float(np.dot(sigma, sigma))
    tilde = np.array([[0, -sigma[2], sigma[1]], [sigma[2], 0, -sigma[0]], [-sigma[1], sigma[0], 0]])
    return np.eye(3) + (8 * tilde @ tilde - 4 * (1 - s2) * tilde) / (1 + s2) ** 2


def build(generated_dir, plt):
    series = []
    for (key, label, color, style) in CASES:
        for _, table in existing(generated_dir, [f"scenarioAerobrake_{key}"]):
            r, v = table.vectors("r_BN_N_m"), table.vectors("v_BN_N_m_per_s")
            sigma = table.vectors("sigma_BN")
            radius = np.linalg.norm(r, axis=1)
            speed = np.linalg.norm(v, axis=1)
            pointing = []
            for s, vel in zip(sigma, v, strict=True):
                b2_n = mrp_to_dcm(s).T @ np.array([0.0, 1.0, 0.0])
                pointing.append(np.degrees(np.arccos(np.clip(np.dot(b2_n, vel / np.linalg.norm(vel)), -1, 1))))
            energy = 0.5 * speed**2 - MU / radius
            semi_major = -MU / (2.0 * energy)
            series.append({
                "label": label, "color": color, "style": style,
                "time": table.time_ns * 1e-9, "r": r, "altitude": radius - EARTH_RADIUS,
                "apoapsis": 2.0 * semi_major - radius.min() - EARTH_RADIUS,
                "theta1": table["panel1_theta_rad"], "pointing": np.array(pointing),
                "density": table["neutralDensity_kg_per_m3"],
            })
    if not series:
        return {}
    figures = {}

    def overlay(name, x, y, xlabel, ylabel, semilogy=False):
        figure = plt.figure()
        axis = figure.gca()
        for data in series:
            plot = axis.semilogy if semilogy else axis.plot
            plot(x(data), y(data), color=data["color"], linestyle=data["style"], label=data["label"])
        axis.set_xlabel(xlabel)
        axis.set_ylabel(ylabel)
        axis.legend()
        figures["scenarioAerobrake" + name] = figure
        return axis

    axis = overlay("OrbitTrace", lambda d: d["r"][:, 0] / 1e3, lambda d: d["r"][:, 1] / 1e3, "x [km]", "y [km]")
    axis.add_patch(plt.Circle((0, 0), EARTH_RADIUS / 1e3, color="0.7", alpha=0.4, zorder=0))
    axis.set_aspect("equal", adjustable="box")
    minutes = lambda d: d["time"] / 60.0  # noqa: E731
    overlay("Altitude", minutes, lambda d: d["altitude"] / 1e3, "time [min]", "altitude [km]")
    overlay("ApoapsisAltitude", minutes, lambda d: d["apoapsis"] / 1e3, "time [min]", "apoapsis altitude [km]")
    overlay("PanelDeflection", minutes, lambda d: np.degrees(d["theta1"]), "time [min]",
            r"panel 1 deflection $\theta_1$ [deg]")
    overlay("PointingError", minutes, lambda d: d["pointing"], "time [min]",
            r"Velocity pointing error $\angle(\hat{b}_2, \hat{v})$ [deg]")
    overlay("Density", minutes, lambda d: np.maximum(d["density"], np.finfo(float).tiny), "time [min]",
            r"atmospheric density [kg/m$^3$]", semilogy=True)
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the aerobraking scenario.", build))
