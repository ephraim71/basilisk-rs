#!/usr/bin/env python3
"""Upstream `scenarioTwoChargedSC` figures from `scenarioTwoChargedSC.csv` and
`scenarioTwoChargedSCSpheres.csv`: the separation, the relative trajectory in
the leader's Hill frame around the leader's spheres, and upstream
`SpherePlot.plotSpheres`, the charged spheres of both buses at the start with
their final charges, and its colour bar."""

import math

import matplotlib as mpl
import numpy as np

from orbit_plotting import MU_EARTH, NANO2SEC, Table, existing, rv2elem, run_plots

STEM = "scenarioTwoChargedSC"


def mrp_to_dcm(sigma):
    """Upstream `RigidBodyKinematics.MRP2C`: [BN] from sigma_BN."""
    s2 = sigma @ sigma
    tilde = np.array([[0, -sigma[2], sigma[1]], [sigma[2], 0, -sigma[0]], [-sigma[1], sigma[0], 0]])
    return np.eye(3) + (8 * tilde @ tilde - 4 * (1 - s2) * tilde) / (1 + s2) ** 2


def sphere_mesh():
    u = np.linspace(0, np.pi, 10)
    v = np.linspace(0, 2 * np.pi, 10)
    return (
        np.outer(np.sin(u), np.sin(v)),
        np.outer(np.sin(u), np.cos(v)),
        np.outer(np.cos(u), np.ones_like(v)),
    )


def charge_colors(leader, follower):
    """Upstream `plotSpheres`' colour map choice from the charge signs."""
    charges = np.concatenate([leader, follower])
    max_q, min_q = charges.max(), charges.min()
    if max_q < 0 and min_q < 0:
        return (mpl.colormaps["Greens"], mpl.colors.Normalize(vmin=min_q, vmax=0),
                lambda q: (q - 0) / (min_q - 0), lambda q: (q - min_q) / (0 - min_q))
    if max_q > 0 and min_q > 0:
        scale = lambda q: q / max_q  # noqa: E731
        return mpl.colormaps["Reds"], mpl.colors.Normalize(vmin=0, vmax=max_q), scale, scale
    if max_q > 0 and min_q < 0:
        scale = lambda q: (q - min_q) / (max_q - min_q)  # noqa: E731
        return mpl.colormaps["RdYlGn"], mpl.colors.Normalize(vmin=min_q, vmax=max_q), scale, scale
    grey = lambda q: 0.5  # noqa: E731
    return mpl.colormaps["Greys"], mpl.colors.Normalize(vmin=min_q, vmax=max_q), grey, grey


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        spheres_path = generated_dir / f"{STEM}Spheres.csv"
        if not spheres_path.is_file():
            continue
        spheres = Table(spheres_path)
        r_l = table.vectors("Leader_r_BN_N_m")
        v_l = table.vectors("Leader_v_BN_N_m_per_s")
        r_f = table.vectors("Follower_r_BN_N_m")
        sigma_l = table.vectors("Leader_sigma_BN")
        sigma_f = table.vectors("Follower_sigma_BN")
        period = 2 * math.pi * math.sqrt(rv2elem(MU_EARTH, r_l[0], v_l[0])["a"] ** 3 / MU_EARTH)
        relative = r_l - r_f

        figure = plt.figure()
        axis = figure.gca()
        axis.ticklabel_format(useOffset=False, style="plain")
        axis.plot(table.time_ns * NANO2SEC / period, np.linalg.norm(relative, axis=1))
        axis.set_xlabel("Time [orbits]")
        axis.set_ylabel("Separation [m]")
        axis.set_title("Total separation")
        figures[f"{STEM}1"] = figure

        # Relative position in the leader's Hill frame.
        hill = []
        for r, v, d in zip(r_l, v_l, relative):
            i_r = r / np.linalg.norm(r)
            i_h = np.cross(r, v) / np.linalg.norm(np.cross(r, v))
            hill.append(np.array([i_r, np.cross(i_h, i_r), i_h]) @ d)
        hill = np.array(hill)
        positions = spheres.vectors("r_SB_B_m")
        radii = spheres["radius_m"]
        x, y, z = sphere_mesh()
        figure = plt.figure(figsize=(5, 4))
        axis = figure.add_subplot(projection="3d")
        # Upstream rotates the leader's spheres with the follower's attitude.
        leader_n = (mrp_to_dcm(sigma_f[0]).T @ positions.T).T
        for center, radius in zip(leader_n, radii):
            axis.plot_surface(center[0] + radius * x, center[1] + radius * y,
                              center[2] + radius * z, color="b")
        axis.plot(hill[:, 0], hill[:, 1], hill[:, 2])
        axis.set_xlabel("Radial(m)")
        axis.set_ylabel("Along Track(m)")
        axis.set_zlabel("Orbit Normal (m)")
        figures[f"{STEM}2"] = figure

        leader_q = spheres["leaderCharge_C"]
        follower_q = spheres["followerCharge_C"]
        cmap, norm, leader_scale, follower_scale = charge_colors(leader_q, follower_q)
        figure = plt.figure()
        axis = figure.add_subplot(projection="3d")
        offset = r_f[0] - r_l[0]
        for dcm_nb, base, charges, scale in (
            (mrp_to_dcm(sigma_l[0]).T, np.zeros(3), leader_q, leader_scale),
            (mrp_to_dcm(sigma_f[0]).T, offset, follower_q, follower_scale),
        ):
            for center, radius, charge in zip((dcm_nb @ positions.T).T + base, radii, charges):
                axis.plot_surface(center[0] + radius * x, center[1] + radius * y,
                                  center[2] + radius * z, color=cmap(scale(charge)))
        limits = np.array([getattr(axis, f"get_{name}lim")() for name in "xyz"])
        axis.set_box_aspect(np.ptp(limits, axis=1))
        axis.set_xlabel("Radial")
        axis.set_ylabel("Along Track")
        axis.set_zlabel("Direction of Angular Momentum")
        axis.set_title("MSM Representation")
        figures[f"{STEM}3"] = figure

        figure, axis = plt.subplots(figsize=(6, 1))
        figure.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap), cax=axis,
                        orientation="horizontal", label="Sphere Charge (C)")
        figures[f"{STEM}4"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(__doc__.splitlines()[0], build))
