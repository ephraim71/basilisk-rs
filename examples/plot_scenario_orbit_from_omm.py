#!/usr/bin/env python3
"""Upstream `scenarioOrbitFromOmm` figures from `scenarioOrbitFromOmm<index>.csv`:
the imported orbits in 3D, and the altitude and speed histories."""

import numpy as np

from orbit_plotting import NANO2MIN, REQ_EARTH, Table, run_plots


def build(generated_dir, plt):
    tables = []
    index = 0
    while (generated_dir / f"scenarioOrbitFromOmm{index}.csv").is_file():
        tables.append(Table(generated_dir / f"scenarioOrbitFromOmm{index}.csv"))
        index += 1
    if not tables:
        return {}
    colors = [plt.get_cmap("tab10")(i % 10) for i in range(len(tables))]
    radius_km = REQ_EARTH / 1000

    orbit_figure = plt.figure(figsize=(8, 7))
    axis = orbit_figure.add_subplot(projection="3d")
    longitude = np.linspace(0.0, 2.0 * np.pi, 40)
    colatitude = np.linspace(0.0, np.pi, 20)
    axis.plot_surface(
        radius_km * np.outer(np.cos(longitude), np.sin(colatitude)),
        radius_km * np.outer(np.sin(longitude), np.sin(colatitude)),
        radius_km * np.outer(np.ones_like(longitude), np.cos(colatitude)),
        color="#abc7da", alpha=0.3, linewidth=0, shade=False,
    )
    limit = radius_km
    for index, (table, color) in enumerate(zip(tables, colors)):
        position_km = table.vectors("r_BN_N_m") / 1000
        axis.plot(*position_km.T, color=color, linewidth=2, label=f"satellite {index}")
        axis.scatter(*position_km[0], color=color, s=45, edgecolors="white", depthshade=False)
        limit = max(limit, np.max(np.linalg.norm(position_km, axis=1)))
    limit *= 1.08
    axis.set(xlim=(-limit, limit), ylim=(-limit, limit), zlim=(-limit, limit),
             xlabel="Inertial x [km]", ylabel="Inertial y [km]", zlabel="Inertial z [km]")
    axis.set_box_aspect((1, 1, 1))
    axis.view_init(elev=45, azim=30)
    axis.set_title("Inertial frame | dots mark the record epoch", fontsize=11)
    axis.legend(loc="upper right")

    history_figure, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for index, (table, color) in enumerate(zip(tables, colors)):
        minutes = table.time_ns * NANO2MIN
        axes[0].plot(minutes, np.linalg.norm(table.vectors("r_BN_N_m"), axis=1) / 1000 - radius_km,
                     color=color, linewidth=2, label=f"satellite {index}")
        axes[1].plot(minutes, np.linalg.norm(table.vectors("v_BN_N_m_per_s"), axis=1) / 1000,
                     color=color, linewidth=2)
    axes[0].set_ylabel("Spherical altitude [km]")
    axes[0].legend(loc="upper right")
    axes[1].set(ylabel="Inertial speed [km/s]", xlabel="Time since OMM epoch [min]")
    for each in axes:
        each.grid(alpha=0.2)
        each.margins(x=0)
    return {"scenarioOrbitFromOmmOrbits": orbit_figure, "scenarioOrbitFromOmmAltitudeSpeed": history_figure}


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the orbit-from-OMM scenario.", build))
