#!/usr/bin/env python3
"""Upstream `scenarioSmallBodyLandmarks` figures from
`scenarioSmallBodyLandmarks.csv`: the Eros-frame configuration at the first
and last sample, the visible landmark pixels at the end, the number of
visible landmarks, and the camera axes against the Hill axes.

The landmark positions are not in the CSV: they are redrawn from upstream's
Eros shape (`assets/gravity/eros007790.tab`) with upstream's
`landmark_distribution`, the same `np.random.seed(0)` draw the Rust example
reproduces."""

from pathlib import Path

import numpy as np
from orbit_plotting import existing, line_color, run_plots

SHAPE = Path(__file__).resolve().parent.parent / "assets/gravity/eros007790.tab"
N_LMK = 100


def mrp2c(sigma):
    """Upstream `RigidBodyKinematics.MRP2C`."""
    s1, s2, s3 = sigma
    q = np.dot(sigma, sigma)
    tilde = np.array([[0.0, -s3, s2], [s3, 0.0, -s1], [-s2, s1, 0.0]])
    return np.eye(3) + (8.0 * tilde @ tilde - 4.0 * (1.0 - q) * tilde) / (1.0 + q) ** 2


def load_shape():
    """Upstream `loadPolyFromFileToList` for the PDS3 `.tab` file: vertices in
    m, one-based faces."""
    vertices, faces = [], []
    with SHAPE.open() as handle:
        for line in handle:
            fields = line.split()
            if fields and fields[0] == "v":
                vertices.append([float(value) * 1e3 for value in fields[1:4]])
            elif fields and fields[0] == "f":
                faces.append([int(value) + 1 for value in fields[1:4]])
    return np.array(vertices), np.array(faces)


def landmark_distribution(vertices, faces, n_lmk):
    """Upstream `landmark_distribution`: the landmark face centers."""
    order = faces - 1
    centers = vertices[order].mean(axis=1)
    np.random.seed(0)
    index = np.random.choice(len(faces), n_lmk, replace=False)
    index.sort()
    return centers[index]


def plot_3d(plt, t, r, vertices, faces, positions, visible, dcm_pc):
    figure = plt.figure()
    axis = figure.add_subplot(1, 1, 1, projection="3d")
    axis.plot(r[0] / 1000, r[1] / 1000, r[2] / 1000, "k", marker="s", markersize=5)
    axis.plot_trisurf(vertices[:, 0] / 1000, vertices[:, 1] / 1000, vertices[:, 2] / 1000,
                      triangles=faces - 1, color=[105 / 255] * 3, zorder=0, alpha=0.1)
    on, off = visible == 1, visible != 1
    axis.plot(*(positions[on].T / 1000), "b", linestyle="", marker=".", markersize=5)
    axis.plot(*(positions[off].T / 1000), "r", linestyle="", marker=".", markersize=5, alpha=0.25)
    for column, color, alpha in [(0, "black", 0.25), (1, "black", 0.25), (2, "blue", 1.0)]:
        axis.quiver(*(r / 1000), *dcm_pc[:, column], length=10, normalize=True, color=color,
                    alpha=alpha)
    axis.set_xlabel("${}^{P}r_{x}$ [km]")
    axis.set_ylabel("${}^{P}r_{y}$ [km]")
    axis.set_zlabel("${}^{P}r_{z}$ [km]")
    axis.set_title("Asteroid rotating frame, t=" + str(int(t / 60)) + " min")
    return figure


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, ["scenarioSmallBodyLandmarks"]):
        t = table.time_ns * 1e-9
        dcm_cb = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
        visible = np.column_stack([table[f"isVisible_{j}"] for j in range(N_LMK)])
        pixels = np.stack([np.column_stack([table[f"pL_{j}_0"], table[f"pL_{j}_1"]])
                           for j in range(N_LMK)], axis=1)
        r_bp_p, dcm_pc, dcm_ph = [], [], []
        for i in range(len(t)):
            dcm_pn = mrp2c(table.vectors("sigma_PN")[i])
            dcm_bp = mrp2c(table.vectors("sigma_BN")[i]) @ dcm_pn.T
            r = dcm_pn @ (table.vectors("r_BN_N_m")[i] - table.vectors("r_PN_N_m")[i])
            v = dcm_pn @ (table.vectors("v_BN_N_m_per_s")[i] - table.vectors("v_PN_N_m_per_s")[i])
            ir = r / np.linalg.norm(r)
            ih = np.cross(r, v) / np.linalg.norm(np.cross(r, v))
            r_bp_p.append(r)
            dcm_pc.append((dcm_cb @ dcm_bp).T)
            dcm_ph.append(np.column_stack([ir, ih, np.cross(ih, ir)]))
        vertices, faces = load_shape()
        positions = landmark_distribution(vertices, faces, N_LMK)

        figures[stem + "1"] = plot_3d(plt, t[0], r_bp_p[0], vertices, faces, positions,
                                      visible[0], dcm_pc[0])
        figures[stem + "2"] = plot_3d(plt, t[-1], r_bp_p[-1], vertices, faces, positions,
                                      visible[-1], dcm_pc[-1])

        figure = plt.figure()
        axis = figure.gca()
        on = pixels[-1][visible[-1] == 1]
        axis.plot(on[:, 0], on[:, 1], linestyle="", marker="s", color="b", markersize=5)
        for index, pixel in enumerate(on):
            axis.annotate(str(index), (pixel[0], pixel[1]))
        axis.set_xlim([-1024, 1024])
        axis.set_ylim([-768, 768])
        axis.set_xlabel("$p_x$ [-]")
        axis.set_ylabel("$p_y$ [-]")
        figures[stem + "3"] = figure

        figure = plt.figure()
        axis = figure.gca()
        axis.plot(t / 3600, visible.sum(axis=1), linestyle="--", marker=".", markersize=8)
        axis.set_xlim([t[0] / 3600, t[-1] / 3600])
        axis.set_xlabel("Time [h]")
        axis.set_ylabel("Visible landmarks [-]")
        figures[stem + "4"] = figure

        figure = plt.figure()
        axis = figure.gca()
        data = np.array([[h[:, 0] @ c[:, 2], h[:, 1] @ c[:, 0], h[:, 2] @ c[:, 1]]
                         for h, c in zip(dcm_ph, dcm_pc)])
        labels = (r"$\hat\imath_r\cdot \hat c_3$", r"${\hat\imath}_{\theta}\cdot \hat c_1$",
                  r"$\hat\imath_h\cdot \hat c_2$")
        for index in range(3):
            axis.plot(t / 3600, data[:, index], color=line_color(index, 3, plt),
                      label=labels[index])
        axis.set_xlim([t[0] / 3600, t[-1] / 3600])
        axis.legend(loc="lower right")
        axis.set_xlabel("Time [h]")
        axis.set_ylabel("Orientation Illustration")
        figures[stem + "5"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the small-body landmarks scenario.", build))
