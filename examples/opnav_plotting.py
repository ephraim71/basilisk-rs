"""Upstream ``examples/OpNavScenarios/plottingOpNav/OpNav_Plotting.py`` for
the Rust CSVs.

Each function draws upstream's figure(s) from numpy arrays read from a CSV
written by an OpNav example (``scenario_op_nav_*``, ``scenario_cnn_att_od``,
``scenario_fault_det_op_nav``, ``scenario_limb_att_od``,
``scenario_op_nav_att_od_mc``) and returns them by name; the
``plot_scenario_*.py`` scripts save them as ``<scenario>_<name>.png``, the
names upstream's commented-out ``savefig`` calls use. The truth upstream's
``pull_outputs`` computes (the planet's true circle in the image, the
camera-frame position) is :func:`camera_truth`.

Upstream time arrays carry a leading time column; here the time is a
separate argument in nanoseconds or minutes.
"""
from __future__ import annotations

from typing import Any

import numpy as np

from orbit_plotting import NANO2MIN, Table

M2KM = 1.0 / 1000.0
NS2MIN = NANO2MIN
# Upstream `Rmars` of the scenarios' `pull_outputs` [m].
R_MARS = 3396.19e3
# Upstream `BSK_OpNavDynamics` camera: resolution [pixels], field of view
# [rad], sensor size and focal length [m], `cameraMRP_CB`.
CAMERA_RESOLUTION = np.array([512, 512])
CAMERA_FOV = np.deg2rad(55.0)
CAMERA_SIZE = np.array([10e-3, 10e-3])
CAMERA_FOCAL = CAMERA_SIZE[1] / 2.0 / np.tan(CAMERA_FOV / 2.0)
SIGMA_CB = np.zeros(3)


def colors(count: int = 10) -> list[Any]:
    """Upstream's ``colorList``: ``count`` colours of the ``inferno`` map."""
    import matplotlib

    inferno = matplotlib.colormaps["inferno"].colors
    step = len(inferno) / count
    return [inferno[int(i * step)] for i in range(count)]


def mrp_to_dcm(sigma: np.ndarray) -> np.ndarray:
    """Upstream ``RigidBodyKinematics.MRP2C``."""
    q1, q2, q3 = sigma
    d1 = q1 * q1 + q2 * q2 + q3 * q3
    s = 1.0 - d1
    d = (1.0 + d1) * (1.0 + d1)
    return np.array([
        [4 * (2 * q1 * q1 - d1) + s * s, 8 * q1 * q2 + 4 * q3 * s, 8 * q1 * q3 - 4 * q2 * s],
        [8 * q2 * q1 - 4 * q3 * s, 4 * (2 * q2 * q2 - d1) + s * s, 8 * q2 * q3 + 4 * q1 * s],
        [8 * q3 * q1 + 4 * q2 * s, 8 * q3 * q2 - 4 * q1 * s, 4 * (2 * q3 * q3 - d1) + s * s],
    ]) / d


def columns(table: Table, prefix: str, count: int) -> np.ndarray:
    """The ``count`` columns ``<prefix>_0`` ... as an ``N x count`` array."""
    return np.column_stack([table[f"{prefix}_{index}"] for index in range(count)])


def wheel_torques(table: Table) -> np.ndarray:
    """Upstream ``rwLogs[i].u_current`` of the four wheels."""
    return np.column_stack([table[f"RW{index}_u_current_Nm"] for index in range(4)])


def camera_truth(position_n: np.ndarray, sigma_bn: np.ndarray) -> dict[str, np.ndarray]:
    """Upstream ``pull_outputs``' truth for each row: ``trueR_C`` (the
    spacecraft relative to Mars, camera frame) and ``trueCircles`` (Mars's
    centre and radius in the image [pixels])."""
    dcm_cb = mrp_to_dcm(SIGMA_CB)
    pixel_size = CAMERA_SIZE / CAMERA_RESOLUTION
    true_r_c = np.array([dcm_cb @ mrp_to_dcm(s) @ r for r, s in zip(position_n, sigma_bn)])
    rhat = true_r_c / np.linalg.norm(position_n, axis=1)[:, None]
    rhat = rhat * (CAMERA_FOCAL / rhat[:, 2])[:, None]
    radius = CAMERA_FOCAL * np.tan(np.arcsin(R_MARS / np.linalg.norm(position_n, axis=1))) / pixel_size[0]
    return {
        "r_C": true_r_c,
        "center_x": rhat[:, 0] / pixel_size[0] + CAMERA_RESOLUTION[0] / 2 - 0.5,
        "center_y": rhat[:, 1] / pixel_size[1] + CAMERA_RESOLUTION[1] / 2 - 0.5,
        "radius": radius,
    }


def plot_rw_motor_torque(plt: Any, time_min: np.ndarray, us_req: np.ndarray,
                         u_current: np.ndarray) -> dict[str, Any]:
    """Upstream ``plot_rw_motor_torque`` (it draws wheels 1 to 3)."""
    color = colors()
    figure = plt.figure(22, figsize=(3, 1.8), facecolor="w", edgecolor="k")
    for idx in range(1, 4):
        plt.plot(time_min, us_req[:, idx], "--", color=color[idx * 2],
                 label=r"$\hat u_{s," + str(idx) + "}$")
        plt.plot(time_min, u_current[:, idx - 1], color=color[idx * 2],
                 label="$u_{s," + str(idx) + "}$")
    plt.legend(loc="lower right")
    plt.xlabel("Time (min)")
    plt.ylabel("RW Motor Torque (Nm)")
    return {"RWMotorTorque": figure}


def plot_attitude_error(plt: Any, time_min: np.ndarray, sigma_br: np.ndarray) -> dict[str, Any]:
    """Upstream ``plot_attitude_error``."""
    figure = plt.figure(5555, figsize=(3, 1.8), facecolor="w", edgecolor="k")
    plt.plot(time_min, np.linalg.norm(sigma_br, axis=1), color=colors()[0])
    plt.xlabel("Time (min)")
    plt.xlim([40, 100])
    plt.ylabel(r"Attitude Error Norm $|\sigma_{C/R}|$")
    figure.gca().set_yscale("log")
    return {"AttErrorNorm": figure}


def plot_rate_error(plt: Any, time_min: np.ndarray, omega_br: np.ndarray) -> dict[str, Any]:
    """Upstream ``plot_rate_error``."""
    color = colors()
    figure = plt.figure(666666, figsize=(3, 1.8), facecolor="w", edgecolor="k")
    for idx, style in zip(range(1, 4), ["-", "--", ":"]):
        plt.plot(time_min, omega_br[:, idx - 1], color=color[idx * 2],
                 label=r"$\omega_{BR," + str(idx) + "}$", linestyle=style)
    plt.legend(loc="lower right")
    plt.xlim([40, 100])
    plt.xlabel("Time (min)")
    plt.ylabel("Rate Tracking Error (rad/s) ")
    return {"RateErrorControl": figure}


def plot_two_orbits(plt: Any, r_bn: np.ndarray, r_bn2: np.ndarray) -> dict[str, Any]:
    """Upstream ``plot_TwoOrbits``: the true orbit and the measured
    positions [km]."""
    color = colors()
    figure = plt.figure(5, figsize=(3.5, 2.0), facecolor="w", edgecolor="k")
    axis = figure.add_subplot(projection="3d")
    axis.set_xlabel(r"$R_x$, km")
    axis.set_ylabel(r"$R_y$, km")
    axis.set_zlabel(r"$R_z$, km")
    axis.plot(r_bn[:, 0] * M2KM, r_bn[:, 1] * M2KM, r_bn[:, 2] * M2KM, color=color[1], label="True")
    measured = (np.abs(r_bn2[:, 0]) > 0) | (np.abs(r_bn2[:, 1]) > 0)
    axis.scatter(r_bn2[measured, 0] * M2KM, r_bn2[measured, 1] * M2KM, r_bn2[measured, 2] * M2KM,
                 color=color[8], label="Measured")
    axis.scatter(0, 0, color="r")
    return {"Orbit": figure}


def diff_vectors(plt: Any, time_ns: np.ndarray, vec1: np.ndarray, vec2: np.ndarray,
                 valid: np.ndarray, label: str) -> dict[str, Any]:
    """Upstream ``diff_vectors``: the true minus the measured camera-frame
    position at the valid measurements, and the norm difference, with their
    means (upstream scales metres by ``m2km * m2km``, kept)."""
    color = colors()
    index = np.abs(valid - 1) < 0.01
    t = time_ns[index] * NS2MIN
    diff = vec1[index] - vec2[index]
    norms = np.linalg.norm(vec1[index], axis=1) - np.linalg.norm(vec2[index], axis=1)
    scale = M2KM * M2KM
    components = plt.figure(1, figsize=(3.5, 2.0), facecolor="w", edgecolor="k")
    for axis_index, (name, c) in enumerate(zip("xyz", [color[1], color[5], color[8]])):
        plt.plot(t, diff[:, axis_index] * scale, color=c, label=rf"${name}_\mathrm{{{label}}}$")
        plt.plot(t, np.full(len(t), np.mean(diff[:, axis_index]) * scale), color=c, linestyle="--")
    plt.legend()
    plt.ylabel(r"$\mathbf{r}_{\mathrm{true}} - \mathbf{r}_{\mathrm{opnav}}$ (km)")
    plt.xlabel("Time (min)")
    norm_figure = plt.figure(2, figsize=(3.5, 2.0), facecolor="w", edgecolor="k")
    plt.plot(t, norms * scale, color=color[1])
    plt.plot(t, np.full(len(t), np.mean(norms) * scale), color=color[1], linestyle="--")
    plt.ylabel(r"$|\mathbf{r}_{\mathrm{true}}|$ - $|\mathbf{r}_{\mathrm{opnav}}|$ (km)")
    plt.xlabel("Time (min)")
    return {"MeasErrorComponents": components, "MeasErrorNorm": norm_figure}


def percent_errors(position: np.ndarray, velocity: np.ndarray, states: np.ndarray,
                   variances: np.ndarray) -> np.ndarray:
    """Upstream ``nav_percentages``' ``diffPos``, ``diffVel``, ``covarPos``,
    ``covarVel`` rows [%] (``variances``: the six covariance diagonal
    entries)."""
    r = np.linalg.norm(position, axis=1)
    v = np.linalg.norm(velocity, axis=1)
    sigma = np.sqrt(variances)
    return np.column_stack([
        np.linalg.norm(states[:, 0:3] - position, axis=1) / r * 100,
        np.linalg.norm(states[:, 3:6] - velocity, axis=1) / v * 100,
        3 * np.linalg.norm(sigma[:, 0:3], axis=1) / r * 100,
        3 * np.linalg.norm(sigma[:, 3:6], axis=1) / v * 100,
    ])


def nav_percentages(plt: Any, time_ns: np.ndarray, position: np.ndarray, velocity: np.ndarray,
                    states: np.ndarray, variances: np.ndarray, valid: np.ndarray,
                    label: str) -> dict[str, Any]:
    """Upstream ``nav_percentages`` at the valid measurements, with its RMS
    printout (``variances``: the filter's covariance diagonal)."""
    color = colors()
    index = np.abs(valid - 1) < 0.01
    t = time_ns[index] * NS2MIN
    errors = percent_errors(position[index], velocity[index], states[index], variances[index])
    pos = plt.figure(101, figsize=(2.7, 1.6), facecolor="w", edgecolor="k")
    plt.plot(t, errors[:, 0], color=color[1], label="Error")
    plt.plot(t, errors[:, 2], color=color[8], linestyle="--", label=r"Covar ($3\sigma$)")
    plt.legend(loc="upper right")
    plt.ylabel(r"$\mathbf{r}_\mathrm{" + label + r"}$ errors ($\%$)")
    plt.xlabel("Time (min)")
    vel = plt.figure(102, figsize=(2.7, 1.6), facecolor="w", edgecolor="k")
    plt.plot(t, errors[:, 1], color=color[1])
    plt.plot(t, errors[:, 3], color=color[8], linestyle="--")
    plt.ylabel(r"$\dot{\mathbf{r}}_\mathrm{" + label + r"}$ errors ($\%$)")
    plt.xlabel("Time (min)")
    if len(errors):
        rms = np.sqrt(np.mean(errors ** 2, axis=0))
        print(f"-------- Pos RMS {label}--------\n{rms[0]}\n{rms[2]}")
        print(f"-------- Vel RMS {label}--------\n{rms[1]}\n{rms[3]}")
    return {"PercentErrorPos": pos, "PercentErrorVel": vel}


def plot_state_covar(plt: Any, time_ns: np.ndarray, error: np.ndarray,
                     variances: np.ndarray) -> dict[str, Any]:
    """Upstream ``plotStateCovarPlot`` of a 6-state filter: each position and
    velocity error [km] with its 3-sigma bound (``variances``: the
    covariance diagonal); rows before the filter runs are NaN."""
    color = colors()
    t = time_ns * NS2MIN
    figures = {}
    for axis in range(3):
        for kind, offset, ylabel in [("pos", 0, r"$r_{}$ Error (km)"), ("vel", 3, r"$v_{}$ Error (km/s)")]:
            state = axis + offset
            figure = plt.figure(20 + 2 * axis + offset // 3, figsize=(2.4, 1.4), facecolor="w", edgecolor="k")
            bound = 3 * np.sqrt(variances[:, state]) * M2KM
            plt.plot(t, error[:, state] * M2KM, color=color[1], label="State Error")
            plt.plot(t, bound, "--", color=color[8], label=r"Covar (3-$\sigma$)")
            plt.plot(t, -bound, "--", color=color[8])
            if state == 0:
                plt.legend(loc="best")
            plt.ylabel(ylabel.format(axis + 1))
            if axis == 2:
                plt.xlabel("Time (min)")
            figures[f"Filter{kind}{axis + 1}"] = figure
    return figures


def img_proc_vs_exp(plt: Any, time_ns: np.ndarray, truth: dict[str, np.ndarray],
                    centers: np.ndarray, radii: np.ndarray) -> dict[str, Any]:
    """Upstream ``imgProcVsExp``: the circles found against the true ones,
    and their differences [pixels]; rows without a circle are dropped."""
    color = colors()
    t = time_ns * NS2MIN
    centers = centers.astype(float).copy()
    radii = radii.astype(float).copy()
    missing = (np.abs(centers[:, 0]) < 1e-10) & (np.abs(centers[:, 1]) < 1e-10)
    centers[missing] = np.nan
    radii[missing] = np.nan
    found = np.cumsum(~missing) > 0
    centerline = np.where(found, CAMERA_RESOLUTION[0] / 2 + 0.5, np.nan)
    figures = {}
    for number, name, true, measured, label in [
        (301, "Xpix", truth["center_x"], centers[:, 0], "X"),
        (302, "Ypix", truth["center_y"], centers[:, 1], "Y"),
    ]:
        figure = plt.figure(number, figsize=(2.7, 1.6), facecolor="w", edgecolor="k")
        plt.plot(t, true, "+", label=f"Truth {label}pix", color=color[1])
        plt.plot(t, centerline, "--", label=f"{label}-center", color=color[9])
        plt.plot(t, measured, ".", label=f"ImagProc {label}pix", color=color[5], alpha=0.7)
        plt.legend(loc="best")
        if np.isfinite(centerline[-1]):
            plt.ylim([centerline[-1] - 15, centerline[-1] + 15])
        plt.ylabel(f"{label} (px)")
        plt.xlabel("Time (min)")
        figures[name] = figure
    figure = plt.figure(312, figsize=(2.7, 1.6), facecolor="w", edgecolor="k")
    plt.plot(t, truth["radius"], "+", label=r"Truth $\rho$", color=color[1])
    plt.plot(t, radii, ".", label=r"ImagProc $\rho$", color=color[5], alpha=0.7)
    plt.legend(loc="best")
    plt.ylabel(r"$\rho$ (px)")
    plt.xlabel("Time (min)")
    figures["Rhopix"] = figure
    for number, name, difference, label in [
        (303, "Xerror", truth["center_x"] - centers[:, 0], r"$\mathrm{X}_\mathrm{true} - \mathrm{X}_\mathrm{hough}$"),
        (304, "Yerror", truth["center_y"] - centers[:, 1], r"$\mathrm{Y}_\mathrm{true} - \mathrm{Y}_\mathrm{hough}$"),
        (305, "Rhoerror", truth["radius"] - radii, r"$\mathrm{\rho}_\mathrm{true} - \mathrm{\rho}_\mathrm{hough}$"),
    ]:
        figure = plt.figure(number, figsize=(2.7, 1.6), facecolor="w", edgecolor="k")
        plt.plot(t, difference, ".", label=label, color=color[1])
        plt.legend(loc="best")
        plt.ylabel(f"{name[:-5]} error (px)")
        plt.grid()
        figures[name] = figure
    figures["Rhoerror"].gca().set_xlabel("Time (min)")
    return figures


def plot_post_fit_residuals(plt: Any, time_ns: np.ndarray, residuals: np.ndarray,
                            noise: np.ndarray) -> dict[str, Any]:
    """Upstream ``plotPostFitResiduals`` of a 3-measurement filter: each
    residual [km] (zeros, no measurement, dropped) with the measurement's
    3-sigma noise over 5 (``noise`` row-major 3x3, NaN without a
    measurement, which holds the last one)."""
    color = colors()
    t = time_ns * NS2MIN
    residuals = residuals.astype(float).copy()
    residuals[np.abs(residuals) < 1e-10] = np.nan
    meas_noise = np.zeros((len(t), 3))
    last = np.zeros(9)
    for row in range(len(t)):
        if not np.isnan(noise[row, 0]):
            last = noise[row]
        meas_noise[row] = 3 * np.sqrt(np.abs(last[[0, 4, 8]])) / 5
    figures = {}
    for axis in range(3):
        figure = plt.figure(401 + axis, figsize=(2.7, 1.6), facecolor="w", edgecolor="k")
        plt.plot(t, residuals[:, axis] * M2KM, ".", label="Residual", color=color[1])
        plt.plot(t, meas_noise[:, axis] * M2KM, "--", label=r"Noise ($3\sigma$)", color=color[8])
        plt.plot(t, -meas_noise[:, axis] * M2KM, "--", color=color[8])
        if axis == 0:
            plt.legend(loc=2)
        bound = np.nanmax(meas_noise[:, axis]) if len(t) else 0.0
        if bound > 1e-15:
            plt.ylim([-2 * bound * M2KM, 2 * bound * M2KM])
        plt.ylabel(rf"$r_{axis + 1}$ Measured (km)")
        plt.xlabel("Time (min)")
        plt.grid()
        figures[f"Res{axis + 1}"] = figure
    return figures


def plot_circles(plt: Any, centers: np.ndarray, radii: np.ndarray,
                 valid: np.ndarray) -> dict[str, Any]:
    """Upstream ``plot_cirlces``: every 30th valid circle in the image."""
    from matplotlib.patches import Ellipse

    import matplotlib

    index = np.flatnonzero(valid == 1)
    inferno = matplotlib.colormaps["inferno"].colors
    step = len(inferno) / (len(index) + 1)
    figure = plt.figure(500, figsize=(3, 3), facecolor="w", edgecolor="k")
    axis = plt.gca()
    for i, row in enumerate(index):
        if i % 30 == 0:
            ellipse = Ellipse(xy=(centers[row, 0], centers[row, 1]), width=radii[row],
                              height=radii[row], angle=0, linestyle="-", linewidth=3,
                              color=inferno[int(i * step)], alpha=0.7)
            ellipse.set(facecolor="none")
            axis.add_patch(ellipse)
    axis.set_xlim(0, CAMERA_RESOLUTION[0])
    axis.set_ylim(CAMERA_RESOLUTION[1], 0)
    plt.xlabel("X-axis (px)")
    plt.ylabel("Y-axis (px)")
    plt.axis("equal")
    return {"Circles": figure}


def plot_limb(plt: Any, limbs: Table) -> dict[str, Any]:
    """Upstream ``plot_limb`` from the ``<stem>_limb.csv`` the examples write:
    the points of every 30th valid limb, and the limb sizes."""
    import matplotlib

    index = limbs["limb_index"]
    count = int(index.max()) + 1 if len(index) else 0
    inferno = matplotlib.colormaps["inferno"].colors
    step = len(inferno) / (count + 1)
    figure = plt.figure(600, figsize=(3, 3), facecolor="w", edgecolor="k")
    axis = plt.gca()
    x, y = limbs["limbPoints_x"], limbs["limbPoints_y"]
    for limb in range(0, count, 30):
        rows = (index == limb) & np.isfinite(x)
        axis.scatter(x[rows], y[rows], color=inferno[int(limb * step)], marker=".", alpha=0.2)
    axis.set_xlim(0, CAMERA_RESOLUTION[0])
    axis.set_ylim(CAMERA_RESOLUTION[1], 0)
    plt.xlabel("X-axis (px)")
    plt.ylabel("Y-axis (px)")
    plt.axis("equal")
    sizes = plt.figure(601, figsize=(2.7, 1.6), facecolor="w", edgecolor="k")
    first = np.unique(index, return_index=True)[1]
    plt.plot(limbs.time_ns[first] * NS2MIN, limbs["numLimbPoints"][first], color=inferno[int(step)])
    plt.xlabel("Time (min)")
    plt.ylabel("Limb Size (px)")
    return {"Limbs": figure, "LimbPoints": sizes}


def plot_faults(plt: Any, time_ns: np.ndarray, faults: np.ndarray, valid1: np.ndarray,
                valid2: np.ndarray) -> dict[str, Any]:
    """Upstream ``plot_faults``."""
    color = colors()
    t = time_ns * NS2MIN
    figure = plt.figure(10101, figsize=(2.7, 1.6), facecolor="w", edgecolor="k")
    plt.xlabel("Time (min)")
    plt.ylabel("Detected Fault")
    plt.scatter(t, valid1, color=color[5], alpha=0.1, label="Limb")
    plt.scatter(t, valid2, color=color[8], alpha=0.1, label="Circ")
    plt.scatter(t, faults, marker=".", color=color[2], label="Faults")
    plt.legend()
    return {"FaultsDetected": figure}


def vec_track(plt: Any, time_ns: np.ndarray, ref: np.ndarray, track: np.ndarray,
              covar: np.ndarray) -> dict[str, Any]:
    """Upstream ``vecTrack``: the filtered unit heading against the truth in
    the camera frame, its off-point [deg] and 3-sigma bounds."""
    color = colors()
    t = time_ns * NS2MIN
    error = ref - track
    off_point = np.degrees(np.arccos(np.clip(np.sum(ref * track, axis=1), -1, 1)))
    covar_vec = track + np.sqrt(np.abs(np.column_stack([covar[:, 0, 0], covar[:, 1, 1], covar[:, 2, 2]])))
    covar_vec /= np.linalg.norm(covar_vec, axis=1)[:, None]
    cov_deg = 3 * np.degrees(np.arccos(np.clip(np.sum(covar_vec * track, axis=1), -1, 1)))
    figures = {}
    figure = plt.figure(101011, figsize=(2.7, 1.6), facecolor="w", edgecolor="k")
    plt.plot(t, off_point, color=color[1], label="Off-point")
    plt.plot(t, cov_deg, color=color[5], linestyle="--", label=r"Covar (3-$\sigma$)")
    plt.legend(loc="upper right")
    plt.ylabel(r"Mean $\hat{\mathbf{h}}$ Error in $\mathcal{C}$ ($^\circ$)")
    plt.xlabel("Time (min)")
    plt.ylim([0, 2])
    figures["HeadingDeg"] = figure
    for axis in range(3):
        figure = plt.figure(10101 + 100 * axis, figsize=(2.7, 1.6), facecolor="w", edgecolor="k")
        bound = 3 * np.sqrt(np.abs(covar[:, axis, axis]))
        plt.plot(t, error[:, axis], color=color[2], label="Error")
        plt.plot(t, bound, color=color[8], linestyle="--", label=r"Covar (3-$\sigma$)")
        plt.plot(t, -bound, color=color[8], linestyle="--")
        if axis == 0:
            plt.legend()
        plt.ylabel(r"$\hat{\mathbf{h}}_{" + str(axis + 1) + r"}$ Error in $\mathcal{C}$ (-)")
        plt.ylim([-0.04, 0.04])
        plt.xlabel("Time (min)")
        figures[f"HeadingCam{axis + 1}"] = figure
    return figures


def filter_variances(table: Table, prefix: str = "filter") -> np.ndarray:
    """The six covariance diagonal entries the examples record of a 6-state
    filter (``<prefix>_covar_0``, ``_7``, ... ``_35``)."""
    return np.column_stack([table[f"{prefix}_covar_{7 * index}"] for index in range(6)])


def filter_errors(table: Table, prefix: str = "filter") -> tuple[np.ndarray, np.ndarray]:
    """Upstream ``stateError`` (truth minus the filter state) and the
    variances of ``navCovar`` of a 6-state filter from the scenario CSV, NaN
    before the filter runs (its covariance is still zero then)."""
    truth = np.column_stack([columns(table, "r_BN_N_m", 3), columns(table, "v_BN_N_m_per_s", 3)])
    state = columns(table, f"{prefix}_state", 6)
    variances = filter_variances(table, prefix)
    running = np.any(variances != 0, axis=1)
    error = np.where(running[:, None], truth - state, np.nan)
    return error, np.where(running[:, None], variances, np.nan)


def od_figures(plt: Any, table: Table, label: str, circles: bool, valid_column: str,
               percentages: bool = True) -> dict[str, Any]:
    """The orbit-determination figures shared by the OD and AttOD scenarios'
    ``pull_outputs`` with ``filterUse = "relOD"``; ``percentages`` adds
    ``nav_percentages`` (the AttOD scenarios)."""
    time = table.time_ns
    position = columns(table, "r_BN_N_m", 3)
    velocity = columns(table, "v_BN_N_m_per_s", 3)
    sigma = columns(table, "sigma_BN", 3)
    truth = camera_truth(position, sigma)
    valid = table[valid_column]
    meas_covar = columns(table, "opnav_covar_N_m2", 9).astype(float)
    meas_covar[table["opnav_valid"] < 0.5] = np.nan
    error, covar = filter_errors(table)
    figures = {}
    figures.update(plot_two_orbits(plt, position, columns(table, "opnav_r_BN_N_m", 3)))
    figures.update(diff_vectors(plt, time, truth["r_C"], columns(table, "opnav_r_BN_C_m", 3), valid, label))
    if percentages:
        figures.update(nav_percentages(plt, time, position, velocity, columns(table, "filter_state", 6),
                                       np.nan_to_num(covar), valid, label))
    if circles:
        centers = columns(table, "circlesCenters", 2)
        figures.update(plot_circles(plt, centers, table["circlesRadii_0"], table["circles_valid"]))
    figures.update(plot_state_covar(plt, time, error, covar))
    if circles:
        figures.update(img_proc_vs_exp(plt, time, truth, columns(table, "circlesCenters", 2),
                                       table["circlesRadii_0"]))
    figures.update(plot_post_fit_residuals(plt, time, columns(table, "filter_postFitRes", 3), meas_covar))
    return figures
