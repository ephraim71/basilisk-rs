"""Upstream ``examples/BskSim/plotting/BSK_Plotting.py`` for the Rust CSVs.

Each function draws one of upstream's figures from a :class:`CsvTable`
written by a ``bsk_sim_*`` example and returns it; the ``plot_bsk_sim_*.py``
scripts collect them under upstream's figure names
(``<scenario>_<figureName>``). As upstream's ``pull_outputs``, the scripts drop
the first sample (``np.delete(..., 0, 0)``) where upstream does, with
:func:`drop_first`.
"""
from __future__ import annotations

import math
from dataclasses import replace
from typing import Any, Sequence

import numpy as np

from scenario_plotting import CsvTable, run_variant_plot

M2KM = 1.0 / 1000.0
RPM = 2.0 * math.pi / 60.0
REQ_EARTH_M = 6378.1366 * 1000.0
RP_EARTH_M = 6356.7519 * 1000.0
MU_EARTH = 398600.436e9
COLOR_X, COLOR_Y, COLOR_Z = "dodgerblue", "salmon", "lightgreen"


def line_color(index: int, count: int) -> Any:
    """Upstream ``simHelpers.getLineColor``: the ``index``-th of ``count``
    colours of the ``gist_earth`` map."""
    import matplotlib

    return matplotlib.colormaps["gist_earth"](float(index + 1) / (count + 1))


def drop_first(table: CsvTable) -> CsvTable:
    """Upstream's ``np.delete(x, 0, 0)``: the first sample is the zeroed FSW
    gateway, recorded before the flight software first runs."""
    return replace(table, rows=table.rows[1:])


def minutes(table: CsvTable) -> np.ndarray:
    return np.array(table.column("time_ns")) * 1e-9 / 60.0


def vectors(table: CsvTable, prefix: str, count: int = 3) -> np.ndarray:
    return np.array([table.column(f"{prefix}_{index}") for index in range(count)]).T


def mrp_to_dcm(sigma: Sequence[float]) -> np.ndarray:
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


def rv2elem(mu: float, r: np.ndarray, v: np.ndarray) -> tuple[float, float, float]:
    """``(rmag, f, omega)`` of upstream ``orbitalMotion.rv2elem``."""
    h = np.cross(r, v)
    rmag = np.linalg.norm(r)
    e_vec = np.cross(v, h) / mu - r / rmag
    e = np.linalg.norm(e_vec)
    node = np.cross([0.0, 0.0, 1.0], h)
    n = np.linalg.norm(node)
    omega = math.acos(np.clip(np.dot(node, e_vec) / (n * e), -1, 1)) if n > 0 and e > 0 else 0.0
    if e_vec[2] < 0:
        omega = 2 * math.pi - omega
    f = math.acos(np.clip(np.dot(e_vec, r) / (e * rmag), -1, 1)) if e > 0 else 0.0
    if np.dot(r, v) < 0:
        f = 2 * math.pi - f
    return rmag, f, omega


def fixedframe2lla(r: np.ndarray, rad_equator: float, rad_ratio: float) -> float:
    """Latitude [rad] of upstream ``vizSupport.fixedframe2lla``."""
    x, y, z = r
    a, b = rad_equator, rad_equator * rad_ratio
    e2 = 1 - (b / a) ** 2
    ep2 = (a * a - b * b) / (b * b)
    rho = math.hypot(x, y)
    theta = math.atan2(z * a, rho * b)
    return math.atan2(z + ep2 * b * math.sin(theta) ** 3, rho - e2 * a * math.cos(theta) ** 3)


def plot3components(plt: Any, time: np.ndarray, vec: np.ndarray) -> Any:
    figure = plt.figure()
    plt.xlabel("Time, min")
    plt.plot(time, vec[:, 0], COLOR_X)
    plt.plot(time, vec[:, 1], COLOR_Y)
    plt.plot(time, vec[:, 2], COLOR_Z)
    return figure


def plot_sigma(plt: Any, time: np.ndarray, sigma: np.ndarray) -> Any:
    figure = plot3components(plt, time, sigma)
    plt.legend([r"$\sigma_1$", r"$\sigma_2$", r"$\sigma_3$"])
    plt.ylabel("MRP")
    return figure


def plot_orbit(plt: Any, r_bn: np.ndarray) -> Any:
    figure = plt.figure()
    plt.xlabel("$R_x$, km")
    plt.ylabel("$R_y$, km")
    plt.plot(r_bn[:, 0] * M2KM, r_bn[:, 1] * M2KM, COLOR_X)
    plt.scatter(0, 0, c=COLOR_X)
    plt.title("Spacecraft Orbit")
    return figure


def plot_attitude_error(plt: Any, time: np.ndarray, sigma_br: np.ndarray) -> Any:
    figure = plt.figure()
    plt.plot(time, np.linalg.norm(sigma_br, axis=1), color=line_color(1, 3))
    plt.xlabel("Time [min]")
    plt.ylabel(r"Attitude Error Norm $|\sigma_{B/R}|$")
    figure.gca().set_yscale("log")
    return figure


def plot_control_torque(plt: Any, time: np.ndarray, lr: np.ndarray) -> Any:
    figure = plt.figure()
    for index in range(3):
        plt.plot(time, lr[:, index], color=line_color(index, 3), label=f"$L_{{r,{index}}}$")
    plt.legend(loc="lower right")
    plt.xlabel("Time [min]")
    plt.ylabel("Control Torque $L_r$ [Nm]")
    return figure


def plot_rate_error(plt: Any, time: np.ndarray, omega_br: np.ndarray) -> Any:
    figure = plt.figure()
    for index in range(3):
        plt.plot(time, omega_br[:, index], color=line_color(index, 3),
                 label=rf"$\omega_{{BR,{index}}}$")
    plt.legend(loc="lower right")
    plt.xlabel("Time [min]")
    plt.ylabel("Rate Tracking Error [rad/s] ")
    return figure


def plot_orientation(plt: Any, time: np.ndarray, r: np.ndarray, v: np.ndarray,
                     sigma_bn: np.ndarray) -> Any:
    data = np.empty([len(r), 3])
    for index in range(len(r)):
        ir = r[index] / np.linalg.norm(r[index])
        hv = np.cross(r[index], v[index])
        ih = hv / np.linalg.norm(hv)
        itheta = np.cross(ih, ir)
        dcm_bn = mrp_to_dcm(sigma_bn[index])
        data[index] = [np.dot(ir, dcm_bn[0]), np.dot(itheta, dcm_bn[1]), np.dot(ih, dcm_bn[2])]
    figure = plt.figure()
    labels = (r"$\hat\imath_r\cdot \hat b_1$", r"${\hat\imath}_{\theta}\cdot \hat b_2$",
              r"$\hat\imath_h\cdot \hat b_3$")
    for index in range(3):
        plt.plot(time, data[:, index], color=line_color(index, 3), label=labels[index])
    plt.legend(loc="lower right")
    plt.xlabel("Time [min]")
    plt.ylabel("Orientation Illustration")
    return figure


def plot_attitude_guidance(plt: Any, time: np.ndarray, sigma_rn: np.ndarray) -> Any:
    """Upstream ``plot_attitudeGuidance``'s first figure, the one upstream
    saves as ``attitudeGuidance``: the reference attitude. (Its rate figure
    is drawn after the last saved one.)"""
    figure = plot_sigma(plt, time, sigma_rn)
    plt.ylim([-1.0, 1.0])
    plt.title(r"Ref Att: $\sigma_{RN}$")
    return figure


def plot_illumination_fraction(plt: Any, time: np.ndarray, illumination: np.ndarray) -> Any:
    figure = plt.figure()
    plt.plot(time, illumination)
    plt.xlabel("Time min")
    plt.ylabel("Illumination Fraction")
    return figure


def plot_sun_point(plt: Any, time: np.ndarray, sun_point: np.ndarray) -> Any:
    figure = plot3components(plt, time, sun_point)
    plt.xlabel("Time")
    plt.ylabel("Sun Point Vec")
    return figure


def plot_rw_cmd_torque(plt: Any, time: np.ndarray, us_req: np.ndarray, num_rw: int) -> Any:
    figure = plt.figure()
    for index in range(3):
        plt.plot(time, us_req[:, index], "--", color=line_color(index, num_rw),
                 label=rf"$\hat u_{{s,{index}}}$")
    plt.legend(loc="lower right")
    plt.xlabel("Time [min]")
    plt.ylabel("RW Motor Torque (Nm)")
    return figure


def plot_rw_cmd_actual_torque(plt: Any, time: np.ndarray, us_req: np.ndarray,
                              us_rw: np.ndarray, num_rw: int) -> Any:
    figure = plt.figure()
    for index in range(num_rw):
        plt.plot(time, us_req[:, index], "--", color=line_color(index, num_rw),
                 label=rf"$\hat u_{{s,{index}}}$")
        plt.plot(time, us_rw[:, index], color=line_color(index, num_rw),
                 label=f"$u_{{s,{index}}}$")
    plt.legend(loc="lower right")
    plt.xlabel("Time [min]")
    plt.ylabel("RW Motor Torque (Nm)")
    return figure


def plot_rw_speeds(plt: Any, time: np.ndarray, omega_rw: np.ndarray, num_rw: int) -> Any:
    figure = plt.figure()
    for index in range(num_rw):
        plt.plot(time, omega_rw[:, index] / RPM, color=line_color(index, num_rw),
                 label=rf"$\Omega_{{{index}}}$")
    plt.legend(loc="upper right")
    plt.xlabel("Time [min]")
    plt.ylabel("RW Speed (RPM) ")
    return figure


def plot_rw_friction(plt: Any, time: np.ndarray, friction: np.ndarray, num_rw: int,
                     fault_times_min: Sequence[float] = ()) -> Any:
    figure = plt.figure()
    for index in range(num_rw):
        plt.plot(time, friction[:, index], color=line_color(index, num_rw),
                 label=rf"$RW_{{{index + 1}}} Friction$")
    if len(fault_times_min):
        plt.scatter(fault_times_min, np.zeros(len(fault_times_min)), marker="x",
                    color=(1, 0, 0), label="Faults")
    plt.legend(loc="upper right")
    plt.xlabel("Time [min]")
    plt.ylabel("RW Static Friction ")
    return figure


def plot_peri_and_orbit(plt: Any, figure: Any, oe: dict[str, float], mu: float,
                        r: np.ndarray, v: np.ndarray) -> None:
    """Upstream ``plot_peri_and_orbit``, onto ``figure`` (upstream draws both
    orbits on the current figure)."""
    plt.figure(figure.number)
    p = oe["a"] * (1 - oe["e"] ** 2)
    radius, angle = [], []
    for position, velocity in zip(r, v):
        rmag, f, omega = rv2elem(mu, position, velocity)
        radius.append(rmag)
        angle.append(f + omega - oe["omega"])
    radius, angle = np.array(radius), np.array(angle)
    plt.plot(radius * np.cos(angle) / 1000, radius * np.sin(angle) / 1000, color="#aa0000",
             linewidth=3.0)
    f = np.linspace(0, 2 * np.pi, 100)
    track = p / (1 + oe["e"] * np.cos(f))
    plt.plot(track * np.cos(f) / 1000, track * np.sin(f) / 1000, "--", color="#555555")


def plot_rel_orbit(plt: Any, time: np.ndarray, r_chief: np.ndarray, r_deputy: np.ndarray) -> Any:
    figure = plt.figure()
    for index, label in enumerate("xyz"):
        plt.plot(time, r_chief[:, index] - r_deputy[:, index], label=label)
    plt.legend()
    plt.grid()
    return figure


def _stacked(plt: Any, title: str, ylabel: str) -> tuple[Any, Any]:
    figure, axes = plt.subplots(3, sharex=True)
    figure.add_subplot(111, frameon=False)
    plt.tick_params(labelcolor="none", top=False, bottom=False, left=False, right=False)
    plt.xlabel("Time [min]")
    plt.title(title)
    for index, axis in enumerate(axes):
        axis.set_ylabel(ylabel.format(index + 1))
    return figure, axes


def plot_position(plt: Any, time: np.ndarray, r_truth: np.ndarray, r_meas: np.ndarray,
                  t_tn: float, r_tn: Sequence[float]) -> Any:
    figure, axes = _stacked(plt, "Spacecraft Position", "${{}}^Nr_{{BN_{}}}$ [m]")
    for index, axis in enumerate(axes):
        axis.plot(time, r_meas[:, index], "k*", markersize=2,
                  label="measurement" if index == 0 else None)
        axis.plot(time, r_truth[:, index], label="truth" if index == 0 else None)
        axis.plot(t_tn, r_tn[index], "rx", label="target" if index == 0 else None)
    axes[0].legend(loc="upper right")
    return figure


def plot_velocity(plt: Any, time: np.ndarray, v_truth: np.ndarray, v_meas: np.ndarray) -> Any:
    figure, axes = _stacked(plt, "Spacecraft Velocity", "${{}}^Nv_{{BN_{}}}$ [m/s]")
    for index, axis in enumerate(axes):
        axis.plot(time, v_meas[:, index], "k*", markersize=2,
                  label="measurement" if index == 0 else None)
        axis.plot(time, v_truth[:, index], label="truth" if index == 0 else None)
    axes[0].legend()
    return figure


def plot_surface_rel_velocity(plt: Any, time: np.ndarray, r: np.ndarray, v: np.ndarray,
                              sigma_pn: np.ndarray, omega_pn_p: np.ndarray) -> Any:
    v_bs_s = np.empty([len(r), 3])
    for index in range(len(r)):
        dcm_pn = mrp_to_dcm(sigma_pn[index])
        omega_pn_n = dcm_pn.T @ omega_pn_p[index]
        s1 = np.cross(omega_pn_n, r[index])
        s1 = s1 / np.linalg.norm(s1)
        s3 = r[index] / np.linalg.norm(r[index])
        s2 = np.cross(s3, s1)
        s2 = s2 / np.linalg.norm(s2)
        v_bs_s[index] = np.array([s1, s2, s3]) @ (v[index] - np.cross(omega_pn_n, r[index]))
    figure, axes = _stacked(plt, "Surface Relative Velocity", "${{}}^Sv_{{BS_{}}}$ [m/s]")
    for index, axis in enumerate(axes):
        axis.plot(time, v_bs_s[:, index])
    return figure


def plot_data_tam(plt: Any, time: np.ndarray, tam: np.ndarray) -> Any:
    figure = plt.figure()
    for index in range(3):
        plt.plot(time, tam[:, index] * 1e9, color=line_color(index, 3),
                 label=rf"$TAM\_S_{{{index}}}$")
    plt.legend(loc="lower right")
    plt.xlabel("Time [min]")
    plt.ylabel(r"Magnetic Field $TAM_S$ [nT]")
    return figure


def plot_data_lat(plt: Any, time: np.ndarray, r: np.ndarray, c_pn: np.ndarray) -> Any:
    figure = plt.figure()
    latitudes = [math.degrees(fixedframe2lla(c_pn[index] @ r[index], REQ_EARTH_M,
                                             RP_EARTH_M / REQ_EARTH_M))
                 for index in range(len(time))]
    plt.plot(time, latitudes)
    plt.yticks(np.arange(-90, 91, 30))
    plt.xlabel("Time [min]")
    plt.ylabel("Latitude [deg]")
    return figure


def run(stem: str, build: Any) -> int:
    """Plot ``<stem>.csv``: ``build(plt, table)`` returns upstream's figures
    by name, saved as ``<stem>_<name>.png``."""
    return run_variant_plot(
        f"Plot {stem} results.",
        stem,
        ("",),
        lambda plt, table, _variant: {
            f"{stem}_{name}": figure for name, figure in build(plt, table).items()
        },
    )

