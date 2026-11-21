#!/usr/bin/env python3
"""Upstream `scenarioBranchingVerification` figures from
`scenarioBranchingVerificationConservation_<host>.csv` and
`scenarioBranchingVerificationConvergence.csv`: the impulse-momentum
residuals of each ported host, and the step-refinement residuals of the hub
rate and velocity against a slope-4 reference.

Upstream plots its `spinningBodyNDOF` and `linearTranslationNDOF` hosts; the
second is not ported, so the conservation figure shows the ported hosts."""

import numpy as np

from orbit_plotting import Table, run_plots

STEM = "scenarioBranchingVerification"
HOSTS = ("spinningBodyNDOF", "nHingedRigidBody")
COLORS = {"spinningBodyNDOF": "C0", "nHingedRigidBody": "C1", "linearTranslationNDOF": "C2"}
PLOT_STRIDE = 20


def conservation(generated_dir, plt):
    tables = [(host, generated_dir / f"{STEM}Conservation_{host}.csv") for host in HOSTS]
    tables = [(host, Table(path)) for host, path in tables if path.is_file()]
    if not tables:
        return None
    figure, axes = plt.subplots(len(tables), 2, figsize=(12, 6.5), sharex=True, squeeze=False)
    labels = (r"$\hat{\mathbf{n}}_1$", r"$\hat{\mathbf{n}}_2$", r"$\hat{\mathbf{n}}_3$")
    for row, (host, table) in enumerate(tables):
        seconds = table["time_s"]
        left, right = axes[row]
        for index, (color, label) in enumerate(zip(("C0", "C1", "C2"), labels, strict=True)):
            left.plot(seconds[::PLOT_STRIDE], table[f"resH_N_m_s_{index}"][::PLOT_STRIDE],
                      color=color, linewidth=0.9, label=label)
            right.plot(seconds[::PLOT_STRIDE], table[f"resV_m_per_s_{index}"][::PLOT_STRIDE],
                       color=color, linewidth=0.9)
        left.set_ylabel(r"angular residual  [N$\cdot$m$\cdot$s]")
        right.set_ylabel(r"linear residual  [m/s]")
        for axis in (left, right):
            axis.axhline(0.0, color="0.4", linewidth=0.5)
            axis.ticklabel_format(axis="y", style="sci", scilimits=(0, 0), useMathText=True)
            axis.grid(True, linewidth=0.3, alpha=0.5)
            axis.set_xlim(0.0, seconds[-1])
            axis.text(0.985, 0.95, host, transform=axis.transAxes, ha="right", va="top", fontsize=12,
                      bbox=dict(boxstyle="round,pad=0.25", facecolor="white", edgecolor="0.7", alpha=0.9))
    for axis in axes[-1, :]:
        axis.set_xlabel(r"$t$ [s]")
    figure.legend(axes[0, 0].lines[:3], labels, loc="upper center", bbox_to_anchor=(0.5, 1.0), ncol=3)
    return figure


def convergence(generated_dir, plt):
    path = generated_dir / f"{STEM}Convergence.csv"
    if not path.is_file():
        return None
    table = Table(path)
    dts = table["dt_s"]
    figure, omega_axis = plt.subplots(figsize=(10.0, 6.0))
    velocity_axis = omega_axis.twinx()
    handles, labels, firsts, lasts = [], [], [], []
    for host in HOSTS:
        if f"{host}_omega_BN_B" not in table.columns:
            continue
        for axis, observable, marker, style, symbol in (
            (omega_axis, "omega_BN_B", "o", "-", r"\omega"),
            (velocity_axis, "v_BN_N", "s", "--", "v"),
        ):
            residuals = table[f"{host}_{observable}"]
            slope = np.polyfit(np.log(dts), np.log(residuals), 1)[0]
            (handle,) = axis.loglog(dts, residuals, color=COLORS[host], marker=marker, linestyle=style)
            handles.append(handle)
            labels.append(rf"{host}, ${symbol}$  ($p={slope:+.2f}$)")
            firsts.append(residuals[0])
            lasts.append(residuals[-1])
    anchor = max(firsts) * 3.0
    (reference,) = omega_axis.loglog(dts, anchor * (dts / dts[0]) ** 4, "k:", linewidth=1.5, alpha=0.8)
    handles.append(reference)
    labels.append("slope = 4")
    omega_axis.set_xlabel(r"integration step  $\Delta t$  [s]")
    omega_axis.set_ylabel(r"$\|\,\omega_{B/N}^{(\Delta t)} - \omega_{B/N}^{(\Delta t/2)}\,\|$ at $t = T$   [rad/s]")
    velocity_axis.set_ylabel(r"$\|\,v_{B/N}^{(\Delta t)} - v_{B/N}^{(\Delta t/2)}\,\|$ at $t = T$   [m/s]")
    omega_axis.grid(True, which="both", linewidth=0.3, alpha=0.5)
    low, high = min(lasts) * 0.3, anchor * 3.0
    omega_axis.set_ylim(low, high)
    velocity_axis.set_ylim(low, high)
    omega_axis.legend(handles, labels, loc="upper left", ncol=2, framealpha=0.95, fontsize=11)
    return figure


def build(generated_dir, plt):
    figures = {}
    for name, figure in (("Conservation", conservation(generated_dir, plt)),
                         ("Convergence", convergence(generated_dir, plt))):
        if figure is not None:
            figures[STEM + name] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the effector branching verification.", build))
