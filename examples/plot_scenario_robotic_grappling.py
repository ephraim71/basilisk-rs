#!/usr/bin/env python3
"""Upstream `scenarioRoboticGrappling` figures from
`scenarioRoboticGrappling<variant>.csv` and its `_events.csv`: the hub
separation, the arm extension, and the position and attitude constraint
violations after the grapple, with the grapple and berth events marked."""

import argparse
import sys

import numpy as np

from multibody_plotting import mrp_to_dcm, sub_mrp, violation_angles_deg
from orbit_plotting import NANO2SEC, Table, run_plots

STEM = "scenarioRoboticGrappling"
R_F0B_B = np.array([1.0, 0.0, 0.0])
F_HAT_B = np.array([1.0, 0.0, 0.0])
R_P2B2_B2 = np.array([0.0, 0.75, -1.1])
SIGMA_B2N_INIT = np.array([0.0, np.tan(np.pi / 8.0), 0.0])


def _annotate(plt, axis, capture_min, retraction_min):
    for value, label, color, style in ((capture_min, "grapple", "0.5", "--"),
                                       (retraction_min, "berth", "k", ":")):
        axis.axvline(value, color=color, linestyle=style, linewidth=1)
        axis.text(value, 0.03, " " + label, rotation=90, verticalalignment="bottom",
                  horizontalalignment="left", transform=axis.get_xaxis_transform(),
                  fontsize=plt.rcParams["xtick.labelsize"])


def build_for(variant):
    def build(generated_dir, plt):
        path = generated_dir / f"{STEM}{variant}.csv"
        if not path.is_file():
            return {}
        table = Table(path)
        events = Table(generated_dir / f"{STEM}{variant}_events.csv")
        capture_s = events["capture_time_s"][0]
        retraction_s = events["retraction_start_s"][0]
        time_s = table.time_ns * NANO2SEC
        time_min = time_s / 60.0
        capture_min, retraction_min = capture_s / 60.0, retraction_s / 60.0

        r1, s1 = table.vectors("r_B1N_N_m"), table.vectors("sigma_B1N")
        r2, s2 = table.vectors("r_B2N_N_m"), table.vectors("sigma_B2N")
        rho = table["rho_m"]
        separation = np.linalg.norm(r2 - r1, axis=1)
        psi = np.empty(r1.shape)
        sigma = np.empty(s1.shape)
        for i in range(len(rho)):
            dcm_b1n = mrp_to_dcm(s1[i])
            dcm_nb2 = mrp_to_dcm(s2[i]).T
            tip_b1 = R_F0B_B + rho[i] * F_HAT_B
            psi[i] = dcm_b1n @ r1[i] + tip_b1 - (dcm_b1n @ r2[i] + dcm_b1n @ dcm_nb2 @ R_P2B2_B2)
            sigma[i] = sub_mrp(sub_mrp(s2[i], s1[i]), SIGMA_B2N_INIT)
        floor = np.finfo(float).eps
        after = time_s >= capture_s

        figures = {}
        for name, values, ylabel in (
            ("HubSeparation", separation, "Hub separation " r"$\|r_{B_2/B_1}\|$ [m]"),
            ("ArmExtension", rho, r"Arm extension $\rho$ [m]"),
        ):
            figure = plt.figure()
            axis = figure.gca()
            axis.plot(time_min, values)
            axis.set_xlabel("time [min]")
            axis.set_ylabel(ylabel)
            axis.axis("tight")
            _annotate(plt, axis, capture_min, retraction_min)
            figures[STEM + name] = figure

        figure = plt.figure()
        axis = figure.gca()
        axis.semilogy(time_min[after], np.maximum(np.linalg.norm(psi, axis=1)[after], floor))
        axis.set_xlabel("time [min]")
        axis.set_ylabel(r"Position constraint violation $\psi$ [m]")
        axis.axis("tight")
        _annotate(plt, axis, capture_min, retraction_min)
        limits = axis.get_xlim()
        figures[STEM + "PositionConstraint"] = figure

        figure = plt.figure()
        axis = figure.gca()
        angle = np.linalg.norm(violation_angles_deg(sigma), axis=1)
        axis.semilogy(time_min[after], np.maximum(angle[after], floor))
        axis.set_xlabel("time [min]")
        axis.set_ylabel(r"Attitude constraint violation $\phi$ [deg]    ")
        axis.set_xlim(limits)
        _annotate(plt, axis, capture_min, retraction_min)
        figures[STEM + "AttitudeConstraint"] = figure
        return figures

    return build


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--variant", default="", choices=("", "Test", "Custom"))
    known, rest = parser.parse_known_args()
    sys.argv = [sys.argv[0], *rest]
    raise SystemExit(run_plots("Plot the robotic grappling scenario.", build_for(known.variant)))
