#!/usr/bin/env python3
"""Upstream `scenarioConstrainedDynamicsManeuverAnalysis` figures from the
per-gain CSVs `scenarioConstrainedDynamicsManeuverAnalysis_<maneuver>_<relpos>_<orbit>_<model>_alpha<gain>.csv`
and their `_summary.csv`: the direction and attitude constraint violations of
each gain, and the gain performance (mean violations and runtime)."""

import argparse
import sys

import numpy as np

from multibody_plotting import model_geometry, violation_angles_deg, violations
from orbit_plotting import NANO2MIN, Table, run_plots

STEM = "scenarioConstrainedDynamicsManeuverAnalysis"


def build_for(label):
    def build(generated_dir, plt):
        model = label.rsplit("_", 1)[1]
        maneuver = label.split("_")[1]
        runs = sorted(
            ((float(path.stem.rsplit("alpha", 1)[1]), Table(path))
             for path in generated_dir.glob(f"{STEM}{label}_alpha*.csv")),
            key=lambda run: run[0],
        )
        if not runs:
            return {}
        labels = [rf"$\alpha = ${gain:.0e}" for gain, _ in runs]
        prefix = STEM + maneuver + "maneuver_"
        direction, attitude = plt.figure(), plt.figure()
        for gain, table in runs:
            # The arm lies along the initial hub offset (the frames start aligned).
            offset = table.vectors("r_B2N_N_m")[0] - table.vectors("r_B1N_N_m")[0]
            psi, sigma = violations(table, *model_geometry(model, offset))
            time = table.time_ns * NANO2MIN
            direction.gca().semilogy(time, np.linalg.norm(psi, axis=1))
            attitude.gca().semilogy(time, np.linalg.norm(violation_angles_deg(sigma), axis=1))
        for figure, ylabel in ((direction, r"Direction Constraint Violation $\psi$ [m]"),
                               (attitude, r"Attitude Constraint Violation $\phi$ [deg]")):
            axis = figure.gca()
            axis.set_xlabel("time [min]")
            axis.set_ylabel(ylabel)
            axis.axis("tight")
            axis.legend(labels, loc="lower right")
        figures = {prefix + "DirectionConstraint": direction,
                   prefix + "AttitudeConstraint": attitude}

        summary_path = generated_dir / f"{STEM}{label}_summary.csv"
        if summary_path.is_file():
            summary = Table(summary_path)
            figure, axis = plt.subplots()
            axis.loglog(summary["gain"], summary["mean_direction_violation_m"],
                        color="tab:blue", linestyle="-")
            axis.loglog(summary["gain"], summary["mean_attitude_violation_deg"],
                        color="tab:blue", linestyle="--")
            axis.set_xlabel("gain")
            axis.set_ylabel("average constraint violation [m, deg]", color="tab:blue")
            axis.tick_params(axis="y", labelcolor="tab:blue")
            axis.legend(["direction", "attitude"], loc="upper center")
            runtime = axis.twinx()
            runtime.loglog(summary["gain"], summary["runtime_s"], color="tab:orange")
            runtime.set_ylabel("runtime [sec]", color="tab:orange")
            runtime.tick_params(axis="y", labelcolor="tab:orange")
            figures[prefix + "GainPerformance"] = figure
        return figures

    return build


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--maneuver", default="attitude", choices=("attitude", "orbit"))
    parser.add_argument("--relpos", default="alongtrackahead")
    parser.add_argument("--orbit", default="LEO", choices=("LEO", "noorbit"))
    parser.add_argument("--model", default="MEV2", choices=("bskSat", "MEV1", "MEV2"))
    known, rest = parser.parse_known_args()
    sys.argv = [sys.argv[0], *rest]
    config = f"_{known.maneuver}_{known.relpos}_{known.orbit}_{known.model}"
    raise SystemExit(run_plots("Plot the constrained dynamics maneuver analysis.",
                               build_for(config)))
