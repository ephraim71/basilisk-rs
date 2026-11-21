#!/usr/bin/env python3
"""Upstream ``scenarioSmallBodyFeedbackControl`` figures from
``scenarioSmallBodyFeedbackControl.csv``: the Hill-frame position and
velocity about Bennu (truth and measurement), the commanded force, and the
spacecraft attitude and rate (truth and measurement)."""

from orbit_plotting import NANO2SEC, existing, run_plots
from small_body_plotting import component_figure, relative_hill

STEM = "scenarioSmallBodyFeedbackControl"
MEASUREMENT = {"fmt": "k*", "label": "measurement", "markersize": 1}


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, [STEM]):
        time = table.time_ns * NANO2SEC
        bennu = (table.vectors("bennu_r_N_m"), table.vectors("bennu_v_N_m_per_s"))
        r_truth, v_truth = relative_hill(
            *bennu, table.vectors("r_BN_N_m"), table.vectors("v_BN_N_m_per_s"), rotating=True
        )
        r_meas, v_meas = relative_hill(
            *bennu,
            table.vectors("r_BN_N_meas_m"),
            table.vectors("v_BN_N_meas_m_per_s"),
            rotating=True,
        )
        figures[f"{STEM}1"] = component_figure(
            plt,
            [
                (time, r_meas, MEASUREMENT),
                (time, r_truth, {"label": r"${}^Or_{BO_{1}}$"}),
            ],
            [rf"${{}}^Or_{{BO_{i}}}$ [m]" for i in (1, 2, 3)],
            "Time [sec]",
            "Relative Spacecraft Position",
        )
        figures[f"{STEM}2"] = component_figure(
            plt,
            [(time, v_meas, MEASUREMENT), (time, v_truth, {"label": "truth"})],
            [rf"${{}}^Ov_{{BO_{i}}}$ [m/s]" for i in (1, 2, 3)],
            "Time [sec]",
            "Relative Spacecraft Velocity",
        )
        figures[f"{STEM}3"] = component_figure(
            plt,
            [(time, table.vectors("forceRequestBody_N"), {"fmt": "k-", "label": "control"})],
            [rf"$\hat{{\mathbf{{b}}}}_{i}$ control [N]" for i in (1, 2, 3)],
            "Time [sec]",
        )
        figures[f"{STEM}4"] = component_figure(
            plt,
            [
                (time, table.vectors("sigma_BN_meas"), MEASUREMENT),
                (time, table.vectors("sigma_BN"), {"label": "truth"}),
            ],
            [rf"$\sigma_{{BN_{i}}}$ [rad]" for i in (1, 2, 3)],
            "Time [sec]",
        )
        figures[f"{STEM}5"] = component_figure(
            plt,
            [
                (time, table.vectors("omega_BN_B_meas_rad_per_s"), MEASUREMENT),
                (time, table.vectors("omega_BN_B_rad_per_s"), {"label": "truth"}),
            ],
            [rf"${{}}^B\omega_{{BN_{{{i}}}}}$ [rad/s]" for i in (1, 2, 3)],
            "Time [sec]",
        )
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(f"Plot the {STEM} scenario.", build))
