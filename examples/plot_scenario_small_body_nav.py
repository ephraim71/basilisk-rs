#!/usr/bin/env python3
"""Upstream ``scenarioSmallBodyNav`` figures from ``scenarioSmallBodyNav.csv``
(truth, filter state and covariance diagonal every step) and
``scenarioSmallBodyNavMeas.csv`` (the measurements, only while the
measurement task runs): the Hill-frame position and velocity about Bennu,
their estimation errors with ``2 sigma`` bounds, and Bennu's attitude and
rate with theirs."""

import numpy as np

from orbit_plotting import NANO2SEC, PlotError, existing, run_plots
from small_body_plotting import columns, component_figure, error_figure, relative_hill

STEM = "scenarioSmallBodyNav"
MEASUREMENT = {"fmt": "k*", "label": "measurement", "markersize": 1}


def build(generated_dir, plt):
    tables = dict(existing(generated_dir, [STEM, f"{STEM}Meas"]))
    if STEM not in tables:
        return {}
    if f"{STEM}Meas" not in tables:
        raise PlotError(f"{generated_dir / (STEM + 'Meas.csv')} is missing")
    table, meas = tables[STEM], tables[f"{STEM}Meas"]
    time = table.time_ns * NANO2SEC
    meas_time = meas.time_ns * NANO2SEC
    # Upstream: each measurement against the truth at its time.
    truth_index = np.searchsorted(table.time_ns, meas.time_ns)

    bennu_r, bennu_v = table.vectors("bennu_r_N_m"), table.vectors("bennu_v_N_m_per_s")
    r_truth, v_truth = relative_hill(
        bennu_r, bennu_v, table.vectors("r_BN_N_m"), table.vectors("v_BN_N_m_per_s"), rotating=False
    )
    r_meas, v_meas = relative_hill(
        bennu_r[truth_index],
        bennu_v[truth_index],
        meas.vectors("r_BN_N_meas_m"),
        meas.vectors("v_BN_N_meas_m_per_s"),
        rotating=False,
    )
    x_hat = columns(table, "x_hat", range(12))
    sigma = np.sqrt(columns(table, "P", range(12)))
    sigma_truth = table.vectors("sigma_AN")
    omega_truth = table.vectors("omega_AN_A_rad_per_s")

    def estimate_figure(meas_values, truth, estimate, truth_label, ylabels, title=None):
        return component_figure(
            plt,
            [
                (meas_time, meas_values, MEASUREMENT),
                (time, truth, {"label": truth_label}),
                (time, estimate, {"label": "estimate"}),
            ],
            ylabels,
            "Time [sec]",
            title,
        )

    figures = {
        f"{STEM}1": estimate_figure(
            r_meas, r_truth, x_hat[:, 0:3], r"${}^Or_{BO_{1}}$",
            [rf"${{}}^O r_{{BO_{{{i}}}}}$ [m]" for i in (1, 2, 3)],
            "Relative Spacecraft Position",
        ),
        f"{STEM}2": estimate_figure(
            v_meas, v_truth, x_hat[:, 3:6], "truth",
            [rf"${{}}^Ov_{{BO_{i}}}$ [m/s]" for i in (1, 2, 3)],
            "Relative Spacecraft Velocity",
        ),
        f"{STEM}3": error_figure(
            plt, time, r_truth - x_hat[:, 0:3], sigma[:, 0:3],
            [rf"${{}}^Or_{{BO_{i}}}$ Error [m]" for i in (1, 2, 3)],
            "Time [sec]", "Position Error and Covariance",
        ),
        f"{STEM}4": error_figure(
            plt, time, v_truth - x_hat[:, 3:6], sigma[:, 3:6],
            [rf"${{}}^Ov_{{BO_{i}}}$ Error [m/s]" for i in (1, 2, 3)],
            "Time [sec]", "Velocity Error and Covariance",
        ),
        f"{STEM}5": estimate_figure(
            meas.vectors("sigma_AN_meas"), sigma_truth, x_hat[:, 6:9], "truth",
            [rf"$\sigma_{{AN_{{{i}}}}}$ [rad]" for i in (1, 2, 3)],
        ),
        f"{STEM}6": estimate_figure(
            meas.vectors("omega_AN_A_meas_rad_per_s"), omega_truth, x_hat[:, 9:12], "truth",
            [rf"${{}}^A\omega_{{AN_{{{i}}}}}$ [rad/s]" for i in (1, 2, 3)],
        ),
        f"{STEM}7": error_figure(
            plt, time, sigma_truth - x_hat[:, 6:9], sigma[:, 6:9],
            [rf"$\sigma_{{AN_{{{i}}}}}$ Error [rad]" for i in (1, 2, 3)],
            "Time [sec]", "Attitude Error and Covariance",
        ),
        f"{STEM}8": error_figure(
            plt, time, omega_truth - x_hat[:, 9:12], sigma[:, 9:12],
            [rf"${{}}^A\omega_{{AN_{{{i}}}}}$ Error [rad/s]" for i in (1, 2, 3)],
            "Time [sec]", "Rate Error and Covariance",
        ),
    }
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots(f"Plot the {STEM} scenario.", build))
