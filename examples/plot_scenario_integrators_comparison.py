#!/usr/bin/env python3
"""Upstream `scenarioIntegratorsComparison` figure from
`scenarioIntegratorsComparison.csv`: the final position error and the run
time of each integrator against the time step."""

from orbit_plotting import Table, line_color, run_plots

# Upstream `INTEGRATORS`: column, label, color index, line style.
INTEGRATORS = (
    ("euler", "Euler", 0, "-"),
    ("rk2", "Heun", 1, "-"),
    ("rk4", "RK 4", 2, "-"),
    ("rkf45_10", r"RKF4(5) - $\epsilon = 10$", 3, "-"),
    ("rkf45_0.1", r"RKF4(5) - $\epsilon = 0.1$", 3, "--"),
    ("rkf78_10", r"RKF7(8) - $\epsilon = 10$", 4, "-"),
    ("rkf78_0.1", r"RKF7(8) - $\epsilon = 0.1$", 4, "--"),
)


def build(generated_dir, plt):
    path = generated_dir / "scenarioIntegratorsComparison.csv"
    if not path.is_file():
        return {}
    table = Table(path)
    steps = table["time_step_s"]
    figure, axes = plt.subplots(nrows=2, figsize=(12, 7))
    for axis, suffix, label in (
        (axes[0], "position_error_m", "Position error [m]"),
        (axes[1], "run_time_s", "Integration time [s]"),
    ):
        for key, legend, color, style in INTEGRATORS:
            axis.loglog(steps, table[f"{key}_{suffix}"], color=line_color(color, 5, plt),
                        linestyle=style, label=legend)
        axis.legend()
        axis.set_ylabel(label)
    axes[1].set_xlabel("Time step [s]")
    return {"scenarioIntegratorsComparison": figure}


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the integrator convergence study.", build))
