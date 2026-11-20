#!/usr/bin/env python3
"""Upstream `scenarioCustomGravBody` figure from `scenarioCustomGravBody.csv`:
the Itokawa-relative position components over the hours."""

from orbit_plotting import NANO2HOUR, existing, plot_components, run_plots


def build(generated_dir, plt):
    figures = {}
    for _, table in existing(generated_dir, ["scenarioCustomGravBody"]):
        relative = table.vectors("r_BN_N_m") - table.vectors("itokawa_r_N_m")
        figure = plt.figure()
        axis = figure.gca()
        axis.ticklabel_format(useOffset=False, style="plain")
        plot_components(axis, table.time_ns * NANO2HOUR, relative, "$r_{{BI,{i}}}$", plt)
        axis.set_xlabel("Time [h]")
        axis.set_ylabel("Itokawa Relative Position [m]")
        figures["scenarioCustomGravBody1"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the custom gravity body scenario.", build))
