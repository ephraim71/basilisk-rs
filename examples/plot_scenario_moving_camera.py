#!/usr/bin/env python3
"""Figure of upstream scenarioMovingCamera.py (its show_plots figure) from
the Rust CSV: the platforms' inertial attitudes and platform 1's
translation."""
from __future__ import annotations

from typing import Any

from scenario_plotting import CsvTable, _line_color, _time_seconds, _vector_columns, run_variant_plot

STEM = "scenarioMovingCamera"


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_seconds(table)
    figure, axes = plt.subplots(2, 1, sharex=True)
    for platform in (1, 2):
        for index, values in enumerate(_vector_columns(table, f"platform{platform}_sigma_PN", 3)):
            axes[0].plot(time, values, color=_line_color(platform - 1),
                         linestyle=("-", "--", ":")[index],
                         label=f"platform {platform}" if index == 0 else None)
    axes[0].set_ylabel("Inertial attitude (MRP)")
    axes[0].legend(loc="lower right")
    axes[1].plot(time, table.column("platform1_r_PM_M_m_0"), color=_line_color(0))
    axes[1].set_ylabel("Platform 1 translation (m)")
    axes[1].set_xlabel("Time (s)")
    return {f"{STEM}1": figure}


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, ("",), build))
