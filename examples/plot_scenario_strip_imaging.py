#!/usr/bin/env python3
"""Figures of upstream scenarioStripImaging.py from the Rust CSV: the
principal rotation error, the view angle and the perpendicular-velocity norm
over the three strips, with upstream's pre-imaging (gray) and constraint
(green/red) shading."""
from __future__ import annotations

import bisect
import math
from typing import Any

from scenario_plotting import CsvTable, _time_minutes, _vector_rows, run_variant_plot

STEM = "scenarioStripImaging"
PRE_IMAGING_S = 120.0
IMAGING_S = 167.0
STRIP_COLORS = ("blue", "magenta", "orangered")
STRIP_LABELS = ("Parallel", "Perpendicular", "Diagonal")
MAX_PRINCIPAL_ROTATION_ERROR_DEG = 5.0
MINIMUM_ELEVATION_DEG = 10.0
P_HAT_B = (0.0, 0.0, 1.0)


def mrp_to_dcm(sigma):
    """Upstream `RigidBodyKinematics.MRP2C`."""
    s1, s2, s3 = sigma
    q = s1 * s1 + s2 * s2 + s3 * s3
    d = (1 + q) ** 2
    return (
        (4 * (s1 * s1 - s2 * s2 - s3 * s3) + (1 - q) ** 2, 8 * s1 * s2 + 4 * s3 * (1 - q),
         8 * s1 * s3 - 4 * s2 * (1 - q)),
        (8 * s2 * s1 - 4 * s3 * (1 - q), 4 * (-s1 * s1 + s2 * s2 - s3 * s3) + (1 - q) ** 2,
         8 * s2 * s3 + 4 * s1 * (1 - q)),
        (8 * s3 * s1 + 4 * s2 * (1 - q), 8 * s3 * s2 - 4 * s1 * (1 - q),
         4 * (-s1 * s1 - s2 * s2 + s3 * s3) + (1 - q) ** 2),
    ), d


def v_perp_norms(sigma_rn_rows, v_rows):
    """Upstream `compute_v_perp_norm`: `|pHat_B x v_TP_B|`."""
    norms = []
    for sigma, v in zip(sigma_rn_rows, v_rows):
        magnitude = math.sqrt(sum(c * c for c in v))
        if magnitude < 1e-12:
            norms.append(0.0)
            continue
        dcm, scale = mrp_to_dcm(sigma)
        v_b = [sum(row[k] * v[k] for k in range(3)) / scale / magnitude for row in dcm]
        size = math.sqrt(sum(c * c for c in v_b))
        if size > 1e-12:
            v_b = [c / size for c in v_b]
        p = P_HAT_B
        cross = (p[1] * v_b[2] - p[2] * v_b[1], p[2] * v_b[0] - p[0] * v_b[2],
                 p[0] * v_b[1] - p[1] * v_b[0])
        norms.append(math.sqrt(sum(c * c for c in cross)))
    return norms


def shading(axis, imaging_regions, threshold, green_below):
    """Upstream `_add_threshold_shading`."""
    y_min, y_max = axis.get_ylim()
    if y_max - y_min < 1e-12:
        return
    for t0, t1 in imaging_regions:
        below, above = ("green", "red") if green_below else ("red", "green")
        if y_max <= threshold:
            axis.axvspan(t0, t1, color=below, alpha=0.14, zorder=0)
        elif y_min >= threshold:
            axis.axvspan(t0, t1, color=above, alpha=0.14, zorder=0)
        else:
            fraction = (threshold - y_min) / (y_max - y_min)
            axis.axvspan(t0, t1, ymin=0, ymax=fraction, color=below, alpha=0.14, zorder=0)
            axis.axvspan(t0, t1, ymin=fraction, ymax=1, color=above, alpha=0.14, zorder=0)


def segments(axis, time, data, seg_indices, gray_indices):
    """Upstream `_plot_segments`."""
    for k, (i0, i1) in enumerate(gray_indices):
        axis.plot(time[i0:i1 + 1], data[i0:i1 + 1], color=STRIP_COLORS[k], ls="--", lw=1.0)
    for k, (i0, i1) in enumerate(seg_indices):
        axis.plot(time[i0:i1 + 1], data[i0:i1 + 1], color=STRIP_COLORS[k],
                  label=STRIP_LABELS[k], lw=1.2)


def build(plt: Any, table: CsvTable, variant: str) -> dict[str, Any]:
    time = _time_minutes(table)
    times_s = [value * 1e-9 for value in table.column("time_ns")]
    phase = PRE_IMAGING_S + IMAGING_S
    starts = [k * phase + PRE_IMAGING_S for k in range(3)]
    ends = [(k + 1) * phase for k in range(3)]
    gray = [(time[0], PRE_IMAGING_S / 60)] + [(ends[k] / 60, starts[k + 1] / 60) for k in range(2)]
    imaging = [(starts[k] / 60, ends[k] / 60) for k in range(3)]

    def index(t_s):
        return bisect.bisect_right(times_s, t_s + 1e-9) - 1

    gray_indices = [(0, index(starts[0]))] + [(index(ends[k]), index(starts[k + 1])) for k in range(2)]
    seg_indices = [(index(starts[k]), index(ends[k]) if k < 2 else len(times_s) - 1) for k in range(3)]

    figures = {}
    error = [math.degrees(4 * math.atan(math.sqrt(sum(c * c for c in row))))
             for row in _vector_rows(table, "sigma_BR")]
    elevation = [math.degrees(value) for value in table.column("elevation_rad")]
    v_perp = v_perp_norms(_vector_rows(table, "sigma_RN"), _vector_rows(table, "v_LP_N_m_per_s"))
    for number, data, ylabel, threshold, green_below, floor in (
        (1, error, "Principal Rotation Error [deg]", MAX_PRINCIPAL_ROTATION_ERROR_DEG, True, True),
        (2, elevation, "View Angle [deg]", MINIMUM_ELEVATION_DEG, False, False),
        (3, v_perp, r"$\|\mathbf{v}_{\perp}\|$", None, None, True),
    ):
        figure, axis = plt.subplots(figsize=(8, 4))
        segments(axis, time, data, seg_indices, gray_indices)
        axis.set_xlim(time[0], time[-1])
        if floor:
            axis.set_ylim(bottom=0.0)
        if threshold is None:
            axis.axhspan(0, 0.1, facecolor="red", alpha=0.2, hatch="//", edgecolor="red",
                         linewidth=0.0, zorder=0, label="Point Imaging Region")
        for t0, t1 in gray:
            axis.axvspan(t0, t1, color="gray", alpha=0.15, zorder=0)
        if threshold is not None:
            shading(axis, imaging, threshold, green_below)
        axis.set_xlabel("Time [min]", fontsize=12)
        axis.set_ylabel(ylabel, fontsize=12)
        axis.grid(True)
        axis.legend(fontsize=10)
        figures[f"{STEM}{number}"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_variant_plot(f"Plot {STEM} results.", STEM, ("",), build))
