"""Shared figure code of the small-body scenario plot scripts
(``plot_scenario_small_body_nav.py``, ``plot_scenario_small_body_nav_ukf.py``,
``plot_scenario_small_body_feedback_control.py``): upstream's three-row
component figures and the Hill-frame conversions their scripts apply to the
recorded states. CSV loading and the command line are ``orbit_plotting``'s.
"""

from __future__ import annotations

from typing import Any, Sequence

import numpy as np


def hill_frame(rc: np.ndarray, vc: np.ndarray) -> np.ndarray:
    """Upstream ``orbitalMotion.hillFrame``: rows ``i_r``, ``i_theta``,
    ``i_h``."""
    i_r = rc / np.linalg.norm(rc)
    h = np.cross(rc, vc)
    i_h = h / np.linalg.norm(h)
    return np.array([i_r, np.cross(i_h, i_r), i_h])


def relative_hill(rc: np.ndarray, vc: np.ndarray, rd: np.ndarray, vd: np.ndarray,
                  rotating: bool) -> tuple[np.ndarray, np.ndarray]:
    """Per row: ``rd - rc`` and ``vd - vc`` in the Hill frame of ``(rc, vc)``.

    ``rotating`` gives upstream ``orbitalMotion.rv2hill`` (the velocity seen
    in the rotating frame); otherwise the velocity difference is only rotated,
    as ``scenarioSmallBodyNav`` does with ``dcm_ON``."""
    positions, velocities = [], []
    for rc_n, vc_n, rd_n, vd_n in zip(rc, vc, rd, vd, strict=True):
        dcm_on = hill_frame(rc_n, vc_n)
        rho = dcm_on @ (rd_n - rc_n)
        rho_dot = dcm_on @ (vd_n - vc_n)
        if rotating:
            f_dot = np.linalg.norm(np.cross(rc_n, vc_n)) / (rc_n @ rc_n)
            rho_dot = rho_dot - np.cross([0.0, 0.0, f_dot], rho)
        positions.append(rho)
        velocities.append(rho_dot)
    return np.array(positions), np.array(velocities)


def three_axes(plt: Any) -> tuple[Any, Any]:
    """Upstream's figure of three stacked axes sharing x, with a frameless
    overlay axis carrying the common labels."""
    figure, axes = plt.subplots(3, sharex=True, figsize=(12, 6))
    overlay = figure.add_subplot(111, frameon=False)
    overlay.tick_params(labelcolor="none", top=False, bottom=False, left=False, right=False)
    return figure, (axes, overlay)


def component_figure(plt: Any, series: Sequence[tuple[np.ndarray, np.ndarray, dict]],
                     ylabels: Sequence[str], xlabel: str, title: str | None = None) -> Any:
    """One row per component; each ``series`` entry is ``(time, values,
    style)`` with ``style`` the matplotlib keywords, its ``label`` shown on
    the first row only."""
    figure, (axes, overlay) = three_axes(plt)
    for series_time, values, style in series:
        for row in range(3):
            keywords = dict(style)
            if row > 0:
                keywords.pop("label", None)
            fmt = keywords.pop("fmt", None)
            arguments = (series_time, values[:, row]) + ((fmt,) if fmt else ())
            axes[row].plot(*arguments, **keywords)
    for row, label in enumerate(ylabels):
        axes[row].set_ylabel(label)
    overlay.set_xlabel(xlabel)
    if title:
        overlay.set_title(title)
    axes[0].legend()
    return figure


def error_figure(plt: Any, time: np.ndarray, error: np.ndarray, sigma: np.ndarray,
                 ylabels: Sequence[str], xlabel: str, title: str, color: str | None = None) -> Any:
    """Upstream's estimation error with its ``2 sigma`` bounds (``sigma``
    holds the one-sigma columns)."""
    figure, (axes, overlay) = three_axes(plt)
    for row in range(3):
        style = {"color": color} if color else {}
        axes[row].plot(time, error[:, row], label="error" if row == 0 else None, **style)
        axes[row].plot(time, 2 * sigma[:, row], "k--", label=r"$2\sigma$" if row == 0 else None)
        axes[row].plot(time, -2 * sigma[:, row], "k--")
        axes[row].set_ylabel(ylabels[row])
    overlay.set_xlabel(xlabel)
    overlay.set_title(title)
    axes[0].legend()
    return figure


def columns(table: Any, prefix: str, indices: Sequence[int]) -> np.ndarray:
    """The columns ``<prefix>_<i>`` for ``i`` in ``indices``, side by side."""
    return np.column_stack([table[f"{prefix}_{index}"] for index in indices])
