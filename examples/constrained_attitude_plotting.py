"""Figures shared by the plot scripts of upstream's
``scenarioAttitudeConstraintViolation.py`` and
``scenarioAttitudeConstrainedManeuver.py``, which draw the same six figures:
attitude error, wheel torques, rate error, wheel speeds, and the star
tracker's and sun sensors' Sun angles with the violations shaded.

A CSV variant is upstream's figure suffix ``<use2SunSensors><starTrackerFov>
<sunSensorFov><attitudeSetCase>``, e.g. ``120702``.
"""
from __future__ import annotations

import math
from typing import Any

from scenario_plotting import (
    RPM_RAD_PER_S,
    CsvTable,
    _line_color,
    _plot_vector_history,
    _time_minutes,
)


def fields_of_view(variant: str) -> tuple[int, int]:
    """The star tracker and sun sensor fields of view [deg] of a variant."""
    return int(variant[1:3]), int(variant[3:5])


def _shade(plt: Any, axis: Any, time: list[float], where: list[bool]) -> None:
    import matplotlib.transforms as mtransforms

    trans = mtransforms.blended_transform_factory(axis.transData, axis.transAxes)
    axis.fill_between(time, 0, 1, where=where, facecolor="red", alpha=0.4,
                      interpolate=True, transform=trans)


def build(plt: Any, table: CsvTable, stem: str, variant: str) -> dict[str, Any]:
    time = _time_minutes(table)
    star_tracker_fov, sun_sensor_fov = fields_of_view(variant)

    attitude = plt.figure()
    _plot_vector_history(attitude.gca(), time, table, "sigma_BR",
                         lambda index: rf"$\sigma_{index}$", r"Attitude Error $\sigma_{B/R}$")

    torque = plt.figure()
    axis = torque.gca()
    for index in range(3):
        axis.plot(time, table.column(f"motorTorque_Nm_{index}"), "--", color=_line_color(index),
                  label=rf"$\hat u_{{s,{index}}}$")
        axis.plot(time, table.column(f"rwTorque_Nm_{index}"), color=_line_color(index),
                  label=rf"$u_{{s,{index}}}$")
    axis.legend(loc="lower right")
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("RW Motor Torque (Nm)")

    rate = plt.figure()
    _plot_vector_history(rate.gca(), time, table, "omega_BR_B_rad_per_s",
                         lambda index: rf"$\omega_{{BR,{index}}}$", "Rate Tracking Error (rad/s) ")

    speeds = plt.figure()
    _plot_vector_history(speeds.gca(), time, table, "wheelSpeeds_rad_per_s",
                         lambda index: rf"$\Omega_{{{index}}}$", "RW Speed (RPM) ",
                         scale=1.0 / RPM_RAD_PER_S)

    star_tracker = plt.figure()
    axis = star_tracker.gca()
    angles = [math.degrees(value) for value in table.column("starTrackerMissAngle_rad")]
    axis.plot(time, [star_tracker_fov] * len(time), "--", color="r", label="f.o.v.")
    axis.plot(time, angles, color=_line_color(0), label=r"$\alpha $")
    _shade(plt, axis, time, [star_tracker_fov >= angle for angle in angles])
    axis.legend(loc="lower right")
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("StarTracker/Sun angle α (deg)")

    sun_sensor = plt.figure()
    axis = sun_sensor.gca()
    sensors = [name for name in ("sunSensorYMissAngle_rad", "sunSensorZMissAngle_rad")
               if name in table.headers]
    data = [[math.degrees(value) for value in table.column(name)] for name in sensors]
    axis.plot(time, [sun_sensor_fov] * len(time), "--", color="r", label="f.o.v.")
    for index, values in enumerate(data):
        axis.plot(time, values, color=_line_color(index), label=rf"$\beta_{{{index + 1}}}$")
    smallest = [min(values[sample] for values in data) for sample in range(len(time))]
    _shade(plt, axis, time, [sun_sensor_fov < angle for angle in smallest])
    axis.legend(loc="lower right")
    axis.set_xlabel("Time [min]")
    axis.set_ylabel("SunSensor/Sun angle β (deg)")

    figures = (attitude, torque, rate, speeds, star_tracker, sun_sensor)
    return {f"{stem}{number}{variant}": figure for number, figure in enumerate(figures, start=1)}
