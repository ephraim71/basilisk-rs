#!/usr/bin/env python3
"""Upstream `scenarioAttitudeFeedbackRWMuJoCo` figures from
`scenarioAttitudeFeedbackRWMuJoCo.csv`: the attitude and rate errors, and the
limited wheel motor torques with the wheel speeds."""

from orbit_plotting import NANO2MIN, existing, line_color, run_plots

RPM = 2.0 * 3.141592653589793 / 60.0
STEM = "scenarioAttitudeFeedbackRWMuJoCo"


def build(generated_dir, plt):
    figures = {}
    for stem, table in existing(generated_dir, [STEM]):
        minutes = table.time_ns * NANO2MIN
        figure, (axis_1, axis_2) = plt.subplots(1, 2, sharex=True)
        sigma = table.vectors("sigma_BR")
        omega = table.vectors("omega_BR_B_rad_per_s")
        for index in range(3):
            axis_1.plot(minutes, sigma[:, index], color=line_color(index, 3, plt),
                        label=rf"$\sigma_{index}$")
            axis_2.plot(minutes, omega[:, index], color=line_color(index, 3, plt),
                        label=rf"$\omega_{{BR,{index}}}$")
        axis_1.set_xlabel("Time [min]")
        axis_1.set_ylabel(r"Attitude Error $[\mathbf{\sigma}]$")
        axis_1.legend(loc="lower right")
        axis_2.set_xlabel("Time [min]")
        axis_2.set_ylabel("Rate Tracking Error [rad/s]")
        axis_2.legend(loc="lower right")
        figures[stem + "1"] = figure

        figure, (axis_1, axis_2) = plt.subplots(1, 2, sharex=True)
        for index in range(3):
            axis_1.plot(minutes, table[f"rwMotorTorque_Nm_{index}"], color=line_color(index, 3, plt),
                        label=rf"$\hat u_{{s,{index}}}$")
            axis_2.plot(minutes, table[f"rwOmega_rad_per_s_{index}"] / RPM,
                        color=line_color(index, 3, plt), label=rf"$\Omega_{index}$")
        axis_1.set_xlabel("Time [min]")
        axis_1.set_ylabel("Commanded RW Motor Torque [Nm]")
        axis_1.legend(loc="lower right")
        axis_2.set_xlabel("Time [min]")
        axis_2.set_ylabel("RW Speed [RPM]")
        axis_2.legend(loc="lower right")
        figures[stem + "2"] = figure
    return figures


if __name__ == "__main__":
    raise SystemExit(run_plots("Plot the MuJoCo reaction-wheel attitude scenario.", build))
