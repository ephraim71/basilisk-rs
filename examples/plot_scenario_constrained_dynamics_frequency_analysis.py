#!/usr/bin/env python3
"""Upstream `scenarioConstrainedDynamicsFrequencyAnalysis` figures from the
per-gain CSVs `scenarioConstrainedDynamicsFrequencyAnalysis_<model>_alpha<gain>.csv`:
the panel angle error against the truth model, and the spectrum of the
attitude violation."""

import argparse
import sys

import numpy as np

from multibody_plotting import sub_mrp
from orbit_plotting import NANO2SEC, Table, line_color, run_plots

STEM = "scenarioConstrainedDynamicsFrequencyAnalysis"
GAINS = ("1e1", "1e2", "1e3", "1e4", "1e5")


def build_for(model):
    def build(generated_dir, plt):
        runs = []
        for gain in GAINS:
            path = generated_dir / f"{STEM}_{model}_alpha{gain}.csv"
            if path.is_file():
                runs.append((gain, Table(path)))
        if not runs:
            return {}
        count = len(runs)

        errors = plt.figure()
        axis = errors.gca()
        for index, (gain, table) in enumerate(runs):
            error = np.abs(table["theta1_rad"] - table["theta3_rad"])
            axis.semilogy(table.time_ns * NANO2SEC, np.degrees(error),
                          color=line_color(index, count, plt), label=rf"$\alpha=${gain}")
        axis.legend(loc="best")
        axis.set_xlabel("Time [sec]")
        axis.set_ylabel(r"Angle Error $\Delta \theta$ [deg]")

        spectrum = plt.figure()
        axis = spectrum.gca()
        for index, (gain, table) in enumerate(runs):
            s1, s2 = table.vectors("sigma_B1N"), table.vectors("sigma_B2N")
            signal = np.array([sub_mrp(s2[i], s1[i])[0] for i in range(len(s1))])
            data = np.abs(np.fft.fft(signal))
            n = len(data)
            sampling_rate = 1.0 / ((table.time_ns[1] - table.time_ns[0]) * NANO2SEC)
            period = n / sampling_rate
            one_sided = n // 2
            frequency = np.arange(one_sided) / period
            amplitude = data[:one_sided] / one_sided
            axis.loglog(frequency, amplitude, color=line_color(index, count, plt),
                        label=rf"$\alpha=${gain}")
            print(f"Frequency of highest amplitude for gain={gain} is "
                  f"{amplitude.argmax() / period} Hz.")
        axis.legend(loc="best")
        axis.set_xlabel("Freq [Hz]")
        axis.set_ylabel(r"$|X|$")
        axis.grid(which="both")
        return {STEM + "PanelAngleErrors": errors, STEM + "FFT": spectrum}

    return build


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--model", default="MEV2", choices=("bskSat", "MEV1", "MEV2"))
    known, rest = parser.parse_known_args()
    sys.argv = [sys.argv[0], *rest]
    raise SystemExit(run_plots("Plot the constrained dynamics frequency analysis.",
                               build_for(known.model)))
