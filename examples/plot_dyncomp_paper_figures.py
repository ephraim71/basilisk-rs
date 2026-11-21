#!/usr/bin/env python3
"""Print-quality figures of the dynamics-engine comparison paper: port of
upstream `examples/dynamicsComparison/paperFigures.py`.

Re-renders, from the JSON summaries the Rust examples write to
`examples/output/scenarios`, the figures upstream places in the paper, sized
for two-column print with one color scheme (Paul Tol: BSM blue, MuJoCo red,
third series yellow):

* `scenarioCompareParetoRwPanels_frontier.pdf`, `..._frontierPosition.pdf`,
  `scenarioCompareParetoFlexPanels_frontier.pdf`, `..._frontierPosition.pdf`:
  the work-precision panels, marker per integrator, with the reference floor;
* `scenarioCompareFlexPanels_runtime.pdf`: wall-clock against degrees of
  freedom for the three implementations, with the BSM/MuJoCo ratios;
* `sweepOrbitDt.pdf`: the analytic and cross-engine orbit errors against
  the step, with a `dt^4` guide;
* `sweepTorqueArtifact.pdf`: the torque artifact against the step and the
  drift speed.

`just dyncomp-paper-figures` regenerates every input (the three scenarios
and two sweeps at their defaults, MuJoCo feature) and renders the PDFs into
`examples/output/scenarios/paper`. Upstream also records a provenance
manifest (source, native-build, dependency and host fingerprints, a
clean-build receipt) and refuses to render from changed inputs; of that,
this script keeps the hash manifest: it writes `paper/provenance.json` with
the SHA-256 of every input and output, and `--verify` checks the inputs
against an existing manifest before rendering and the outputs after.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

from orbit_plotting import DEFAULT_GENERATED_DIR

PAPER_INPUT_FILES = (
    "scenarioCompareParetoRwPanels.json",
    "scenarioCompareParetoFlexPanels.json",
    "scenarioCompareFlexPanels.json",
    "sweepOrbitDt.json",
    "sweepTorqueArtifact.json",
)
PAPER_OUTPUT_FILES = (
    "scenarioCompareParetoRwPanels_frontier.pdf",
    "scenarioCompareParetoRwPanels_frontierPosition.pdf",
    "scenarioCompareParetoFlexPanels_frontier.pdf",
    "scenarioCompareParetoFlexPanels_frontierPosition.pdf",
    "scenarioCompareFlexPanels_runtime.pdf",
    "sweepOrbitDt.pdf",
    "sweepTorqueArtifact.pdf",
)
PDF_METADATA = {"Creator": "basilisk-rs dynamics-engine comparison", "CreationDate": None, "ModDate": None}

BLUE = "#004488"    # BSM
RED = "#BB5566"     # MuJoCo
YELLOW = "#DDAA33"  # third series
GRAY = "#666666"

STYLE = {
    "font.size": 9,
    "axes.labelsize": 9,
    "axes.titlesize": 9,
    "legend.fontsize": 7.0,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "lines.linewidth": 1.3,
    "lines.markersize": 4.0,
    "legend.framealpha": 0.9,
    "figure.constrained_layout.use": True,
}

# Upstream INTEGRATOR_MARKERS, shared by both Pareto panels.
INTEGRATOR_MARKERS = (("Euler", "v"), ("RK2", "s"), ("RK4", "o"), ("RKF45", "D"), ("RKF78", "^"))


def load(results: Path, name: str) -> dict:
    with (results / name).open(encoding="utf-8") as handle:
        return json.load(handle)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def marker_for(integrator: str) -> str:
    """Upstream `markerFor`: RKF45 before RK4."""
    for key, marker in reversed(INTEGRATOR_MARKERS):
        if key in integrator:
            return marker
    return "x"


def pareto_panel(plt, data: dict, error_key: str, label: str, floor_key: str, output: Path) -> None:
    from matplotlib.lines import Line2D

    figure, axis = plt.subplots(figsize=(3.5, 2.7))
    for engine, color in (("bsm", BLUE), ("mujoco", RED)):
        families: dict[str, list] = {}
        for row in data[engine]:
            families.setdefault(row["integrator"], []).append(row)
        for integrator, rows in families.items():
            rows.sort(key=lambda row: row["wall"])
            adaptive = "RKF" in integrator
            axis.loglog([row["wall"] for row in rows], [max(row[error_key], 1e-18) for row in rows],
                        linestyle="-" if adaptive else "--", marker=marker_for(integrator), color=color,
                        markerfacecolor=color if adaptive else "white", markeredgecolor=color, linewidth=1.0)
    floor = data[floor_key]
    if floor > 0.0:
        axis.axhline(floor, color=GRAY, linestyle=":", linewidth=1.0)
        axis.text(0.98, floor * 1.6, "reference floor", color=GRAY, fontsize=6.5, ha="right",
                  transform=axis.get_yaxis_transform())
    handles = [Line2D([], [], color=BLUE, label="BSM"), Line2D([], [], color=RED, label="MuJoCo")]
    present = {row["integrator"] for row in data["bsm"]}
    for key, marker in INTEGRATOR_MARKERS:
        if any(key in name for name in present):
            handles.append(Line2D([], [], color=GRAY, linestyle="none", marker=marker, label=key))
    axis.legend(handles=handles, ncol=2, loc="best", handlelength=1.4, columnspacing=0.8, borderpad=0.4)
    axis.set_xlabel("wall-clock time [s]")
    axis.set_ylabel(label)
    axis.grid(True, which="both", alpha=0.2)
    figure.savefig(output, metadata=PDF_METADATA)
    plt.close(figure)


def pareto_figures(plt, results: Path, output_dir: Path) -> None:
    for stem in ("scenarioCompareParetoRwPanels", "scenarioCompareParetoFlexPanels"):
        data = load(results, stem + ".json")
        pareto_panel(plt, data, "error", "final attitude error [rad]", "sharedReferenceThreshold",
                     output_dir / f"{stem}_frontier.pdf")
        pareto_panel(plt, data, "positionError", "final position error [m]", "sharedPositionReferenceThreshold",
                     output_dir / f"{stem}_frontierPosition.pdf")


def scaling_figure(plt, results: Path, output_dir: Path) -> None:
    rows = load(results, "scenarioCompareFlexPanels.json")["rows"]
    dof = np.array([row["dof"] for row in rows])
    bsm = np.array([row["bsmWall"] for row in rows])
    ndof = np.array([row["ndofWall"] for row in rows])
    figure, axis = plt.subplots(figsize=(4.8, 3.3))
    axis.loglog(dof, bsm, "o-", color=BLUE, label="BSM nHingedRigidBody")
    axis.loglog(dof, ndof, "^-", color=YELLOW, label="BSM spinningBodyNDOF")
    if all("mujocoWall" in row for row in rows):
        mujoco = np.array([row["mujocoWall"] for row in rows])
        axis.loglog(dof, mujoco, "s-", color=RED, label="MuJoCo")
        for x, y_bsm, y_mujoco in zip(dof, bsm, mujoco):
            axis.text(x, np.sqrt(y_bsm * y_mujoco), f"{y_bsm / y_mujoco:.1f}$\\times$", fontsize=7, color=GRAY,
                      ha="center", va="center")
    axis.set_xlabel("system degrees of freedom")
    axis.set_ylabel("wall-clock for 600 steps [s]")
    axis.legend(loc="upper left")
    axis.grid(True, which="both", alpha=0.2)
    figure.savefig(output_dir / "scenarioCompareFlexPanels_runtime.pdf", metadata=PDF_METADATA)
    plt.close(figure)


def orbit_dt_figure(plt, results: Path, output_dir: Path) -> None:
    rows = load(results, "sweepOrbitDt.json")["rows"]
    dts = np.array([row["dt"] for row in rows])
    analytic = np.array([row["bsmVsAnalyticMax"] for row in rows])
    cross = np.array([row["crossParadigmPosMax"] for row in rows])
    figure, axis = plt.subplots(figsize=(4.2, 3.1))
    axis.loglog(dts, analytic, "o-", color=BLUE, label="engine vs analytic Kepler")
    positive = cross > 0.0
    if positive.any():
        axis.loglog(dts[positive], cross[positive], "s-", color=RED, label="BSM vs MuJoCo")
        axis.set_ylim(bottom=cross[positive].min() / 60.0)
    reference = np.array([dts.max(), dts.min()])
    axis.loglog(reference, analytic[0] * (reference / dts.max()) ** 4, ":", color=GRAY, label=r"$\propto \Delta t^4$")
    axis.set_xlabel(r"integrator step $\Delta t$ [s]")
    axis.set_ylabel("max position difference [m]")
    axis.legend(loc="lower right")
    axis.grid(True, which="both", alpha=0.2)
    figure.savefig(output_dir / "sweepOrbitDt.pdf", metadata=PDF_METADATA)
    plt.close(figure)


def torque_artifact_figure(plt, results: Path, output_dir: Path) -> None:
    data = load(results, "sweepTorqueArtifact.json")
    figure, (left, right) = plt.subplots(1, 2, figsize=(7.0, 2.9))
    dts = [row["dt"] for row in data["dtSweep"]]
    rest = [row["crossEngineRestAttMax"] for row in data["dtSweep"]]
    floor = 1e-18
    left.loglog(dts, [max(row["mujocoMotionAttMax"], floor) for row in data["dtSweep"]], "o-", color=RED,
                label="MuJoCo orbiting-vs-rest")
    left.loglog(dts, rest, "s-", color=YELLOW, label="cross-engine, at rest")
    left.loglog(dts, [max(row["bsmMotionAttMax"], floor) for row in data["dtSweep"]], "^-", color=BLUE,
                label="BSM orbiting-vs-rest")
    reference = np.array([max(dts), min(dts)])
    left.loglog(reference, rest[0] * (reference / max(dts)) ** 4, ":", color=GRAY, label=r"$\propto \Delta t^4$")
    left.set_xlabel(r"integrator step $\Delta t$ [s]")
    left.set_ylabel("max principal angle [rad]")
    left.legend(loc="lower right")
    left.grid(True, which="both", alpha=0.2)
    speeds = np.array([row["speed"] for row in data["velocitySweep"]]) / 1000.0
    right.semilogy(speeds, [max(row["mujocoDriftAttMax"], floor) for row in data["velocitySweep"]], "o-",
                   color=RED, label="MuJoCo drifting-vs-rest")
    right.semilogy(speeds, [max(row["bsmDriftAttMax"], floor) for row in data["velocitySweep"]], "^-",
                   color=BLUE, label="BSM drifting-vs-rest")
    right.set_xlabel("inertial drift speed [km/s]")
    right.set_ylabel("max principal angle [rad]")
    right.legend(loc="center right")
    right.grid(True, which="both", alpha=0.2)
    figure.savefig(output_dir / "sweepTorqueArtifact.pdf", metadata=PDF_METADATA)
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--generated-dir", type=Path, default=DEFAULT_GENERATED_DIR,
                        help="directory of the JSON summaries")
    parser.add_argument("--output-dir", type=Path, help="PDF directory (default: <generated-dir>/paper)")
    parser.add_argument("--verify", action="store_true",
                        help="check the inputs and outputs against an existing provenance.json")
    arguments = parser.parse_args()
    results = arguments.generated_dir
    output_dir = arguments.output_dir or results / "paper"
    missing = [name for name in PAPER_INPUT_FILES if not (results / name).is_file()]
    if missing:
        print(f"error: missing inputs in {results}: {', '.join(missing)} (run `just dyncomp-paper-figures`)",
              file=sys.stderr)
        return 1
    manifest_path = output_dir / "provenance.json"
    if arguments.verify:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        for name in PAPER_INPUT_FILES:
            if manifest["inputs"].get(name) != sha256(results / name):
                print(f"error: input hash mismatch for {name}", file=sys.stderr)
                return 1

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    matplotlib.rcParams.update(STYLE)
    output_dir.mkdir(parents=True, exist_ok=True)
    pareto_figures(plt, results, output_dir)
    scaling_figure(plt, results, output_dir)
    orbit_dt_figure(plt, results, output_dir)
    torque_artifact_figure(plt, results, output_dir)
    outputs = {name: sha256(output_dir / name) for name in PAPER_OUTPUT_FILES}
    if arguments.verify:
        for name, digest in outputs.items():
            if manifest["outputs"].get(name) != digest:
                print(f"error: output hash mismatch for {name}", file=sys.stderr)
                return 1
        print("paper figures match", manifest_path)
    else:
        manifest = {"inputs": {name: sha256(results / name) for name in PAPER_INPUT_FILES}, "outputs": outputs}
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        print(f"wrote {manifest_path}")
    print(f"Paper figures written to {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
