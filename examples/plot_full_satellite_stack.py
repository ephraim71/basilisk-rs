from __future__ import annotations

import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt


def load_csv(path: Path) -> list[dict[str, float]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        rows: list[dict[str, float]] = []
        for row in reader:
            rows.append({key: float(value) for key, value in row.items()})
        return rows


def norm3(row: dict[str, float], prefix: str) -> float:
    return (
        row[f"{prefix}.x"] ** 2
        + row[f"{prefix}.y"] ** 2
        + row[f"{prefix}.z"] ** 2
    ) ** 0.5


def main() -> None:
    output_dir = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path("examples/output/full_satellite_stack")
    )

    spacecraft_rows = load_csv(output_dir / "spacecraft_state.csv")
    imu_1_rows = load_csv(output_dir / "imu_1.csv")
    imu_2_rows = load_csv(output_dir / "imu_2.csv")
    tam_1_rows = load_csv(output_dir / "tam_1.csv")
    tam_2_rows = load_csv(output_dir / "tam_2.csv")
    gps_rows = load_csv(output_dir / "gps.csv")
    sun_sensor_rows = {
        "sun_sensor_px": load_csv(output_dir / "sun_sensor_px.csv"),
        "sun_sensor_mx": load_csv(output_dir / "sun_sensor_mx.csv"),
        "sun_sensor_py": load_csv(output_dir / "sun_sensor_py.csv"),
        "sun_sensor_my": load_csv(output_dir / "sun_sensor_my.csv"),
        "sun_sensor_pz": load_csv(output_dir / "sun_sensor_pz.csv"),
        "sun_sensor_mz": load_csv(output_dir / "sun_sensor_mz.csv"),
    }

    spacecraft_time = [row["sim_time_s"] for row in spacecraft_rows]
    imu_time = [row["sim_time_s"] for row in imu_1_rows]
    tam_time = [row["sim_time_s"] for row in tam_1_rows]
    gps_time = [row["sim_time_s"] for row in gps_rows]

    figure, axes = plt.subplots(4, 2, figsize=(16, 14), sharex="col")

    axes[0, 0].plot(spacecraft_time, [row["omega_radps.x"] for row in spacecraft_rows], label="omega x")
    axes[0, 0].plot(spacecraft_time, [row["omega_radps.y"] for row in spacecraft_rows], label="omega y")
    axes[0, 0].plot(spacecraft_time, [row["omega_radps.z"] for row in spacecraft_rows], label="omega z")
    axes[0, 0].set_title("Spacecraft Body Rates")
    axes[0, 0].set_ylabel("rad/s")
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()

    axes[0, 1].plot(imu_time, [row["angular_rate_sensor_radps.x"] for row in imu_1_rows], label="imu_1 x")
    axes[0, 1].plot(imu_time, [row["angular_rate_sensor_radps.y"] for row in imu_1_rows], label="imu_1 y")
    axes[0, 1].plot(imu_time, [row["angular_rate_sensor_radps.z"] for row in imu_1_rows], label="imu_1 z")
    axes[0, 1].plot(imu_time, [row["angular_rate_sensor_radps.x"] for row in imu_2_rows], "--", label="imu_2 x")
    axes[0, 1].plot(imu_time, [row["angular_rate_sensor_radps.y"] for row in imu_2_rows], "--", label="imu_2 y")
    axes[0, 1].plot(imu_time, [row["angular_rate_sensor_radps.z"] for row in imu_2_rows], "--", label="imu_2 z")
    axes[0, 1].set_title("IMU Rates")
    axes[0, 1].set_ylabel("rad/s")
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend(ncol=2, fontsize=8)

    axes[1, 0].plot(tam_time, [norm3(row, "magnetic_field_sensor_t") for row in tam_1_rows], label="tam_1 |B|")
    axes[1, 0].plot(tam_time, [norm3(row, "magnetic_field_sensor_t") for row in tam_2_rows], label="tam_2 |B|")
    axes[1, 0].set_title("TAM Field Magnitude")
    axes[1, 0].set_ylabel("tesla")
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()

    for name, rows in sun_sensor_rows.items():
        axes[1, 1].plot(
            [row["sim_time_s"] for row in rows],
            [row["alpha_rad"] for row in rows],
            label=name,
        )
    axes[1, 1].set_title("Sun Sensor Alpha")
    axes[1, 1].set_ylabel("rad")
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend(ncol=2, fontsize=8)

    for name, rows in sun_sensor_rows.items():
        axes[2, 0].plot(
            [row["sim_time_s"] for row in rows],
            [row["beta_rad"] for row in rows],
            label=name,
        )
    axes[2, 0].set_title("Sun Sensor Beta")
    axes[2, 0].set_ylabel("rad")
    axes[2, 0].grid(True, alpha=0.3)
    axes[2, 0].legend(ncol=2, fontsize=8)

    for name, rows in sun_sensor_rows.items():
        axes[2, 1].plot(
            [row["sim_time_s"] for row in rows],
            [row["valid"] for row in rows],
            label=name,
        )
    axes[2, 1].set_title("Sun Sensor Valid")
    axes[2, 1].set_ylabel("0/1")
    axes[2, 1].grid(True, alpha=0.3)
    axes[2, 1].legend(ncol=2, fontsize=8)

    axes[3, 0].plot(gps_time, [row["position_m.x"] for row in gps_rows], label="gps x")
    axes[3, 0].plot(gps_time, [row["position_m.y"] for row in gps_rows], label="gps y")
    axes[3, 0].plot(gps_time, [row["position_m.z"] for row in gps_rows], label="gps z")
    axes[3, 0].set_title("GPS Position")
    axes[3, 0].set_xlabel("sim time [s]")
    axes[3, 0].set_ylabel("m")
    axes[3, 0].grid(True, alpha=0.3)
    axes[3, 0].legend()

    axes[3, 1].plot(gps_time, [row["gps_week"] for row in gps_rows], label="gps week")
    axes[3, 1].plot(gps_time, [row["gps_seconds_of_week"] for row in gps_rows], label="gps sow [s]")
    axes[3, 1].set_title("GPS Time")
    axes[3, 1].set_xlabel("sim time [s]")
    axes[3, 1].grid(True, alpha=0.3)
    axes[3, 1].legend()

    figure.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
