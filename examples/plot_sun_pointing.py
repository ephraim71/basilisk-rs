from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt


SUN_HAT_INERTIAL = (1.0, 0.0, 0.0)


def load_csv(path: Path) -> list[dict[str, float]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        rows: list[dict[str, float]] = []
        for row in reader:
            rows.append({key: float(value) for key, value in row.items()})
        return rows


def rotate_body_vector_by_quaternion(row: dict[str, float], vector: tuple[float, float, float]) -> tuple[float, float, float]:
    x = row["attitude_b_to_i.x"]
    y = row["attitude_b_to_i.y"]
    z = row["attitude_b_to_i.z"]
    w = row["attitude_b_to_i.w"]
    vx, vy, vz = vector

    r11 = 1.0 - 2.0 * (y * y + z * z)
    r12 = 2.0 * (x * y - z * w)
    r13 = 2.0 * (x * z + y * w)
    r21 = 2.0 * (x * y + z * w)
    r22 = 1.0 - 2.0 * (x * x + z * z)
    r23 = 2.0 * (y * z - x * w)
    r31 = 2.0 * (x * z - y * w)
    r32 = 2.0 * (y * z + x * w)
    r33 = 1.0 - 2.0 * (x * x + y * y)

    return (
        r11 * vx + r12 * vy + r13 * vz,
        r21 * vx + r22 * vy + r23 * vz,
        r31 * vx + r32 * vy + r33 * vz,
    )


def pointing_error_deg(row: dict[str, float]) -> float:
    body_z_inertial = rotate_body_vector_by_quaternion(row, (0.0, 0.0, 1.0))
    dot_product = sum(a * b for a, b in zip(body_z_inertial, SUN_HAT_INERTIAL))
    return math.degrees(math.acos(max(-1.0, min(1.0, dot_product))))


def main() -> None:
    output_dir = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path("examples/output/sun_pointing")
    )

    spacecraft_rows = load_csv(output_dir / "spacecraft_state.csv")
    imu_rows = load_csv(output_dir / "imu.csv")
    sunline_rows = load_csv(output_dir / "sunline.csv")
    guidance_rows = load_csv(output_dir / "guidance.csv")
    body_torque_rows = load_csv(output_dir / "body_torque.csv")
    rw_x_rows = load_csv(output_dir / "rw_x_cmd.csv")
    rw_y_rows = load_csv(output_dir / "rw_y_cmd.csv")

    times = [row["sim_time_s"] for row in spacecraft_rows]
    pointing_error = [pointing_error_deg(row) for row in spacecraft_rows]

    figure, axes = plt.subplots(2, 2, figsize=(14, 10), sharex=True)

    axes[0, 0].plot(times, pointing_error, label="pointing error")
    axes[0, 0].set_title("Sun Pointing Error")
    axes[0, 0].set_ylabel("deg")
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()

    axes[0, 1].plot(times, [row["omega_radps.x"] for row in spacecraft_rows], label="omega x")
    axes[0, 1].plot(times, [row["omega_radps.y"] for row in spacecraft_rows], label="omega y")
    axes[0, 1].plot(times, [row["omega_radps.z"] for row in spacecraft_rows], label="omega z")
    axes[0, 1].plot([row["sim_time_s"] for row in imu_rows], [row["angular_rate_sensor_radps.x"] for row in imu_rows], "--", label="imu x")
    axes[0, 1].plot([row["sim_time_s"] for row in imu_rows], [row["angular_rate_sensor_radps.y"] for row in imu_rows], "--", label="imu y")
    axes[0, 1].plot([row["sim_time_s"] for row in imu_rows], [row["angular_rate_sensor_radps.z"] for row in imu_rows], "--", label="imu z")
    axes[0, 1].set_title("Body Rates")
    axes[0, 1].set_ylabel("rad/s")
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend(ncol=2, fontsize=8)

    axes[1, 0].plot([row["sim_time_s"] for row in sunline_rows], [row["sun_vector_body.x"] for row in sunline_rows], label="sunline x")
    axes[1, 0].plot([row["sim_time_s"] for row in sunline_rows], [row["sun_vector_body.y"] for row in sunline_rows], label="sunline y")
    axes[1, 0].plot([row["sim_time_s"] for row in sunline_rows], [row["sun_vector_body.z"] for row in sunline_rows], label="sunline z")
    axes[1, 0].plot([row["sim_time_s"] for row in guidance_rows], [row["sigma_br.x"] for row in guidance_rows], "--", label="sigma x")
    axes[1, 0].plot([row["sim_time_s"] for row in guidance_rows], [row["sigma_br.y"] for row in guidance_rows], "--", label="sigma y")
    axes[1, 0].plot([row["sim_time_s"] for row in guidance_rows], [row["sigma_br.z"] for row in guidance_rows], "--", label="sigma z")
    axes[1, 0].set_title("Estimated Sunline and Guidance Error")
    axes[1, 0].set_xlabel("sim time [s]")
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend(ncol=2, fontsize=8)

    axes[1, 1].plot([row["sim_time_s"] for row in body_torque_rows], [row["torque_request_body_nm.x"] for row in body_torque_rows], label="body torque x")
    axes[1, 1].plot([row["sim_time_s"] for row in body_torque_rows], [row["torque_request_body_nm.y"] for row in body_torque_rows], label="body torque y")
    axes[1, 1].plot([row["sim_time_s"] for row in rw_x_rows], [row["motor_torque_nm"] for row in rw_x_rows], "--", label="rw x cmd")
    axes[1, 1].plot([row["sim_time_s"] for row in rw_y_rows], [row["motor_torque_nm"] for row in rw_y_rows], "--", label="rw y cmd")
    axes[1, 1].set_title("Control Torque and Wheel Commands")
    axes[1, 1].set_xlabel("sim time [s]")
    axes[1, 1].set_ylabel("Nm")
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend(fontsize=8)

    print(f"final pointing error [deg]: {pointing_error[-1]:.6f}")
    plot_path = output_dir / "sun_pointing_summary.png"
    figure.savefig(plot_path, dpi=150)
    print(f"saved plot: {plot_path}")

    figure.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
