from __future__ import annotations

import csv
import io
import math
import os
import zipfile
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from pathlib import Path, PurePosixPath
from typing import TextIO


DEFAULT_REFERENCE_ZIP = Path(
    os.environ.get("BASILISK_REFERENCE_CSV_ZIP", "/home/ephraim/hamming/csvs.zip")
)
DEFAULT_GENERATED_DIR = Path("examples/output/scenarios")


class CsvToolError(RuntimeError):
    """A malformed, missing, or ambiguous scenario CSV input."""


@dataclass(frozen=True)
class ToleranceGroup:
    name: str
    column_prefixes: tuple[str, ...]
    absolute: float
    relative: float

    def matches(self, column: str) -> bool:
        return column.startswith(self.column_prefixes)


@dataclass(frozen=True)
class InactiveArray:
    column_prefix: str
    first_inactive_index: int

    def matches(self, column: str) -> bool:
        if not column.startswith(self.column_prefix):
            return False
        suffix = column.removeprefix(self.column_prefix)
        return suffix.isdigit() and int(suffix) >= self.first_inactive_index


@dataclass(frozen=True)
class ScenarioSpec:
    key: str
    filename: str
    rows: int
    columns: int
    sample_period_ns: int
    tolerance_groups: tuple[ToleranceGroup, ...]
    inactive_arrays: tuple[InactiveArray, ...] = ()

    def tolerance_for(self, column: str) -> ToleranceGroup | None:
        return next(
            (group for group in self.tolerance_groups if group.matches(column)), None
        )

    def is_inactive(self, column: str) -> bool:
        return any(rule.matches(column) for rule in self.inactive_arrays)


SCENARIOS: dict[str, ScenarioSpec] = {
    "basic-orbit": ScenarioSpec(
        key="basic-orbit",
        filename="scenarioBasicOrbitLEO0Earth.csv",
        rows=110,
        columns=7,
        sample_period_ns=40_000_000_000,
        tolerance_groups=(
            ToleranceGroup("position", ("r_BN_N_m_",), 1.0e-5, 1.0e-12),
            ToleranceGroup("velocity", ("v_BN_N_m_per_s_",), 1.0e-8, 1.0e-12),
        ),
    ),
    "css": ScenarioSpec(
        key="css",
        filename="scenarioCSS0010.csv",
        rows=301,
        columns=4,
        sample_period_ns=1_000_000_000,
        tolerance_groups=(
            ToleranceGroup(
                "css signal",
                ("css1Signal", "css2Signal", "css3Signal"),
                5.0e-10,
                1.0e-10,
            ),
        ),
    ),
    "drag-deorbit": ScenarioSpec(
        key="drag-deorbit",
        filename="scenarioDragDeorbitmsis0.csv",
        rows=49,
        columns=11,
        sample_period_ns=45_000_000_000,
        tolerance_groups=(
            ToleranceGroup("position", ("r_BN_N_m_",), 1.0e-4, 1.0e-12),
            ToleranceGroup("velocity", ("v_BN_N_m_per_s_",), 1.0e-6, 1.0e-12),
            ToleranceGroup("drag force", ("forceExternal_B_N_",), 2.0e-4, 1.0e-3),
            ToleranceGroup("density", ("neutralDensity_kg_per_m3",), 1.0e-18, 1.0e-9),
        ),
    ),
    "attitude-feedback-rw": ScenarioSpec(
        key="attitude-feedback-rw",
        filename="scenarioAttitudeFeedbackRW01.csv",
        rows=101,
        columns=121,
        sample_period_ns=6_000_000_000,
        tolerance_groups=(
            ToleranceGroup("requested torque", ("motorTorque_Nm_",), 1.0e-10, 1.0e-9),
            ToleranceGroup("attitude error", ("sigma_BR_",), 1.0e-10, 1.0e-9),
            ToleranceGroup("rate error", ("omega_BR_B_rad_per_s_",), 1.0e-11, 1.0e-9),
            ToleranceGroup("position", ("r_BN_N_m_",), 1.0e-5, 1.0e-12),
            ToleranceGroup("wheel speed", ("wheelSpeeds_rad_per_s_",), 1.0e-8, 1.0e-10),
            ToleranceGroup("actual torque", ("rwTorque_Nm_",), 1.0e-10, 1.0e-9),
            ToleranceGroup("wheel voltage", ("rwVoltage_V_",), 1.0e-9, 1.0e-9),
        ),
        inactive_arrays=(
            InactiveArray("motorTorque_Nm_", 3),
            InactiveArray("wheelSpeeds_rad_per_s_", 3),
            InactiveArray("rwVoltage_V_", 3),
        ),
    ),
    "mtb-momentum-management": ScenarioSpec(
        key="mtb-momentum-management",
        filename="scenarioMtbMomentumManagement.csv",
        rows=201,
        columns=128,
        sample_period_ns=36_000_000_000,
        tolerance_groups=(
            ToleranceGroup("requested torque", ("motorTorque_Nm_",), 1.0e-9, 1.0e-8),
            ToleranceGroup("attitude error", ("sigma_BR_",), 1.0e-8, 1.0e-8),
            ToleranceGroup("rate error", ("omega_BR_B_rad_per_s_",), 1.0e-9, 1.0e-8),
            ToleranceGroup("wheel speed", ("wheelSpeeds_rad_per_s_",), 2.0e-3, 1.0e-5),
            ToleranceGroup("actual torque", ("rwTorque_Nm_",), 5.0e-9, 1.0e-5),
            ToleranceGroup(
                "magnetic field",
                ("magField_N_T_", "tam_S_T_", "tam_B_T_"),
                1.0e-12,
                1.0e-8,
            ),
            ToleranceGroup("dipole command", ("mtbDipoleCmds_A_m2_",), 1.0e-3, 1.0e-4),
        ),
        inactive_arrays=(
            InactiveArray("motorTorque_Nm_", 4),
            InactiveArray("wheelSpeeds_rad_per_s_", 4),
            InactiveArray("mtbDipoleCmds_A_m2_", 4),
        ),
    ),
}


@dataclass(frozen=True)
class CsvTable:
    headers: tuple[str, ...]
    rows: tuple[tuple[float, ...], ...]
    timestamps: tuple[Decimal, ...]

    def column(self, name: str) -> list[float]:
        try:
            index = self.headers.index(name)
        except ValueError as error:
            raise CsvToolError(f"CSV has no column named {name!r}") from error
        return [row[index] for row in self.rows]


def _read_table(handle: TextIO, source: str) -> CsvTable:
    reader = csv.reader(handle)
    try:
        raw_headers = next(reader)
    except StopIteration as error:
        raise CsvToolError(f"{source}: CSV is empty") from error

    headers = tuple(header.strip() for header in raw_headers)
    if not headers or headers[0] != "time_ns":
        raise CsvToolError(f"{source}: first column must be 'time_ns'")
    if len(headers) != len(set(headers)):
        raise CsvToolError(f"{source}: CSV contains duplicate headers")

    rows: list[tuple[float, ...]] = []
    timestamps: list[Decimal] = []
    for line_number, raw_row in enumerate(reader, start=2):
        if not raw_row or all(not value.strip() for value in raw_row):
            continue
        if len(raw_row) != len(headers):
            raise CsvToolError(
                f"{source}:{line_number}: expected {len(headers)} fields, found {len(raw_row)}"
            )
        try:
            row = tuple(float(value) for value in raw_row)
            timestamp = Decimal(raw_row[0])
        except (ValueError, InvalidOperation) as error:
            raise CsvToolError(
                f"{source}:{line_number}: invalid numeric value"
            ) from error
        rows.append(row)
        timestamps.append(timestamp)

    if not rows:
        raise CsvToolError(f"{source}: CSV has a header but no data rows")
    return CsvTable(headers=headers, rows=tuple(rows), timestamps=tuple(timestamps))


def load_csv_path(path: Path) -> CsvTable:
    try:
        with path.open("r", encoding="utf-8", newline="") as handle:
            return _read_table(handle, str(path))
    except OSError as error:
        raise CsvToolError(f"cannot read generated CSV {path}: {error}") from error


def _reference_member(archive: zipfile.ZipFile, filename: str) -> str:
    matches = [
        name
        for name in archive.namelist()
        if not name.endswith("/") and PurePosixPath(name).name == filename
    ]
    if not matches:
        raise CsvToolError(f"reference archive has no member named {filename}")
    if len(matches) > 1:
        raise CsvToolError(
            f"reference archive contains multiple members named {filename}: {matches}"
        )
    return matches[0]


def load_reference_csv(reference_zip: Path, filename: str) -> CsvTable:
    if not reference_zip.is_file():
        raise CsvToolError(
            f"reference ZIP does not exist: {reference_zip} "
            "(set BASILISK_REFERENCE_CSV_ZIP or pass --reference-zip)"
        )
    try:
        with zipfile.ZipFile(reference_zip) as archive:
            member = _reference_member(archive, filename)
            with archive.open(member) as raw_handle:
                with io.TextIOWrapper(
                    raw_handle, encoding="utf-8", newline=""
                ) as handle:
                    return _read_table(handle, f"{reference_zip}!/{member}")
    except zipfile.BadZipFile as error:
        raise CsvToolError(f"invalid reference ZIP {reference_zip}: {error}") from error
    except OSError as error:
        raise CsvToolError(
            f"cannot read reference ZIP {reference_zip}: {error}"
        ) from error


def load_scenario_pair(
    scenario: ScenarioSpec,
    generated_dir: Path = DEFAULT_GENERATED_DIR,
    reference_zip: Path = DEFAULT_REFERENCE_ZIP,
) -> tuple[CsvTable, CsvTable]:
    generated = load_csv_path(generated_dir / scenario.filename)
    reference = load_reference_csv(reference_zip, scenario.filename)
    return generated, reference


def finite_values(table: CsvTable) -> bool:
    return all(math.isfinite(value) for row in table.rows for value in row)
