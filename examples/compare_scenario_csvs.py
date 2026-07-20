#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
from collections import defaultdict
from dataclasses import dataclass
from decimal import Decimal
from pathlib import Path

from scenario_csv_tools import (
    DEFAULT_GENERATED_DIR,
    DEFAULT_REFERENCE_ZIP,
    SCENARIOS,
    CsvTable,
    CsvToolError,
    ScenarioSpec,
    ToleranceGroup,
    finite_values,
    load_scenario_pair,
)


@dataclass
class GroupStatistics:
    samples: int = 0
    violations: int = 0
    maximum_absolute: float = 0.0
    maximum_relative: float = 0.0
    squared_error_sum: float = 0.0

    def include(
        self, actual: float, expected: float, tolerance: ToleranceGroup
    ) -> bool:
        absolute_error = abs(actual - expected)
        if expected == 0.0:
            relative_error = 0.0 if absolute_error == 0.0 else math.inf
        else:
            relative_error = absolute_error / abs(expected)
        limit = tolerance.absolute + tolerance.relative * abs(expected)
        passed = absolute_error <= limit

        self.samples += 1
        self.violations += not passed
        self.maximum_absolute = max(self.maximum_absolute, absolute_error)
        self.maximum_relative = max(self.maximum_relative, relative_error)
        self.squared_error_sum += absolute_error * absolute_error
        return passed

    @property
    def rms(self) -> float:
        return math.sqrt(self.squared_error_sum / self.samples) if self.samples else 0.0


def _validate_shape(label: str, table: CsvTable, scenario: ScenarioSpec) -> list[str]:
    errors: list[str] = []
    if len(table.rows) != scenario.rows:
        errors.append(
            f"{label} row count: expected {scenario.rows}, found {len(table.rows)}"
        )
    if len(table.headers) != scenario.columns:
        errors.append(
            f"{label} column count: expected {scenario.columns}, found {len(table.headers)}"
        )
    expected_times = tuple(
        Decimal(index * scenario.sample_period_ns) for index in range(scenario.rows)
    )
    if table.timestamps != expected_times:
        mismatch = next(
            (
                index
                for index, (actual, expected) in enumerate(
                    zip(table.timestamps, expected_times, strict=False)
                )
                if actual != expected
            ),
            min(len(table.timestamps), len(expected_times)),
        )
        errors.append(
            f"{label} timestamp sequence differs at row {mismatch} "
            f"(expected a {scenario.sample_period_ns} ns cadence starting at zero)"
        )
    if not finite_values(table):
        errors.append(f"{label} contains NaN or infinite values")
    return errors


def compare_scenario(
    scenario: ScenarioSpec, generated_dir: Path, reference_zip: Path
) -> bool:
    print(f"\n[{scenario.key}] {scenario.filename}")
    try:
        generated, reference = load_scenario_pair(
            scenario, generated_dir, reference_zip
        )
    except CsvToolError as error:
        print(f"  ERROR: {error}")
        return False

    errors = _validate_shape("generated", generated, scenario)
    errors.extend(_validate_shape("reference", reference, scenario))
    if generated.headers != reference.headers:
        errors.append(
            "generated headers do not exactly match the reference header order"
        )
        for index, (actual, expected) in enumerate(
            zip(generated.headers, reference.headers, strict=False)
        ):
            if actual != expected:
                errors.append(
                    f"first header mismatch at column {index}: {actual!r} != {expected!r}"
                )
                break
    if generated.timestamps != reference.timestamps:
        errors.append(
            "generated timestamps do not exactly match the reference timestamp vector"
        )

    inactive_errors: list[str] = []
    if generated.headers == reference.headers:
        for column_index, column in enumerate(generated.headers[1:], start=1):
            if not scenario.is_inactive(column):
                continue
            for row_index, row in enumerate(generated.rows):
                if row[column_index] != 0.0:
                    inactive_errors.append(
                        f"inactive column {column!r} is nonzero at row {row_index}: "
                        f"{row[column_index]:.17g}"
                    )
                    break
    errors.extend(inactive_errors[:10])
    if len(inactive_errors) > 10:
        errors.append(
            f"... and {len(inactive_errors) - 10} more inactive-column failures"
        )

    if errors:
        for error in errors:
            print(f"  ERROR: {error}")
        return False

    statistics: dict[str, GroupStatistics] = defaultdict(GroupStatistics)
    details: list[str] = []
    unclassified: list[str] = []

    for column_index, column in enumerate(generated.headers[1:], start=1):
        if scenario.is_inactive(column):
            continue
        tolerance = scenario.tolerance_for(column)
        if tolerance is None:
            unclassified.append(column)
            continue
        group_statistics = statistics[tolerance.name]
        for row_index, (actual_row, expected_row) in enumerate(
            zip(generated.rows, reference.rows, strict=True)
        ):
            actual = actual_row[column_index]
            expected = expected_row[column_index]
            if (
                not group_statistics.include(actual, expected, tolerance)
                and len(details) < 12
            ):
                error = abs(actual - expected)
                limit = tolerance.absolute + tolerance.relative * abs(expected)
                details.append(
                    f"{column} row {row_index}: actual={actual:.17g}, "
                    f"reference={expected:.17g}, abs_error={error:.6e}, limit={limit:.6e}"
                )

    if unclassified:
        print(
            f"  ERROR: no tolerance configured for columns: {', '.join(unclassified)}"
        )
        return False

    passed = True
    for tolerance in scenario.tolerance_groups:
        group = statistics[tolerance.name]
        passed &= group.violations == 0
        relative = (
            "inf"
            if math.isinf(group.maximum_relative)
            else f"{group.maximum_relative:.6e}"
        )
        status = "PASS" if group.violations == 0 else "FAIL"
        print(
            f"  {status:<4} {tolerance.name:<18} "
            f"max_abs={group.maximum_absolute:.6e} max_rel={relative:<12} "
            f"rms={group.rms:.6e} violations={group.violations}/{group.samples}"
        )

    for detail in details:
        print(f"    {detail}")
    print(f"  {'PASS' if passed else 'FAIL'}: {scenario.key}")
    return passed


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare basilisk-rs scenario CSVs against Basilisk C++ references."
    )
    parser.add_argument(
        "--generated-dir",
        type=Path,
        default=DEFAULT_GENERATED_DIR,
        help=f"directory containing generated CSVs (default: {DEFAULT_GENERATED_DIR})",
    )
    parser.add_argument(
        "--reference-zip",
        type=Path,
        default=DEFAULT_REFERENCE_ZIP,
        help=(
            "reference ZIP (default: BASILISK_REFERENCE_CSV_ZIP or "
            f"{DEFAULT_REFERENCE_ZIP})"
        ),
    )
    parser.add_argument(
        "--scenario",
        action="append",
        choices=tuple(SCENARIOS),
        help="scenario to compare; repeat as needed (default: all five)",
    )
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    selected = arguments.scenario or list(SCENARIOS)
    results = [
        compare_scenario(
            SCENARIOS[key], arguments.generated_dir, arguments.reference_zip
        )
        for key in selected
    ]
    passed = sum(results)
    print(f"\nScenario parity: {passed}/{len(results)} passed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
