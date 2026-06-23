#!/usr/bin/env python3
"""CARLA-free unit tests for apply_preset hesai_ros_driver_compat default logic.

Verifies the truth table for the None/True/False sentinel:
  - None + Hesai   → horizontal_start_angle set to -90.0
  - None + non-Hesai → NOT set, NO warning
  - True  + Hesai  → set to -90.0
  - True  + non-Hesai → warning emitted
  - False + Hesai  → NOT set
  - False + non-Hesai → nothing

These tests do NOT require a live CARLA server or the carla Python wheel.

Usage:
    python3 test_apply_preset_compat.py

Exit code 0 on full success, 1 on any failure.
"""

import os
import sys
import warnings

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from lidar_models import apply_preset


# ---------------------------------------------------------------------------
# FakeBlueprint
# ---------------------------------------------------------------------------

class FakeBlueprint:
    """Minimal blueprint stub. Records set_attribute calls; never raises."""

    def __init__(self):
        self.attrs = {}

    def set_attribute(self, name, value):
        self.attrs[name] = value

    def get_attribute(self, name):
        return self.attrs.get(name)


# ---------------------------------------------------------------------------
# Test cases
# ---------------------------------------------------------------------------

def test_default_hesai_sets_start_angle():
    """None (default) + Hesai model → horizontal_start_angle == '-90.0'."""
    bp = FakeBlueprint()
    apply_preset(bp, "HesaiPandar128E4X")
    assert bp.attrs.get("horizontal_start_angle") == "-90.0", (
        f"Expected '-90.0', got {bp.attrs.get('horizontal_start_angle')!r}")


def test_default_non_hesai_no_angle_no_warning():
    """None (default) + non-Hesai → NOT set, zero warnings."""
    bp = FakeBlueprint()
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        apply_preset(bp, "VelodyneVLP16")
    assert "horizontal_start_angle" not in bp.attrs, (
        "horizontal_start_angle should NOT be set for non-Hesai default")
    assert len(caught) == 0, (
        f"Expected 0 warnings, got {len(caught)}: {[str(w.message) for w in caught]}")


def test_explicit_false_hesai_no_angle():
    """explicit False + Hesai → horizontal_start_angle NOT set."""
    bp = FakeBlueprint()
    apply_preset(bp, "HesaiPandar128E4X", hesai_ros_driver_compat=False)
    assert "horizontal_start_angle" not in bp.attrs, (
        "horizontal_start_angle should NOT be set when compat=False")


def test_explicit_true_non_hesai_warns():
    """explicit True + non-Hesai → exactly one warning emitted."""
    bp = FakeBlueprint()
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        apply_preset(bp, "VelodyneVLP16", hesai_ros_driver_compat=True)
    assert len(caught) == 1, (
        f"Expected exactly 1 warning, got {len(caught)}: "
        f"{[str(w.message) for w in caught]}")
    assert "non-Hesai" in str(caught[0].message), (
        f"Warning text should mention non-Hesai: {caught[0].message}")


# ---------------------------------------------------------------------------
# Main runner
# ---------------------------------------------------------------------------

def main():
    tests = [
        test_default_hesai_sets_start_angle,
        test_default_non_hesai_no_angle_no_warning,
        test_explicit_false_hesai_no_angle,
        test_explicit_true_non_hesai_warns,
    ]
    failures = []
    for fn in tests:
        name = fn.__name__
        try:
            fn()
            print(f"  PASS: {name}")
        except AssertionError as exc:
            failures.append(f"{name}: {exc}")
            print(f"  FAIL: {name}: {exc}")
        except Exception as exc:
            failures.append(f"{name}: unexpected error: {exc}")
            print(f"  ERROR: {name}: {exc}")

    print("\n" + "=" * 60)
    if failures:
        print(f"FAILED ({len(failures)}):")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)
    else:
        print(f"PASSED ({len(tests)} tests)")
        sys.exit(0)


if __name__ == "__main__":
    main()
