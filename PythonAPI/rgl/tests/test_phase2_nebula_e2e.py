#!/usr/bin/env python3
"""Phase 2 verification: tier4/nebula E2E for all 8 LiDAR models.

The script auto-launches the CARLA shipping binary, then for each LiDAR
model spawns a Nebula process via ros2 launch, exercises the CARLA RGL
UDP raw-packet output, and validates the resulting PointCloud2 messages
against five indicators (receipt count / ring count / point count /
azimuth coverage / distance range).

Usage:
    # Activate Nebula's colcon workspace first:
    source /opt/ros/humble/setup.bash
    source /mnt/dsk0/wk0/ROS2/humble/X2/shiojiri/pilot-auto.x2_v4.3.1_awsim/install/setup.bash

    # Run (test auto-launches CARLA):
    python3 test_phase2_nebula_e2e.py

Exit codes:
    0 = all 8 models passed
    1 = at least one model failed
    2 = prerequisite missing (CARLA binary, carla wheel, rclpy, nebula_ros)
"""

import os
import signal
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

# Path setup for lidar_models import (relative to PythonAPI/rgl/). The
# actual `import carla` and `from lidar_models import apply_preset` calls
# are deferred to verify_model() so that check_prereqs() can run on
# machines where the CARLA wheel is not installed and still return exit 2
# (per the documented contract in the module docstring).
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

# ----------------------------------------------------------------------------
# Constants
# ----------------------------------------------------------------------------

CARLA_HOST = "127.0.0.1"
CARLA_RPC_PORT = 2000
DEST_IP = "127.0.0.1"
DEST_PORT_BASE = 2368

CARLA_BIN = Path(
    "/mnt/dsk0/wk0/CARLA/T4Fork.rgl/CarlaUE5/Build/Package/"
    "Carla-0.10.0-Linux-Shipping/Linux/CarlaUnreal.sh"
)

ALL_MODELS = [
    "VelodyneVLP16", "VelodyneVLP32C", "VelodyneVLS128",
    "HesaiPandar40P", "HesaiPandarQT", "HesaiPandarXT32",
    "HesaiQT128C2X", "HesaiPandar128E4X",
]

# (CARLA return_mode, Nebula return_mode) per model.
# HesaiPandarQT's AWSIM whitelist (LidarUdpPublisher.cs::SupportedLidarsAndReturnModes)
# does not include Strongest, so use First instead.
RETURN_MODE_FOR_PHASE2 = {
    "VelodyneVLP16":     ("strongest", "Strongest"),
    "VelodyneVLP32C":    ("strongest", "Strongest"),
    "VelodyneVLS128":    ("strongest", "Strongest"),
    "HesaiPandar40P":    ("strongest", "Strongest"),
    "HesaiPandarQT":     ("first",     "First"),
    "HesaiPandarXT32":   ("strongest", "Strongest"),
    "HesaiQT128C2X":     ("strongest", "Strongest"),
    "HesaiPandar128E4X": ("strongest", "Strongest"),
}

# CARLA preset name -> Nebula sensor_model launch arg.
NEBULA_MODEL = {
    "VelodyneVLP16":     "VLP16",
    "VelodyneVLP32C":    "VLP32",
    "VelodyneVLS128":    "VLS128",
    "HesaiPandar40P":    "Pandar40P",
    "HesaiPandarQT":     "PandarQT64",
    "HesaiPandarXT32":   "PandarXT32",
    "HesaiQT128C2X":     "PandarQT128",
    "HesaiPandar128E4X": "Pandar128E4X",
}

# Per-model expected values for validation.
EXPECTED = {
    "VelodyneVLP16":     {"rings": 16,  "min_pts_avg": 1_000, "max_range_m": 200.0},
    "VelodyneVLP32C":    {"rings": 32,  "min_pts_avg": 1_000, "max_range_m": 200.0},
    "VelodyneVLS128":    {"rings": 128, "min_pts_avg": 5_000, "max_range_m": 200.0},
    "HesaiPandar40P":    {"rings": 40,  "min_pts_avg": 1_000, "max_range_m": 200.0},
    "HesaiPandarQT":     {"rings": 64,  "min_pts_avg": 1_000, "max_range_m": 200.0},
    "HesaiPandarXT32":   {"rings": 32,  "min_pts_avg": 1_000, "max_range_m": 200.0},
    "HesaiQT128C2X":     {"rings": 128, "min_pts_avg": 5_000, "max_range_m": 200.0},
    "HesaiPandar128E4X": {"rings": 128, "min_pts_avg": 5_000, "max_range_m": 200.0},
}

MIN_RANGE_M = 0.1
AZIMUTH_BINS = 36          # 10 degrees per bin
AZIMUTH_COVERAGE_MIN = 32  # at least 32 of 36 bins must have points
DIST_AZ_PASSING_SCANS = 3  # 3 of 5 scans must satisfy the soft criteria


# ----------------------------------------------------------------------------
# CARLA helpers
# ----------------------------------------------------------------------------

def setup_sync(world):
    """Enable synchronous mode with a fixed 0.05 s step. Tick once so the
    world settles before the caller starts to spawn actors."""
    s = world.get_settings()
    s.synchronous_mode = True
    s.fixed_delta_seconds = 0.05
    world.apply_settings(s)
    world.tick()


def teardown_sync(world):
    """Restore asynchronous mode."""
    s = world.get_settings()
    s.synchronous_mode = False
    world.apply_settings(s)


def wait_for_carla_rpc(host, port, timeout=60.0):
    """Poll the CARLA RPC port until it accepts a TCP connection or the
    timeout expires. Returns True on success, False otherwise."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(1.0)
            try:
                sock.connect((host, port))
                return True
            except (socket.timeout, ConnectionRefusedError, OSError):
                pass
        time.sleep(1.0)
    return False


# ----------------------------------------------------------------------------
# Nebula / rclpy helpers
# ----------------------------------------------------------------------------

def find_aw_points_topic(node, max_wait=15.0):
    """Poll the ROS graph until any topic ending in '/aw_points' of type
    sensor_msgs/msg/PointCloud2 appears, then return its absolute name.
    Raises RuntimeError on timeout."""
    deadline = time.time() + max_wait
    while time.time() < deadline:
        for name, types in node.get_topic_names_and_types():
            if name.endswith("/aw_points") and \
               "sensor_msgs/msg/PointCloud2" in types:
                return name
        time.sleep(0.5)
    raise RuntimeError(
        f"Nebula '/aw_points' topic did not appear within {max_wait:.0f}s"
    )


# ----------------------------------------------------------------------------
# Prerequisite check
# ----------------------------------------------------------------------------

def check_prereqs():
    """Verify all external dependencies are reachable. Returns 0 on success,
    2 if any prerequisite is missing (prints actionable hints to stdout)."""
    issues = []

    if not CARLA_BIN.exists():
        issues.append(
            f"CARLA binary not found: {CARLA_BIN}\n"
            f"  Build the shipping package first:\n"
            f"    bash RglSetup.sh build --package=shipping"
        )

    try:
        import carla  # noqa: F401
    except ImportError as e:
        issues.append(f"carla Python wheel not importable: {e}")

    try:
        from lidar_models import apply_preset  # noqa: F401
    except ImportError as e:
        issues.append(
            f"lidar_models package not importable: {e}\n"
            f"  Verify PythonAPI/rgl/lidar_models/ exists and is intact"
        )

    try:
        import rclpy  # noqa: F401
        from sensor_msgs.msg import PointCloud2  # noqa: F401
    except ImportError as e:
        issues.append(
            f"rclpy / sensor_msgs not importable: {e}\n"
            f"  Source ROS2 Humble first:\n"
            f"    source /opt/ros/humble/setup.bash"
        )

    try:
        import numpy  # noqa: F401
    except ImportError as e:
        issues.append(f"numpy not importable: {e}")

    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "nebula_ros"],
            capture_output=True, text=True, timeout=10,
        )
        ros2_ok = result.returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired):
        ros2_ok = False
    if not ros2_ok:
        issues.append(
            "nebula_ros package not on AMENT_PREFIX_PATH.\n"
            "  Source the colcon workspace that contains Nebula, e.g.\n"
            "    source /mnt/dsk0/wk0/ROS2/humble/X2/shiojiri/"
            "pilot-auto.x2_v4.3.1_awsim/install/setup.bash"
        )

    if issues:
        print("PREREQUISITE CHECK FAILED:")
        for i in issues:
            # Each issue may span multiple lines (hint text). Indent
            # continuation lines so the "- " bullet visually owns the block.
            lines = i.splitlines()
            print(f"  - {lines[0]}")
            for cont in lines[1:]:
                print(f"    {cont}")
        return 2
    return 0


def main():
    raise NotImplementedError("main implemented in later tasks")


if __name__ == "__main__":
    sys.exit(main())
