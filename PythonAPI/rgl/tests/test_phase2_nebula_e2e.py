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

# CARLA Python API + lidar_models (relative to PythonAPI/rgl/)
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import carla
from lidar_models import apply_preset

# ROS2 / numpy will be imported lazily inside main() so that --help / prereq
# checks can run without a sourced ROS2 environment.

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


def main():
    raise NotImplementedError("main implemented in later tasks")


if __name__ == "__main__":
    sys.exit(main())
