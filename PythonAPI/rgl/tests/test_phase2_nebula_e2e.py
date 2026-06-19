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


# ----------------------------------------------------------------------------
# PointCloud2 decoding + validation
# ----------------------------------------------------------------------------

# Mapping from sensor_msgs/PointField datatype id to (struct format, size).
_PC2_TYPE = {
    1: ("<b", 1),  # INT8
    2: ("<B", 1),  # UINT8
    3: ("<h", 2),  # INT16
    4: ("<H", 2),  # UINT16
    5: ("<i", 4),  # INT32
    6: ("<I", 4),  # UINT32
    7: ("<f", 4),  # FLOAT32
    8: ("<d", 8),  # FLOAT64
}


def _pc2_to_numpy(msg):
    """Decode a sensor_msgs/PointCloud2 message into a dict containing
    'xyz' (Nx3 float32), 'ring' (N int32), and 'intensity' (N float32).
    Missing optional fields fall back to zero arrays."""
    import numpy as np

    fields = {f.name: f for f in msg.fields}
    if not {"x", "y", "z"}.issubset(fields):
        raise RuntimeError(
            "PointCloud2 message is missing required x/y/z fields"
        )

    point_step = msg.point_step
    n = msg.width * msg.height
    raw = bytes(msg.data)

    xyz = np.zeros((n, 3), dtype=np.float32)
    ring = np.zeros(n, dtype=np.int32)
    intensity = np.zeros(n, dtype=np.float32)

    x_off = fields["x"].offset
    y_off = fields["y"].offset
    z_off = fields["z"].offset

    ring_field = fields.get("ring")
    ring_off = ring_field.offset if ring_field else None
    ring_fmt = _PC2_TYPE[ring_field.datatype][0] if ring_field else None

    intensity_field = fields.get("intensity")
    intensity_off = intensity_field.offset if intensity_field else None
    intensity_fmt = _PC2_TYPE[intensity_field.datatype][0] \
        if intensity_field else None

    for i in range(n):
        base = i * point_step
        xyz[i, 0] = struct.unpack_from("<f", raw, base + x_off)[0]
        xyz[i, 1] = struct.unpack_from("<f", raw, base + y_off)[0]
        xyz[i, 2] = struct.unpack_from("<f", raw, base + z_off)[0]
        if ring_off is not None:
            ring[i] = struct.unpack_from(ring_fmt, raw, base + ring_off)[0]
        if intensity_off is not None:
            intensity[i] = struct.unpack_from(
                intensity_fmt, raw, base + intensity_off)[0]

    return {"xyz": xyz, "ring": ring, "intensity": intensity}


def _fail(indicators, detail):
    return {"passed": False, "indicators": indicators, "detail": detail}


def validate_pointclouds(model_name, msgs):
    """Apply the 5 Phase 2 indicators to a list of PointCloud2 messages.

    Returns a dict with keys:
      passed (bool), indicators (dict[str, bool]), detail (str).
    """
    import numpy as np

    exp = EXPECTED[model_name]
    indicators = {}

    # Indicator 1: receipt count
    indicators["received_5"] = len(msgs) >= 5
    if not indicators["received_5"]:
        return _fail(
            indicators,
            f"only {len(msgs)} PointCloud2 msgs received (need >=5)"
        )

    decoded = [_pc2_to_numpy(m) for m in msgs[:5]]

    # Indicator 2: ring count (skip empty scans; if all empty, fail explicitly)
    non_empty = [d for d in decoded if d["ring"].size]
    if non_empty:
        max_ring = max(int(d["ring"].max()) for d in non_empty)
        indicators["ring_count_exact"] = (max_ring + 1) == exp["rings"]
        detail_ring = f"rings={max_ring + 1} (expected {exp['rings']})"
    else:
        indicators["ring_count_exact"] = False
        detail_ring = "rings=0 (all scans empty)"

    # Indicator 3: average points per scan
    avg_pts = sum(d["xyz"].shape[0] for d in decoded) / len(decoded)
    indicators["min_points_avg"] = avg_pts >= exp["min_pts_avg"]
    detail_pts = f"avg_pts={avg_pts:.0f} (>={exp['min_pts_avg']})"

    # Indicator 4: azimuth coverage (>=32 of 36 ten-degree bins in >=3 scans)
    coverage_ok = 0
    for d in decoded:
        if d["xyz"].shape[0] == 0:
            continue
        az = np.degrees(np.arctan2(d["xyz"][:, 1], d["xyz"][:, 0])) % 360.0
        bin_ids = np.unique((az // 10).astype(np.int32))
        if bin_ids.size >= AZIMUTH_COVERAGE_MIN:
            coverage_ok += 1
    indicators["azimuth_full_circle"] = coverage_ok >= DIST_AZ_PASSING_SCANS
    detail_az = f"az_coverage_ok={coverage_ok}/5 (need >={DIST_AZ_PASSING_SCANS})"

    # Indicator 5: distance range
    dist_within = 0
    for d in decoded:
        if d["xyz"].shape[0] == 0:
            continue
        ranges = np.linalg.norm(d["xyz"], axis=1)
        if ranges.min() >= MIN_RANGE_M and ranges.max() <= exp["max_range_m"]:
            dist_within += 1
    indicators["distance_in_range"] = dist_within >= DIST_AZ_PASSING_SCANS
    detail_dist = (
        f"dist_in_[{MIN_RANGE_M},{exp['max_range_m']}]_m={dist_within}/5 "
        f"(need >={DIST_AZ_PASSING_SCANS})"
    )

    passed = all(indicators.values())
    detail = " | ".join([detail_ring, detail_pts, detail_az, detail_dist])
    return {"passed": passed, "indicators": indicators, "detail": detail}


# ----------------------------------------------------------------------------
# Per-model verification cycle
# ----------------------------------------------------------------------------

def verify_model(world, model_name):
    """Run the full Nebula <- CARLA UDP cycle for one model and return the
    validation result dict."""
    import carla  # noqa: F401  (used via world parameter; ensure module loaded)
    from lidar_models import apply_preset
    import rclpy
    from rclpy.qos import qos_profile_sensor_data as SENSOR_DATA
    from sensor_msgs.msg import PointCloud2

    vendor = "velodyne" if model_name.startswith("Velodyne") else "hesai"
    carla_mode, nebula_mode = RETURN_MODE_FOR_PHASE2[model_name]
    udp_port = DEST_PORT_BASE + ALL_MODELS.index(model_name)

    # 1. Launch Nebula
    nebula_proc = subprocess.Popen(
        [
            "ros2", "launch", "nebula_ros",
            f"{vendor}_launch_all_hw.xml",
            f"sensor_model:={NEBULA_MODEL[model_name]}",
            "launch_hw:=false",
            f"return_mode:={nebula_mode}",
            f"host_ip:={DEST_IP}",
            f"data_port:={udp_port}",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        env=os.environ.copy(),
    )

    node = rclpy.create_node(f"phase2_test_{model_name.lower()}")
    sensor = None
    sync_active = False

    try:
        # 2. Wait for the Nebula PointCloud2 topic to appear
        try:
            topic_name = find_aw_points_topic(node, max_wait=15.0)
        except RuntimeError as e:
            stderr = nebula_proc.stderr.read().decode(
                errors="replace") if nebula_proc.stderr else ""
            return {
                "passed": False,
                "detail": f"Nebula topic timeout: {e}",
                "nebula_stderr": stderr[-500:],
            }

        msgs = []
        node.create_subscription(
            PointCloud2, topic_name,
            lambda m: msgs.append(m), qos_profile=SENSOR_DATA)

        # 3. Spawn the CARLA LiDAR
        bp = world.get_blueprint_library().find("sensor.lidar.rgl")
        apply_preset(
            bp, model_name,
            udp_publish={"dest_ip": DEST_IP, "dest_port": udp_port})
        bp.set_attribute("return_mode", carla_mode)
        spawn_point = world.get_map().get_spawn_points()[0]
        sensor = world.spawn_actor(bp, spawn_point)

        # 4. Tick CARLA + spin rclpy until we have 5 messages or 10 s elapsed
        setup_sync(world)
        sync_active = True
        deadline = time.time() + 10.0
        while len(msgs) < 5 and time.time() < deadline:
            world.tick()
            rclpy.spin_once(node, timeout_sec=0.05)

        # 5. Validate
        return validate_pointclouds(model_name, msgs)
    finally:
        # 6. Cleanup
        if sensor is not None:
            try:
                sensor.destroy()
            except Exception:
                pass
        if sync_active:
            try:
                teardown_sync(world)
            except Exception:
                pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            nebula_proc.send_signal(signal.SIGINT)
            nebula_proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            nebula_proc.kill()
            nebula_proc.wait(timeout=5)


def main():
    rc = check_prereqs()
    if rc != 0:
        return rc

    import rclpy
    rclpy.init(args=None)

    carla_log_path = Path("/tmp/carla_phase2.log")
    carla_log = open(carla_log_path, "w")
    print(f"Launching CARLA, log -> {carla_log_path}")
    carla_proc = subprocess.Popen(
        [str(CARLA_BIN), "-RenderOffScreen", "-nosound"],
        stdout=carla_log,
        stderr=subprocess.STDOUT,
        env=os.environ.copy(),
    )

    try:
        if not wait_for_carla_rpc(CARLA_HOST, CARLA_RPC_PORT, timeout=60.0):
            print("FAIL: CARLA server did not start within 60s")
            return 1

        import carla
        client = carla.Client(CARLA_HOST, CARLA_RPC_PORT)
        client.set_timeout(10.0)
        world = client.get_world()

        summary = {}
        for model in ALL_MODELS:
            print(f"\n=== {model} ===")
            try:
                summary[model] = verify_model(world, model)
            except Exception as e:
                summary[model] = {"passed": False, "detail": f"exception: {e}"}
            res = summary[model]
            marker = "PASS" if res.get("passed") else "FAIL"
            print(f"  [{marker}] {res.get('detail', '')}")
            if not res.get("passed") and "nebula_stderr" in res:
                print(f"  nebula stderr (last 500B): {res['nebula_stderr']}")

        passed = sum(1 for r in summary.values() if r.get("passed"))
        print("\n" + "=" * 60)
        print(f"Phase 2 results: {passed}/{len(ALL_MODELS)} models passed")
        for model in ALL_MODELS:
            res = summary[model]
            marker = "PASS" if res.get("passed") else "FAIL"
            print(f"  [{marker}] {model}: {res.get('detail', '')}")
        return 0 if passed == len(ALL_MODELS) else 1
    finally:
        try:
            carla_proc.send_signal(signal.SIGINT)
            carla_proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            carla_proc.kill()
            carla_proc.wait(timeout=10)
        try:
            carla_log.close()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
