#!/usr/bin/env python3
"""Regression test suite for CARLA RGL LiDAR preset features.

Tests all AWSIM-ported preset functionality:
- 13 model presets (spawn/tick/destroy)
- Per-channel angles and ring IDs
- Special firing patterns (AT128 range, QT128/Pandar128 step offset)
- Ray masks (azimuth, ring, rect, raw)
- Noise model (angular ray/hitpoint, distance, disable)
- Beam divergence (on/off, manual, disable, invalid input)
- delta+zlib+base64 encoding roundtrip
- ROS2 direct publish (optional, requires ROS2 environment)
- Legacy mode (no preset)

Usage:
    # Basic tests (no ROS2 needed):
    python3 rgl_test_regression.py

    # Full tests with ROS2 verification:
    source /opt/ros/humble/setup.bash  # or your ROS2 workspace
    python3 rgl_test_regression.py --ros2

    # Test specific category:
    python3 rgl_test_regression.py --test spawn
    python3 rgl_test_regression.py --test mask --ros2

Requires CARLA server running with --ros2 flag.
"""

import argparse
import math
import os
import struct
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import carla

from lidar_models import (
    MODEL_REGISTRY, NOISE_REGISTRY, BEAM_DIVERGENCE_REGISTRY,
    apply_preset, list_models,
    _encode_float_array, _encode_int_array,
    set_azimuth_fov, set_disabled_rings, set_mask_rectangles, set_raw_mask,
    set_noise, disable_noise,
    set_beam_divergence, disable_beam_divergence,
    set_return_mode,
)


class TestResult:
    def __init__(self):
        self.passed = []
        self.failed = []

    def ok(self, name, detail=""):
        self.passed.append(name)
        print(f"  PASS: {name}" + (f" ({detail})" if detail else ""))

    def fail(self, name, detail=""):
        self.failed.append(name)
        print(f"  FAIL: {name}" + (f" ({detail})" if detail else ""))

    def check(self, name, condition, detail=""):
        if condition:
            self.ok(name, detail)
        else:
            self.fail(name, detail)

    def summary(self):
        total = len(self.passed) + len(self.failed)
        print(f"\n{'='*60}")
        print(f"Results: {len(self.passed)}/{total} passed, {len(self.failed)} failed")
        if self.failed:
            print("Failed tests:")
            for name in self.failed:
                print(f"  - {name}")
        print(f"{'='*60}")
        return len(self.failed) == 0


# ============================================================================
# Helpers
# ============================================================================

def setup_sync(world):
    s = world.get_settings()
    s.synchronous_mode = True
    s.fixed_delta_seconds = 0.05
    world.apply_settings(s)
    world.tick()


def teardown_sync(world):
    s = world.get_settings()
    s.synchronous_mode = False
    world.apply_settings(s)


def spawn_rgl_sensor(world, preset_name=None, extra_setup=None,
                     location=carla.Location(z=3), rotation=carla.Rotation()):
    bp = world.get_blueprint_library().find("sensor.lidar.rgl")
    if preset_name:
        apply_preset(bp, preset_name)
    else:
        bp.set_attribute("channels", "16")
        bp.set_attribute("range", "100")
        bp.set_attribute("upper_fov", "10")
        bp.set_attribute("lower_fov", "-20")
        bp.set_attribute("points_per_second", "288000")
    bp.set_attribute("sensor_tick", "0.1")
    if extra_setup:
        extra_setup(bp)
    sensor = world.spawn_actor(bp, carla.Transform(location, rotation))
    return sensor


def tick_and_destroy(world, sensor, ticks=5):
    for _ in range(ticks):
        world.tick()
        time.sleep(0.01)
    sensor.destroy()
    world.tick()


def capture_ros2_msg(world, sensor, topic, ticks=80):
    """Capture one PointCloud2 message via rclpy. Returns msg or None."""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import PointCloud2
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    except ImportError:
        return None

    rclpy.init()
    node = rclpy.create_node("regression_test_%d" % (int(time.time() * 1000) % 100000))
    msg_data = [None]

    def cb(msg):
        if msg_data[0] is None:
            msg_data[0] = msg

    qos = QoSProfile(depth=10)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.durability = DurabilityPolicy.VOLATILE
    node.create_subscription(PointCloud2, topic, cb, qos)

    for _ in range(ticks):
        world.tick()
        rclpy.spin_once(node, timeout_sec=0.01)

    node.destroy_node()
    rclpy.shutdown()
    return msg_data[0]


def parse_points(msg):
    """Parse PointXYZIRCAEDT message into list of dicts."""
    points = []
    for i in range(msg.width):
        off = i * 32
        x, y, z = struct.unpack_from("<fff", msg.data, off)
        intensity = msg.data[off + 12]
        return_type = msg.data[off + 13]
        ring_id = struct.unpack_from("<H", msg.data, off + 14)[0]
        azimuth, elevation, distance = struct.unpack_from("<fff", msg.data, off + 16)
        timestamp = struct.unpack_from("<I", msg.data, off + 28)[0]
        points.append({
            "x": x, "y": y, "z": z,
            "ring_id": ring_id, "distance": distance,
            "azimuth": azimuth, "elevation": elevation,
            "return_type": return_type,
        })
    return points


# ============================================================================
# Test categories
# ============================================================================

def test_encoding(result):
    """Test delta+zlib+base64 encoding roundtrip."""
    print("\n--- Encoding tests ---")

    # Float encoding
    values = [15.0, -1.0, 13.0, -3.0, 11.0, -5.0]
    encoded = _encode_float_array(values)
    result.check("encode_float_length", len(encoded) < 100,
                 f"{len(encoded)} chars")
    result.check("encode_float_base64", encoded.isascii(),
                 "ASCII-safe")

    # Int encoding
    ints = [0, 8, 1, 9, 2, 10]
    encoded_int = _encode_int_array(ints)
    result.check("encode_int_length", len(encoded_int) < 100,
                 f"{len(encoded_int)} chars")

    # 128-channel encoding stays under FName limit
    big = [float(i) * 0.8 - 52.0 for i in range(128)]
    encoded_big = _encode_float_array(big)
    result.check("encode_128ch_fname_limit", len(encoded_big) <= 1023,
                 f"{len(encoded_big)} chars")

    # 256-entry (AT128 range pattern) stays under limit
    pattern = [0.5, 7.2] * 128
    encoded_pattern = _encode_float_array(pattern)
    result.check("encode_256entry_fname_limit", len(encoded_pattern) <= 1023,
                 f"{len(encoded_pattern)} chars")


def test_preset_data(result):
    """Test preset data integrity."""
    print("\n--- Preset data tests ---")

    result.check("model_count", len(MODEL_REGISTRY) == 13,
                 f"got {len(MODEL_REGISTRY)}")

    expected_models = [
        "RangeMeter", "SickMRS6000",
        "VelodyneVLP16", "VelodyneVLP32C", "VelodyneVLS128",
        "HesaiPandar40P", "HesaiPandarQT", "HesaiPandarXT32",
        "HesaiAT128E2X", "HesaiQT128C2X", "HesaiPandar128E4X",
        "HesaiPandar128E4XHighRes", "OusterOS1_64",
    ]
    for name in expected_models:
        result.check(f"preset_{name}", name in MODEL_REGISTRY)

    # Verify key properties
    vlp16 = MODEL_REGISTRY["VelodyneVLP16"]
    result.check("vlp16_channels", vlp16["channels"] == 16)
    result.check("vlp16_angles", len(vlp16["vertical_angles"]) == 16)
    result.check("vlp16_first_angle", vlp16["vertical_angles"][0] == 15.0,
                 f"got {vlp16['vertical_angles'][0]}")

    at128 = MODEL_REGISTRY["HesaiAT128E2X"]
    result.check("at128_range_pattern", at128.get("range_pattern_period") == 2)
    result.check("at128_range_entries", len(at128.get("per_channel_min_ranges", [])) == 256,
                 f"got {len(at128.get('per_channel_min_ranges', []))}")

    qt128 = MODEL_REGISTRY["HesaiQT128C2X"]
    result.check("qt128_step_offsets", len(qt128.get("horizontal_step_offsets", [])) == 128)
    bank_cd = sum(1 for o in qt128.get("horizontal_step_offsets", []) if o > 0.1)
    result.check("qt128_bank_cd_count", bank_cd == 64, f"got {bank_cd}")

    p128hr = MODEL_REGISTRY["HesaiPandar128E4XHighRes"]
    result.check("p128hr_channels", p128hr["channels"] == 192)
    shifted = sum(1 for o in p128hr.get("horizontal_step_offsets", []) if o > 0)
    result.check("p128hr_shifted_count", shifted == 96, f"got {shifted}")

    # Noise preset data
    result.check("noise_registry_count", len(NOISE_REGISTRY) == 13,
                 f"got {len(NOISE_REGISTRY)}")
    for name in expected_models:
        result.check(f"noise_{name}", name in NOISE_REGISTRY)
    vlp16_noise = NOISE_REGISTRY["VelodyneVLP16"]
    result.check("noise_vlp16_type", vlp16_noise["angular_type"] == "ray")
    result.check("noise_vlp16_angular_stddev",
                 abs(vlp16_noise["angular_stddev"] - 0.05730) < 0.001,
                 f"got {vlp16_noise['angular_stddev']}")
    result.check("noise_vlp16_distance_base", vlp16_noise["distance_stddev_base"] == 0.02)

    # Beam divergence preset data
    result.check("bd_registry_count", len(BEAM_DIVERGENCE_REGISTRY) == 13,
                 f"got {len(BEAM_DIVERGENCE_REGISTRY)}")
    for name in expected_models:
        result.check(f"bd_{name}", name in BEAM_DIVERGENCE_REGISTRY)
    vlp16_bd = BEAM_DIVERGENCE_REGISTRY["VelodyneVLP16"]
    result.check("bd_vlp16_horizontal",
                 abs(vlp16_bd["horizontal"] - 0.171887) < 0.001,
                 f"got {vlp16_bd['horizontal']}")
    hesai_bd = BEAM_DIVERGENCE_REGISTRY["HesaiPandarQT"]
    result.check("bd_hesai_default",
                 abs(hesai_bd["horizontal"] - 0.13) < 0.001,
                 f"got {hesai_bd['horizontal']}")

    # ring_ids must be 0-based and contiguous (0..D-1 over distinct rings).
    # The ring_id is published verbatim as the ROS2 PointXYZIRCAEDT "channel"
    # field; Autoware's blockage diagnostic rejects channel >= vertical_bins
    # (== channel count), so a 1-based top ring (value == channels) throws
    # "Vertical bin is not valid" and aborts the perception container.
    # 0-based is grounded in manufacturer specs (Hesai/SICK number channels
    # 1-based -> driver subtracts 1; see docs/LidarSpecs) and the drivers that
    # publish "channel" 0-based: tier4/nebula (point.channel = channel_id 0..N-1,
    # elev_angle_map[laser_id-1]), velodyne_pointcloud/nebula (laser_ring 0-based),
    # ouster-ros (ring = row index 0..h-1), SICK sick_scan_xd (Row 0..23).
    # RangeMeter is a generic single-ray sample (no manufacturer) and follows the
    # same 0-based convention.
    for name, m in sorted(MODEL_REGISTRY.items()):
        rids = m.get("ring_ids", [])
        if not rids:
            continue
        distinct = sorted(set(rids))
        result.check(f"ring_ids_0based_{name}", min(rids) == 0,
                     f"min={min(rids)}")
        result.check(f"ring_ids_contiguous_{name}",
                     distinct == list(range(len(distinct))),
                     f"max={max(rids)} distinct={len(distinct)}")
        result.check(f"ring_ids_below_channels_{name}", max(rids) < m["channels"],
                     f"max={max(rids)} channels={m['channels']}")


def test_spawn_all(world, result):
    """Test spawning all 13 presets without crash."""
    print("\n--- Spawn tests (all presets) ---")

    for name in sorted(MODEL_REGISTRY.keys()):
        try:
            sensor = spawn_rgl_sensor(world, name)
            tick_and_destroy(world, sensor, ticks=3)
            result.ok(f"spawn_{name}")
        except Exception as e:
            result.fail(f"spawn_{name}", str(e))

    # Legacy mode (no preset)
    try:
        sensor = spawn_rgl_sensor(world, preset_name=None)
        tick_and_destroy(world, sensor, ticks=3)
        result.ok("spawn_legacy")
    except Exception as e:
        result.fail("spawn_legacy", str(e))


def test_masks(world, result):
    """Test ray mask functionality."""
    print("\n--- Mask tests ---")

    # Baseline
    sensor = spawn_rgl_sensor(world, "VelodyneVLP16")
    tick_and_destroy(world, sensor, ticks=5)
    result.ok("mask_baseline_spawn")

    # Azimuth FOV
    try:
        sensor = spawn_rgl_sensor(world, "VelodyneVLP16",
            extra_setup=lambda bp: set_azimuth_fov(bp, [[0, 180]]))
        tick_and_destroy(world, sensor)
        result.ok("mask_azimuth_spawn")
    except Exception as e:
        result.fail("mask_azimuth_spawn", str(e))

    # Ring mask
    try:
        sensor = spawn_rgl_sensor(world, "VelodyneVLP16",
            extra_setup=lambda bp: set_disabled_rings(bp, [0, 1]))
        tick_and_destroy(world, sensor)
        result.ok("mask_ring_spawn")
    except Exception as e:
        result.fail("mask_ring_spawn", str(e))

    # Rect mask
    try:
        sensor = spawn_rgl_sensor(world, "VelodyneVLP16",
            extra_setup=lambda bp: set_mask_rectangles(bp, [
                {"az_start": 170, "az_end": 190, "el_start": -30, "el_end": 30}]))
        tick_and_destroy(world, sensor)
        result.ok("mask_rect_spawn")
    except Exception as e:
        result.fail("mask_rect_spawn", str(e))

    # Raw mask
    try:
        sensor = spawn_rgl_sensor(world, "VelodyneVLP16",
            extra_setup=lambda bp: set_raw_mask(bp, [1, 0] * 8))
        tick_and_destroy(world, sensor)
        result.ok("mask_raw_spawn")
    except Exception as e:
        result.fail("mask_raw_spawn", str(e))

    # Combined mask
    try:
        def combo(bp):
            set_azimuth_fov(bp, [[0, 270]])
            set_disabled_rings(bp, [15])
        sensor = spawn_rgl_sensor(world, "VelodyneVLP16", extra_setup=combo)
        tick_and_destroy(world, sensor)
        result.ok("mask_combined_spawn")
    except Exception as e:
        result.fail("mask_combined_spawn", str(e))


def test_python_validation(result):
    """Test Python-side input validation."""
    print("\n--- Python validation tests ---")

    try:
        set_azimuth_fov(None, [[0, 120]] * 6)
        result.fail("validate_az_max5", "no error raised")
    except ValueError:
        result.ok("validate_az_max5")

    try:
        set_azimuth_fov(None, [[0, 400]])
        result.fail("validate_az_range", "no error raised")
    except ValueError:
        result.ok("validate_az_range")

    try:
        set_disabled_rings(None, [-1])
        result.fail("validate_ring_negative", "no error raised")
    except ValueError:
        result.ok("validate_ring_negative")

    try:
        set_mask_rectangles(None, [{"az_start": 0}])
        result.fail("validate_rect_missing_key", "no error raised")
    except ValueError:
        result.ok("validate_rect_missing_key")

    try:
        apply_preset(None, "NonExistentModel")
        result.fail("validate_unknown_model", "no error raised")
    except ValueError:
        result.ok("validate_unknown_model")

    # Noise validation
    try:
        set_noise(None, angular_type="invalid")
        result.fail("validate_noise_type", "no error raised")
    except ValueError:
        result.ok("validate_noise_type")

    try:
        set_noise(None, angular_stddev=-1)
        result.fail("validate_noise_neg_stddev", "no error raised")
    except ValueError:
        result.ok("validate_noise_neg_stddev")

    try:
        set_noise(None, distance_stddev_base=-0.1)
        result.fail("validate_noise_neg_dist", "no error raised")
    except ValueError:
        result.ok("validate_noise_neg_dist")

    # Beam divergence validation
    try:
        set_beam_divergence(None, 0.1, 0)
        result.fail("validate_bd_asymmetric", "no error raised")
    except ValueError:
        result.ok("validate_bd_asymmetric")

    try:
        set_beam_divergence(None, -0.1, -0.1)
        result.fail("validate_bd_negative", "no error raised")
    except ValueError:
        result.ok("validate_bd_negative")

    # Return mode validation
    try:
        set_return_mode(None, "invalid_mode")
        result.fail("validate_return_mode", "no error raised")
    except ValueError:
        result.ok("validate_return_mode")


def test_noise(world, result):
    """Test noise model spawn/tick/destroy."""
    print("\n--- Noise model tests ---")

    # Noise ON (preset default)
    try:
        sensor = spawn_rgl_sensor(world, "VelodyneVLP16")
        tick_and_destroy(world, sensor, ticks=3)
        result.ok("noise_on_default")
    except Exception as e:
        result.fail("noise_on_default", str(e))

    # Noise OFF (apply_noise=False)
    try:
        bp = world.get_blueprint_library().find("sensor.lidar.rgl")
        apply_preset(bp, "VelodyneVLP16", apply_noise=False)
        bp.set_attribute("sensor_tick", "0.1")
        sensor = world.spawn_actor(bp, carla.Transform(carla.Location(z=3)))
        tick_and_destroy(world, sensor, ticks=3)
        result.ok("noise_off")
    except Exception as e:
        result.fail("noise_off", str(e))

    # HitpointBased angular noise
    try:
        def setup_hp(bp):
            set_noise(bp, angular_type="hitpoint")
        sensor = spawn_rgl_sensor(world, "VelodyneVLP16", extra_setup=setup_hp)
        tick_and_destroy(world, sensor, ticks=3)
        result.ok("noise_hitpoint")
    except Exception as e:
        result.fail("noise_hitpoint", str(e))

    # disable_noise
    try:
        def setup_dis(bp):
            disable_noise(bp)
        sensor = spawn_rgl_sensor(world, "VelodyneVLP16", extra_setup=setup_dis)
        tick_and_destroy(world, sensor, ticks=3)
        result.ok("noise_disabled")
    except Exception as e:
        result.fail("noise_disabled", str(e))

    # Distance noise only
    try:
        def setup_dist(bp):
            disable_noise(bp)
            set_noise(bp, distance_stddev_base=0.05)
        sensor = spawn_rgl_sensor(world, "VelodyneVLP16", extra_setup=setup_dist)
        tick_and_destroy(world, sensor, ticks=3)
        result.ok("noise_distance_only")
    except Exception as e:
        result.fail("noise_distance_only", str(e))

    # X-axis angular noise
    try:
        def setup_axis(bp):
            set_noise(bp, angular_axis="X")
        sensor = spawn_rgl_sensor(world, "VelodyneVLP16", extra_setup=setup_axis)
        tick_and_destroy(world, sensor, ticks=3)
        result.ok("noise_axis_x")
    except Exception as e:
        result.fail("noise_axis_x", str(e))


def test_beam_divergence(world, result):
    """Test beam divergence spawn/tick/destroy."""
    print("\n--- Beam divergence tests ---")

    # BD OFF (default)
    try:
        sensor = spawn_rgl_sensor(world, "VelodyneVLP16")
        tick_and_destroy(world, sensor, ticks=3)
        result.ok("bd_off_default")
    except Exception as e:
        result.fail("bd_off_default", str(e))

    # BD ON via preset
    try:
        bp = world.get_blueprint_library().find("sensor.lidar.rgl")
        apply_preset(bp, "VelodyneVLP16", apply_beam_divergence=True)
        bp.set_attribute("sensor_tick", "0.1")
        sensor = world.spawn_actor(bp, carla.Transform(carla.Location(z=3)))
        tick_and_destroy(world, sensor, ticks=3)
        result.ok("bd_on_preset")
    except Exception as e:
        result.fail("bd_on_preset", str(e))

    # BD manual set
    try:
        def setup_manual(bp):
            set_beam_divergence(bp, 0.2, 0.1)
        sensor = spawn_rgl_sensor(world, "VelodyneVLP16", extra_setup=setup_manual)
        tick_and_destroy(world, sensor, ticks=3)
        result.ok("bd_manual")
    except Exception as e:
        result.fail("bd_manual", str(e))

    # disable_beam_divergence
    try:
        def setup_dis(bp):
            set_beam_divergence(bp, 0.2, 0.1)
            disable_beam_divergence(bp)
        sensor = spawn_rgl_sensor(world, "VelodyneVLP16", extra_setup=setup_dis)
        tick_and_destroy(world, sensor, ticks=3)
        result.ok("bd_disabled")
    except Exception as e:
        result.fail("bd_disabled", str(e))

    # Asymmetric input (server should warn, not crash)
    try:
        def setup_bad(bp):
            bp.set_attribute("beam_divergence_h", "0.1")
            bp.set_attribute("beam_divergence_v", "0.0")
        sensor = spawn_rgl_sensor(world, "VelodyneVLP16", extra_setup=setup_bad)
        tick_and_destroy(world, sensor, ticks=3)
        result.ok("bd_invalid_asymmetric", "no crash")
    except Exception as e:
        result.fail("bd_invalid_asymmetric", str(e))


def test_return_mode(world, result):
    """Test return mode spawn/tick/destroy."""
    print("\n--- Return mode tests ---")

    # Single return modes
    for mode in ["first", "second", "last", "strongest"]:
        try:
            def setup(bp, m=mode):
                set_return_mode(bp, m)
            sensor = spawn_rgl_sensor(world, "VelodyneVLP16", extra_setup=setup)
            tick_and_destroy(world, sensor, ticks=3)
            result.ok(f"return_{mode}")
        except Exception as e:
            result.fail(f"return_{mode}", str(e))

    # Dual return with beam divergence enabled
    try:
        def setup_dual(bp):
            set_beam_divergence(bp, 0.13, 0.13)
            set_return_mode(bp, "first_last")
        sensor = spawn_rgl_sensor(world, "VelodyneVLP16", extra_setup=setup_dual)
        tick_and_destroy(world, sensor, ticks=3)
        result.ok("return_first_last_with_bd")
    except Exception as e:
        result.fail("return_first_last_with_bd", str(e))

    # Dual return WITHOUT beam divergence (should fallback, not crash)
    try:
        def setup_dual_no_bd(bp):
            set_return_mode(bp, "first_last")
        sensor = spawn_rgl_sensor(world, "VelodyneVLP16", extra_setup=setup_dual_no_bd)
        tick_and_destroy(world, sensor, ticks=3)
        result.ok("return_dual_no_bd_fallback", "no crash")
    except Exception as e:
        result.fail("return_dual_no_bd_fallback", str(e))

    # Invalid return mode string (should fallback, not crash)
    try:
        def setup_bad(bp):
            bp.set_attribute("return_mode", "garbage")
        sensor = spawn_rgl_sensor(world, "VelodyneVLP16", extra_setup=setup_bad)
        tick_and_destroy(world, sensor, ticks=3)
        result.ok("return_invalid_string", "no crash")
    except Exception as e:
        result.fail("return_invalid_string", str(e))


def test_resilience_bad_mask(world, result):
    """Test that invalid mask strings don't crash the server."""
    print("\n--- Resilience: invalid mask strings ---")

    bad_cases = [
        ("bad_az_garbage",    {"ray_mask_azimuth": "garbage"}),
        ("bad_az_3values",    {"ray_mask_azimuth": "0,120,240"}),
        ("bad_az_negative",   {"ray_mask_azimuth": "-10,50"}),
        ("bad_ring_letters",  {"ray_mask_rings": "abc,def"}),
        ("bad_ring_float",    {"ray_mask_rings": "1.5,2.5"}),
        ("bad_rect_2values",  {"ray_mask_rects": "0,120"}),
        ("bad_rect_garbage",  {"ray_mask_rects": "a,b,c,d"}),
        ("bad_rect_semicol",  {"ray_mask_rects": ";;;"}),
        ("bad_combo",         {"ray_mask_azimuth": "xxx", "ray_mask_rings": "yyy",
                               "ray_mask_rects": "zzz"}),
    ]

    for name, attrs in bad_cases:
        try:
            def setup(bp, a=attrs):
                for k, v in a.items():
                    bp.set_attribute(k, v)
            sensor = spawn_rgl_sensor(world, "VelodyneVLP16", extra_setup=setup)
            tick_and_destroy(world, sensor, ticks=3)
            result.ok(f"resilience_{name}", "no crash")
        except Exception as e:
            result.fail(f"resilience_{name}", str(e))


def test_resilience_bad_encoding(world, result):
    """Test that corrupted delta+zlib+base64 data doesn't crash."""
    print("\n--- Resilience: corrupted encoded data ---")

    bad_cases = [
        ("bad_b64_garbage",     {"vertical_angles": "not_valid!!!"}),
        ("bad_b64_short",       {"vertical_angles": "AAAA"}),
        ("bad_b64_empty",       {"vertical_angles": ""}),
        ("bad_horiz_garbage",   {"horizontal_angle_offsets": "corrupt"}),
        ("bad_ringids_garbage",  {"ring_ids": "broken_data"}),
        ("bad_raw_mask",        {"ray_mask_raw": "invalid_base64"}),
    ]

    for name, attrs in bad_cases:
        try:
            def setup(bp, a=attrs):
                for k, v in a.items():
                    try:
                        bp.set_attribute(k, v)
                    except RuntimeError:
                        pass  # attribute may reject value
            sensor = spawn_rgl_sensor(world, "VelodyneVLP16", extra_setup=setup)
            tick_and_destroy(world, sensor, ticks=3)
            result.ok(f"resilience_{name}", "no crash, fallback OK")
        except Exception as e:
            result.fail(f"resilience_{name}", str(e))


def test_resilience_edge_params(world, result):
    """Test edge-case parameters don't crash."""
    print("\n--- Resilience: edge-case parameters ---")

    edge_cases = [
        ("zero_range",     {"range": "0.001"}),
        ("huge_range",     {"range": "99999"}),
        ("one_channel",    {"channels": "1", "upper_fov": "0", "lower_fov": "0"}),
        ("high_pps",       {"points_per_second": "10000000"}),
        ("low_pps",        {"points_per_second": "1"}),
        ("tiny_fov",       {"horizontal_fov": "1.0"}),
        ("zero_freq",      {"rotation_frequency": "0.001"}),
    ]

    for name, attrs in edge_cases:
        try:
            def setup(bp, a=attrs):
                for k, v in a.items():
                    bp.set_attribute(k, v)
            sensor = spawn_rgl_sensor(world, "VelodyneVLP16", extra_setup=setup)
            tick_and_destroy(world, sensor, ticks=3)
            result.ok(f"edge_{name}", "no crash")
        except Exception as e:
            result.fail(f"edge_{name}", str(e))


def test_ros2_point_verification(world, result):
    """Test ROS2 point cloud output with numerical verification."""
    print("\n--- ROS2 point verification tests ---")

    topic = "/test/regression"

    # VLP16 baseline
    def setup_ros2(bp):
        bp.set_attribute("rgl_lidar_topic_name", topic)
        bp.set_attribute("rgl_lidar_topic_frame_id", "lidar")
        bp.set_attribute("rgl_lidar_pointcloud_format", "PointXYZIRCAEDT")

    sensor = spawn_rgl_sensor(world, "VelodyneVLP16", extra_setup=setup_ros2)
    msg = capture_ros2_msg(world, sensor, topic)
    sensor.destroy(); world.tick()

    if msg is None:
        result.fail("ros2_vlp16_capture", "rclpy not available or no message")
        return

    result.check("ros2_vlp16_width", msg.width > 0, f"width={msg.width}")
    result.check("ros2_vlp16_point_step", msg.point_step == 32,
                 f"point_step={msg.point_step}")

    points = parse_points(msg)
    rings = set(p["ring_id"] for p in points if p["distance"] > 0)
    result.check("ros2_vlp16_rings", len(rings) > 0,
                 f"{len(rings)} unique rings")

    # AT128 per-channel range at z=6.4m (pitch=-90, down)
    sensor = spawn_rgl_sensor(world, "HesaiAT128E2X",
        extra_setup=setup_ros2,
        location=carla.Location(z=6.4),
        rotation=carla.Rotation(pitch=-90))
    msg = capture_ros2_msg(world, sensor, topic)
    sensor.destroy(); world.tick()

    if msg:
        m = MODEL_REGISTRY["HesaiAT128E2X"]
        nf_rings = set()
        for ch in range(128):
            if m["per_channel_min_ranges"][ch * 2] < 1.0:
                nf_rings.add(m["ring_ids"][ch])

        nf_near = 0; nnf_near = 0
        for p in parse_points(msg):
            if p["distance"] <= 0:
                continue
            if p["ring_id"] in nf_rings:
                if p["distance"] < 7.2:
                    nf_near += 1
            else:
                if p["distance"] < 7.2:
                    nnf_near += 1

        result.check("ros2_at128_nf_detects_ground", nf_near > 0,
                     f"NF near={nf_near}")
        result.check("ros2_at128_non_nf_no_ground", nnf_near == 0,
                     f"non-NF near={nnf_near}")

    # Azimuth mask [0,180] should halve points
    def setup_az_mask(bp):
        setup_ros2(bp)
        set_azimuth_fov(bp, [[0, 180]])
    sensor = spawn_rgl_sensor(world, "VelodyneVLP16", extra_setup=setup_az_mask)
    msg_masked = capture_ros2_msg(world, sensor, topic)
    sensor.destroy(); world.tick()

    if msg_masked and msg:
        masked_hits = sum(1 for p in parse_points(msg_masked) if p["distance"] > 0)
        base_hits = sum(1 for p in parse_points(msg) if p["distance"] > 0)
        ratio = masked_hits / max(base_hits, 1) if base_hits > 0 else 0
        result.check("ros2_azimuth_mask_ratio", 0.3 < ratio < 0.7,
                     f"ratio={ratio:.2f}")

    # Ring mask (disable ring 0)
    def setup_ring_mask(bp):
        setup_ros2(bp)
        set_disabled_rings(bp, [0])
    sensor = spawn_rgl_sensor(world, "VelodyneVLP16", extra_setup=setup_ring_mask)
    msg_ring = capture_ros2_msg(world, sensor, topic)
    sensor.destroy(); world.tick()

    if msg_ring:
        rings_seen = set(p["ring_id"] for p in parse_points(msg_ring) if p["distance"] > 0)
        result.check("ros2_ring_mask_excludes_0", 0 not in rings_seen,
                     f"rings={sorted(rings_seen)}")

    # Dual return: point count and return_type verification
    def setup_single_ros2(bp):
        setup_ros2(bp)
        set_beam_divergence(bp, 0.13, 0.13)
        set_return_mode(bp, "first")
    sensor = spawn_rgl_sensor(world, "VelodyneVLP16", extra_setup=setup_single_ros2)
    msg_single = capture_ros2_msg(world, sensor, topic)
    sensor.destroy(); world.tick()

    def setup_dual_ros2(bp):
        setup_ros2(bp)
        set_beam_divergence(bp, 0.13, 0.13)
        set_return_mode(bp, "first_last")
    sensor = spawn_rgl_sensor(world, "VelodyneVLP16", extra_setup=setup_dual_ros2)
    msg_dual = capture_ros2_msg(world, sensor, topic)
    sensor.destroy(); world.tick()

    if msg_single and msg_dual:
        single_hits = sum(1 for p in parse_points(msg_single) if p["distance"] > 0)
        dual_hits = sum(1 for p in parse_points(msg_dual) if p["distance"] > 0)
        if single_hits > 0:
            ratio = dual_hits / single_hits
            result.check("ros2_dual_point_ratio", 1.3 < ratio < 2.1,
                         f"dual={dual_hits} single={single_hits} ratio={ratio:.2f}")
        else:
            result.fail("ros2_dual_point_ratio", "single_hits=0")

        dual_points = parse_points(msg_dual)
        return_types = set(p["return_type"] for p in dual_points if p["distance"] > 0)
        # RGL_RETURN_TYPE_FIRST=4, RGL_RETURN_TYPE_LAST=2
        result.check("ros2_dual_has_first", 4 in return_types,
                     f"return_types={return_types}")
        result.check("ros2_dual_has_last", 2 in return_types,
                     f"return_types={return_types}")
    elif msg_single is None:
        result.fail("ros2_dual_point_ratio", "rclpy not available or no single message")
    else:
        result.fail("ros2_dual_point_ratio", "no dual message received")


# ============================================================================
# Main
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description="RGL LiDAR preset regression tests")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--ros2", action="store_true",
                        help="Run ROS2 point cloud verification tests")
    parser.add_argument("--test", default="all",
                        choices=["all", "encoding", "data", "spawn", "mask",
                                 "noise", "beam_divergence", "return_mode",
                                 "validation", "resilience", "ros2"],
                        help="Run specific test category")
    args = parser.parse_args()

    result = TestResult()

    # Offline tests (no CARLA needed)
    if args.test in ("all", "encoding"):
        test_encoding(result)

    if args.test in ("all", "data"):
        test_preset_data(result)

    if args.test in ("all", "validation"):
        test_python_validation(result)

    # Online tests (need CARLA server)
    need_carla = args.test in ("all", "spawn", "mask", "noise", "beam_divergence",
                                "return_mode", "resilience", "ros2")
    if need_carla:
        print("\nConnecting to CARLA...")
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)
        world = client.get_world()
        print(f"Connected: map={world.get_map().name}")
        setup_sync(world)

        if args.test in ("all", "spawn"):
            test_spawn_all(world, result)

        if args.test in ("all", "mask"):
            test_masks(world, result)

        if args.test in ("all", "noise"):
            test_noise(world, result)

        if args.test in ("all", "beam_divergence"):
            test_beam_divergence(world, result)

        if args.test in ("all", "return_mode"):
            test_return_mode(world, result)

        if args.test in ("all", "resilience"):
            test_resilience_bad_mask(world, result)
            test_resilience_bad_encoding(world, result)
            test_resilience_edge_params(world, result)

        if args.ros2 or args.test == "ros2":
            try:
                import rclpy
                test_ros2_point_verification(world, result)
            except ImportError:
                print("\n--- ROS2 tests skipped (rclpy not available) ---")
                print("  Run with ROS2 environment sourced for full tests")

        teardown_sync(world)

    all_passed = result.summary()
    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()
