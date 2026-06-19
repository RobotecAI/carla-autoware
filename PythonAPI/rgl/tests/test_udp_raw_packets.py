#!/usr/bin/env python3
"""UDP Raw Packet Phase 1 verification test.

Verifies that the CARLA RGL backend emits Velodyne/Hesai UDP raw packets
when the optional RGL UDP extension is available. The test self-skips on
builds where libRobotecGPULidar.so was compiled without the UDP extension.

Usage:
    # Start CARLA server in another terminal first, then:
    python3 test_udp_raw_packets.py

Exit code 0 on full success, 1 on any failure, 2 if UDP extension absent
(treated as skip — not a failure).
"""

import os
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import carla
from lidar_models import apply_preset


# ----------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------

CARLA_HOST = "127.0.0.1"
CARLA_PORT = 2000
DEST_IP = "127.0.0.1"
DEST_PORT_BASE = 2368  # parametrised tests increment this per model

ALL_MODELS = [
    "VelodyneVLP16", "VelodyneVLP32C", "VelodyneVLS128",
    "HesaiPandar40P", "HesaiPandarQT", "HesaiPandarXT32",
    "HesaiQT128C2X", "HesaiPandar128E4X",
]


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


def make_udp_listener(port, buffer_bytes=2 * 1024 * 1024):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, buffer_bytes)
    except OSError:
        pass
    sock.bind(("0.0.0.0", port))
    sock.settimeout(2.0)
    return sock


def recv_packets(sock, duration_sec):
    """Drain UDP packets from the socket for up to duration_sec."""
    packets = []
    deadline = time.time() + duration_sec
    while time.time() < deadline:
        try:
            data, _ = sock.recvfrom(2048)
            packets.append(data)
        except socket.timeout:
            break
    return packets


# Models whose UDP raw packet format whitelist excludes "first":
# Velodyne (Legacy Packet Format) and Hesai Pandar40P/XT32 require
# strongest/last/last_strongest. CARLA's default return_mode is "first",
# which would otherwise cause the C++ backend to reject UDP for these
# models. Hesai QT128C2X / Pandar128E4X / PandarQT accept "first" natively.
_NO_FIRST_MODELS = {
    "VelodyneVLP16", "VelodyneVLP32C", "VelodyneVLS128",
    "HesaiPandar40P", "HesaiPandarXT32",
}


def spawn_sensor(world, preset_name, port, extra_kwargs=None):
    bp = world.get_blueprint_library().find("sensor.lidar.rgl")
    kwargs = {"udp_publish": {"dest_ip": DEST_IP, "dest_port": port}}
    if extra_kwargs:
        kwargs.update(extra_kwargs)
    apply_preset(bp, preset_name, **kwargs)
    # Override default return_mode for models that do not accept "first"
    # under the UDP whitelist. Tests assume the simplest valid mode.
    if preset_name in _NO_FIRST_MODELS:
        bp.set_attribute("return_mode", "strongest")
    spawn_point = world.get_map().get_spawn_points()[0]
    return world.spawn_actor(bp, spawn_point)


# ----------------------------------------------------------------------------
# Velodyne Legacy Packet Format decoder
# Spec reference: Velodyne VLP-16 User Manual (63-9243 Rev E)
# 1206-byte packet = 12 firing blocks (100 B each) + 6 B tail
# ----------------------------------------------------------------------------

VLP_PACKET_SIZE = 1206
VLP_FIRING_FLAG = 0xEEFF
VLP_PRODUCT_ID_VLP16 = 0x22
VLP_RETURN_MODE_STRONGEST = 0x37
VLP_RETURN_MODE_LAST = 0x38
VLP_RETURN_MODE_DUAL = 0x39


def decode_vlp_packet(pkt):
    """Validate one VLP16 Legacy Format packet. Returns (azimuths, timestamp_us)
    on success or raises AssertionError.

    RGL's UDP extension emits one rotation-boundary packet every ~38 packets
    that has blocks 0..N populated (N<12) and blocks N..11 all-zero padded.
    This decoder accepts those by stopping validation at the first all-zero
    flag and verifying the remaining bytes are pure padding.
    """
    assert len(pkt) == VLP_PACKET_SIZE, \
        f"packet size {len(pkt)} != {VLP_PACKET_SIZE}"

    azimuths = []
    rotation_boundary_at = None  # index of first zero-padding block, or None

    for fseq in range(12):
        offset = fseq * 100
        flag = struct.unpack_from("<H", pkt, offset)[0]

        if flag == 0:
            # Rotation-boundary padding starts here. Verify rest of this 100-byte
            # block is fully zero, then break (remaining blocks must also be all-zero).
            assert pkt[offset:offset + 100] == b"\x00" * 100, \
                f"firing block {fseq} has zero flag but non-zero payload"
            rotation_boundary_at = fseq
            break

        assert flag == VLP_FIRING_FLAG, \
            f"firing block {fseq} flag mismatch: {flag:#06x}"

        az_raw = struct.unpack_from("<H", pkt, offset + 2)[0]
        az_deg = az_raw / 100.0
        assert 0.0 <= az_deg < 360.0, \
            f"firing block {fseq} azimuth out of range: {az_deg}"
        azimuths.append(az_deg)

        for ch in range(32):
            d_raw = struct.unpack_from("<H", pkt, offset + 4 + ch * 3)[0]
            distance_m = d_raw * 0.002  # 2 mm units
            # 0 means no-hit; positive values must respect Legacy Format cap.
            assert 0.0 <= distance_m <= 262.14, \
                f"firing block {fseq} ch {ch} distance out of range: {distance_m}"

    # If we hit a zero-padding boundary, verify all subsequent blocks are also
    # all-zero (no partially-corrupted blocks past the boundary).
    if rotation_boundary_at is not None:
        pad_start = rotation_boundary_at * 100
        pad_end = 12 * 100  # block region ends at byte 1200
        assert pkt[pad_start:pad_end] == b"\x00" * (pad_end - pad_start), \
            f"non-zero bytes found in rotation-padding region " \
            f"(blocks {rotation_boundary_at}..11)"

    timestamp_us = struct.unpack_from("<I", pkt, 1200)[0]
    return_mode = pkt[1204]
    product_id = pkt[1205]
    assert product_id == VLP_PRODUCT_ID_VLP16, \
        f"product id {product_id:#04x} != VLP16 ({VLP_PRODUCT_ID_VLP16:#04x})"
    assert return_mode in (VLP_RETURN_MODE_STRONGEST, VLP_RETURN_MODE_LAST,
                           VLP_RETURN_MODE_DUAL), \
        f"unexpected return mode {return_mode:#04x}"

    return azimuths, timestamp_us


# ----------------------------------------------------------------------------
# UDP extension availability probe
# ----------------------------------------------------------------------------

def check_udp_extension(world):
    """Best-effort probe: spawn a UDP-enabled sensor and look for emitted
    packets. Returns True if any packet arrived within ~1 second."""
    sock = make_udp_listener(DEST_PORT_BASE + 99)
    try:
        sensor = spawn_sensor(world, "VelodyneVLP16", DEST_PORT_BASE + 99)
        try:
            for _ in range(20):
                world.tick()
            pkts = recv_packets(sock, 0.5)
        finally:
            sensor.destroy()
        return len(pkts) > 0
    finally:
        sock.close()


# ----------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------

def test_smoke_all_models(world):
    """Each model emits >=1 UDP packet within 1 second."""
    failures = []
    for idx, model in enumerate(ALL_MODELS):
        port = DEST_PORT_BASE + idx
        sock = make_udp_listener(port)
        try:
            sensor = spawn_sensor(world, model, port)
            try:
                for _ in range(20):
                    world.tick()
                pkts = recv_packets(sock, 0.5)
            finally:
                sensor.destroy()
            if not pkts:
                failures.append(f"{model}: no packets on :{port}")
                print(f"  FAIL: {model}: no packets received on :{port}")
            else:
                print(f"  PASS: {model}: {len(pkts)} packets on :{port}")
        finally:
            sock.close()
    assert not failures, "Smoke test failures: " + "; ".join(failures)


def test_vlp16_deep_validation(world):
    """VLP16 packets decode cleanly and azimuths sweep monotonically."""
    port = DEST_PORT_BASE + 20
    sock = make_udp_listener(port)
    try:
        sensor = spawn_sensor(world, "VelodyneVLP16", port)
        try:
            for _ in range(40):
                world.tick()
            pkts = recv_packets(sock, 1.0)
        finally:
            sensor.destroy()

        assert len(pkts) >= 10, f"expected >=10 packets, got {len(pkts)}"

        # All packets pass the decoder.
        all_az = []
        for pkt in pkts:
            azimuths, ts = decode_vlp_packet(pkt)
            all_az.append((azimuths, ts))

        # Within each packet, azimuths must be non-decreasing modulo wrap.
        # Rotation-boundary packets may have only a few populated blocks; skip
        # those without enough data to compare.
        for i, (azimuths, _) in enumerate(all_az):
            for j in range(1, len(azimuths)):
                diff = (azimuths[j] - azimuths[j - 1]) % 360.0
                assert diff < 30.0, \
                    f"packet {i} block {j} az jumps by {diff} deg"

        # Timestamps non-decreasing across packets.
        for i in range(1, len(all_az)):
            assert all_az[i][1] >= all_az[i - 1][1], \
                f"timestamp went backwards at packet {i}"

        print(f"  PASS: VLP16 deep validation: {len(pkts)} packets decoded")
    finally:
        sock.close()


def test_hesai_hfov_start_offset(world):
    """HesaiPandar40P with hesai_ros_driver_compat shifts the sweep start.
    Use UDP packet flow as a proxy: with the shift active, packets still
    arrive (no regression) and we additionally verify horizontal_start_angle
    was applied by reading the attribute back."""
    port = DEST_PORT_BASE + 30
    sock = make_udp_listener(port)
    try:
        bp = world.get_blueprint_library().find("sensor.lidar.rgl")
        apply_preset(bp, "HesaiPandar40P",
                     hesai_ros_driver_compat=True,
                     udp_publish={"dest_ip": DEST_IP, "dest_port": port,
                                  "ensure_hesai_pandar_driver_compat": True})
        # HesaiPandar40P UDP whitelist requires strongest/last/last_strongest.
        # See _NO_FIRST_MODELS in spawn_sensor() for rationale.
        bp.set_attribute("return_mode", "strongest")
        start_attr = bp.get_attribute("horizontal_start_angle")
        start_val = start_attr.as_float()
        assert start_val == -90.0, \
            f"horizontal_start_angle should be -90 after hesai_ros_driver_compat, got {start_val}"
        spawn_point = world.get_map().get_spawn_points()[0]
        sensor = world.spawn_actor(bp, spawn_point)
        try:
            for _ in range(40):
                world.tick()
            pkts = recv_packets(sock, 1.0)
        finally:
            sensor.destroy()
        assert len(pkts) >= 5, f"Hesai sweep shift broke flow: {len(pkts)} packets"
        print(f"  PASS: Hesai HFOV start offset: {len(pkts)} packets")
    finally:
        sock.close()


# ----------------------------------------------------------------------------
# Main entry
# ----------------------------------------------------------------------------

def main():
    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(10.0)
    world = client.get_world()

    setup_sync(world)
    try:
        if not check_udp_extension(world):
            print("SKIP: RGL UDP extension not detected in libRobotecGPULidar.so. "
                  "Rebuild RGL with RGL_BUILD_UDP_EXTENSION=ON to enable.")
            return 2

        failures = []
        for fn in (test_smoke_all_models, test_vlp16_deep_validation,
                   test_hesai_hfov_start_offset):
            try:
                print(f"\n=== {fn.__name__} ===")
                fn(world)
            except AssertionError as e:
                failures.append(f"{fn.__name__}: {e}")
                print(f"  FAIL: {e}")

        print("\n" + "=" * 60)
        if failures:
            print(f"FAILED ({len(failures)}):")
            for f in failures:
                print(f"  - {f}")
            return 1
        print("ALL TESTS PASSED")
        return 0
    finally:
        teardown_sync(world)


if __name__ == "__main__":
    sys.exit(main())
