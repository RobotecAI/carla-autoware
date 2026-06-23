"""AWSIM-compatible LiDAR presets for CARLA RGL.

Usage:
    from rgl_lidar_models import apply_preset, list_models

    list_models()  # prints available model names
    blueprint = world.get_blueprint_library().find("sensor.lidar.rgl")
    apply_preset(blueprint, "VelodyneVLP16")
"""

import base64
import struct
import zlib

from .range_meter import MODEL as _RangeMeter
from .sick_mrs6000 import MODEL as _SickMRS6000
from .velodyne_vlp16 import MODEL as _VelodyneVLP16
from .velodyne_vlp32c import MODEL as _VelodyneVLP32C
from .velodyne_vls128 import MODEL as _VelodyneVLS128
from .hesai_pandar40p import MODEL as _HesaiPandar40P
from .hesai_pandarqt import MODEL as _HesaiPandarQT
from .hesai_pandarxt32 import MODEL as _HesaiPandarXT32
from .hesai_at128e2x import MODEL as _HesaiAT128E2X
from .hesai_qt128c2x import MODEL as _HesaiQT128C2X
from .hesai_pandar128e4x import MODEL as _HesaiPandar128E4X
from .hesai_pandar128e4x_highres import MODEL as _HesaiPandar128E4XHighRes
from .ouster_os1_64 import MODEL as _OusterOS1_64

from .range_meter import NOISE as _NoiseRangeMeter
from .sick_mrs6000 import NOISE as _NoiseSickMRS6000
from .velodyne_vlp16 import NOISE as _NoiseVelodyneVLP16
from .velodyne_vlp32c import NOISE as _NoiseVelodyneVLP32C
from .velodyne_vls128 import NOISE as _NoiseVelodyneVLS128
from .hesai_pandar40p import NOISE as _NoiseHesaiPandar40P
from .hesai_pandarqt import NOISE as _NoiseHesaiPandarQT
from .hesai_pandarxt32 import NOISE as _NoiseHesaiPandarXT32
from .hesai_at128e2x import NOISE as _NoiseHesaiAT128E2X
from .hesai_qt128c2x import NOISE as _NoiseHesaiQT128C2X
from .hesai_pandar128e4x import NOISE as _NoiseHesaiPandar128E4X
from .hesai_pandar128e4x_highres import NOISE as _NoiseHesaiPandar128E4XHighRes
from .ouster_os1_64 import NOISE as _NoiseOusterOS1_64

from .range_meter import BEAM_DIVERGENCE as _BDRangeMeter
from .sick_mrs6000 import BEAM_DIVERGENCE as _BDSickMRS6000
from .velodyne_vlp16 import BEAM_DIVERGENCE as _BDVelodyneVLP16
from .velodyne_vlp32c import BEAM_DIVERGENCE as _BDVelodyneVLP32C
from .velodyne_vls128 import BEAM_DIVERGENCE as _BDVelodyneVLS128
from .hesai_pandar40p import BEAM_DIVERGENCE as _BDHesaiPandar40P
from .hesai_pandarqt import BEAM_DIVERGENCE as _BDHesaiPandarQT
from .hesai_pandarxt32 import BEAM_DIVERGENCE as _BDHesaiPandarXT32
from .hesai_at128e2x import BEAM_DIVERGENCE as _BDHesaiAT128E2X
from .hesai_qt128c2x import BEAM_DIVERGENCE as _BDHesaiQT128C2X
from .hesai_pandar128e4x import BEAM_DIVERGENCE as _BDHesaiPandar128E4X
from .hesai_pandar128e4x_highres import BEAM_DIVERGENCE as _BDHesaiPandar128E4XHighRes
from .ouster_os1_64 import BEAM_DIVERGENCE as _BDOusterOS1_64

MODEL_REGISTRY = {m["name"]: m for m in [
    _RangeMeter, _SickMRS6000,
    _VelodyneVLP16, _VelodyneVLP32C, _VelodyneVLS128,
    _HesaiPandar40P, _HesaiPandarQT, _HesaiPandarXT32,
    _HesaiAT128E2X, _HesaiQT128C2X, _HesaiPandar128E4X,
    _HesaiPandar128E4XHighRes, _OusterOS1_64,
]}

NOISE_REGISTRY = {
    "RangeMeter": _NoiseRangeMeter,
    "SickMRS6000": _NoiseSickMRS6000,
    "VelodyneVLP16": _NoiseVelodyneVLP16,
    "VelodyneVLP32C": _NoiseVelodyneVLP32C,
    "VelodyneVLS128": _NoiseVelodyneVLS128,
    "HesaiPandar40P": _NoiseHesaiPandar40P,
    "HesaiPandarQT": _NoiseHesaiPandarQT,
    "HesaiPandarXT32": _NoiseHesaiPandarXT32,
    "HesaiAT128E2X": _NoiseHesaiAT128E2X,
    "HesaiQT128C2X": _NoiseHesaiQT128C2X,
    "HesaiPandar128E4X": _NoiseHesaiPandar128E4X,
    "HesaiPandar128E4XHighRes": _NoiseHesaiPandar128E4XHighRes,
    "OusterOS1_64": _NoiseOusterOS1_64,
}

BEAM_DIVERGENCE_REGISTRY = {
    "RangeMeter": _BDRangeMeter,
    "SickMRS6000": _BDSickMRS6000,
    "VelodyneVLP16": _BDVelodyneVLP16,
    "VelodyneVLP32C": _BDVelodyneVLP32C,
    "VelodyneVLS128": _BDVelodyneVLS128,
    "HesaiPandar40P": _BDHesaiPandar40P,
    "HesaiPandarQT": _BDHesaiPandarQT,
    "HesaiPandarXT32": _BDHesaiPandarXT32,
    "HesaiAT128E2X": _BDHesaiAT128E2X,
    "HesaiQT128C2X": _BDHesaiQT128C2X,
    "HesaiPandar128E4X": _BDHesaiPandar128E4X,
    "HesaiPandar128E4XHighRes": _BDHesaiPandar128E4XHighRes,
    "OusterOS1_64": _BDOusterOS1_64,
}


def _encode_float_array(floats):
    """Encode float array as delta-int32(×1e6) + zlib + base64.

    CARLA's actor attribute values pass through UE5's FName, which has a hard
    limit of 1023 characters.  A plain comma-separated text representation of
    128-channel float32 angles (e.g. HesaiQT128C2X) exceeds this limit at
    ~1600 chars, making it impossible to transmit per-channel data at full
    float32 precision without compression.

    Encoding pipeline:
      float × 1e6 → int32 (fixed-point, 0.000001° precision ≈ float32)
      → delta encoding (exploits near-monotonic angle sequences)
      → zlib level-9 compression
      → base64 (ASCII-safe for attribute transport)

    Result: 128ch worst case ≈ 572 chars, well within the 1023 char limit.
    """
    ints = [round(f * 1e6) for f in floats]
    deltas = [ints[0]] + [ints[i] - ints[i - 1] for i in range(1, len(ints))]
    raw = struct.pack(f">{len(deltas)}i", *deltas)
    compressed = zlib.compress(raw, 9)
    # Prepend 4-byte big-endian uncompressed size for C++ FCompression::UncompressMemory
    # which requires the exact output size.
    payload = struct.pack(">I", len(raw)) + compressed
    return base64.b64encode(payload).decode("ascii")


def _encode_int_array(ints):
    """Encode int array as delta-int32 + zlib + base64.

    Same encoding as _encode_float_array but without the ×1e6 scaling.
    See _encode_float_array docstring for rationale (FName 1023 char limit).
    """
    deltas = [ints[0]] + [ints[i] - ints[i - 1] for i in range(1, len(ints))]
    raw = struct.pack(f">{len(deltas)}i", *deltas)
    compressed = zlib.compress(raw, 9)
    payload = struct.pack(">I", len(raw)) + compressed
    return base64.b64encode(payload).decode("ascii")


def list_models():
    """Print all available LiDAR model names."""
    for name, m in sorted(MODEL_REGISTRY.items()):
        print(f"  {name:24s} {m['channels']:>3d}ch  range={m['min_range']}-{m['range']}m  "
              f"H-FOV={m['horizontal_fov']}")


def apply_preset(
    blueprint,
    model_name,
    apply_noise=True,
    apply_beam_divergence=False,
    hesai_ros_driver_compat=None,
    udp_publish=None,
):
    """Apply a LiDAR preset to a CARLA blueprint.

    Args:
        blueprint: carla.ActorBlueprint for sensor.lidar.rgl
        model_name: one of the names from list_models() (e.g. "VelodyneVLP16")
        apply_noise: if True (default), apply preset noise params.
            Set to False for deterministic (noise-free) operation.
        apply_beam_divergence: if True, apply preset beam divergence params.
            Default False (58x ray cost when enabled).
        hesai_ros_driver_compat: controls the Hesai ROS driver sweep-start
            convention (horizontal_start_angle = -90.0).
            None (default): auto — enabled for Hesai models, disabled for
                non-Hesai models (no warning). This means Hesai models get
                the -90 deg sweep start by default without any explicit opt-in.
            True: force-enable. For Hesai models sets -90.0; for non-Hesai
                models emits a warnings.warn() and does nothing else.
            False: force-disable. Never sets horizontal_start_angle,
                regardless of model. Use this to opt out on Hesai models.
        udp_publish: dict configuring AWSIM-style UDP Raw Packet publishing.
            None (default) disables UDP. When provided, supported keys:
              dest_ip                                  (str, required to enable)
              dest_port                                (int,  default 2368)
              source_ip                                (str,  default "0.0.0.0")
              enable_hesai_udp_sequence                (bool, default False)
              enable_hesai_blockage_detection          (bool, default False)
              ensure_hesai_pandar_driver_compat        (bool, default False)
    """
    if model_name not in MODEL_REGISTRY:
        available = ", ".join(sorted(MODEL_REGISTRY.keys()))
        raise ValueError(f"Unknown model '{model_name}'. Available: {available}")

    import warnings as _warnings

    # Always propagate the model name so the C++ side can map to RGL enum
    # (cheap; harmless when UDP is disabled).
    try:
        blueprint.set_attribute("rgl_lidar_model_name", model_name)
    except RuntimeError:
        pass  # attribute not available in older builds

    # Hesai ROS driver coordinate compatibility (sets sweep start to -90 deg).
    is_hesai = model_name.startswith("Hesai")
    # Auto-default: enabled for Hesai models by default (None), so the sweep
    # start aligns with the Hesai ROS driver convention without the caller
    # opting in. Explicit True/False override the auto-default.
    effective_compat = is_hesai if hesai_ros_driver_compat is None else hesai_ros_driver_compat
    if effective_compat:
        if is_hesai:
            try:
                blueprint.set_attribute("horizontal_start_angle", "-90.0")
            except RuntimeError:
                pass
        else:
            _warnings.warn(
                f"hesai_ros_driver_compat=True is ignored for non-Hesai "
                f"model '{model_name}'",
                stacklevel=2,
            )

    m = MODEL_REGISTRY[model_name]

    blueprint.set_attribute("channels", str(m["channels"]))
    blueprint.set_attribute("range", str(m["range"]))
    blueprint.set_attribute("upper_fov", str(m["upper_fov"]))
    blueprint.set_attribute("lower_fov", str(m["lower_fov"]))
    blueprint.set_attribute("horizontal_fov", str(m["horizontal_fov"]))
    blueprint.set_attribute("rotation_frequency", str(m["rotation_frequency"]))
    blueprint.set_attribute("points_per_second", str(m["points_per_second"]))

    # These attributes require the RGL preset C++ changes.
    # Use try/except so presets degrade gracefully on older builds.
    def _try_set(attr, val):
        try:
            blueprint.set_attribute(attr, val)
        except RuntimeError:
            pass  # attribute not available in this build

    if m.get("min_range", 0) > 0:
        _try_set("min_range", str(m["min_range"]))

    # Encode arrays as delta+zlib+base64 to stay within UE5 FName 1023 char limit.
    if m["vertical_angles"]:
        _try_set("vertical_angles", _encode_float_array(m["vertical_angles"]))

    if m["horizontal_angle_offsets"]:
        _try_set("horizontal_angle_offsets", _encode_float_array(m["horizontal_angle_offsets"]))

    if m["ring_ids"]:
        _try_set("ring_ids", _encode_int_array(m["ring_ids"]))

    if m.get("per_channel_min_ranges"):
        _try_set("per_channel_min_ranges",
                 _encode_float_array(m["per_channel_min_ranges"]))
        _try_set("per_channel_max_ranges",
                 _encode_float_array(m["per_channel_max_ranges"]))
        _try_set("range_pattern_period",
                 str(m.get("range_pattern_period", 0)))

    if m.get("horizontal_step_offsets"):
        _try_set("horizontal_step_offsets",
                 _encode_float_array(m["horizontal_step_offsets"]))

    # Noise parameters
    if apply_noise and model_name in NOISE_REGISTRY:
        noise = NOISE_REGISTRY[model_name]
        _try_set("noise_angular_type", noise["angular_type"])
        _try_set("noise_angular_mean", str(noise["angular_mean"]))
        _try_set("noise_angular_stddev", str(noise["angular_stddev"]))
        _try_set("noise_distance_mean", str(noise["distance_mean"]))
        _try_set("noise_distance_stddev_base", str(noise["distance_stddev_base"]))
        _try_set("noise_distance_stddev_rise", str(noise["distance_stddev_rise"]))
        _try_set("noise_angular_axis", noise["angular_axis"])

    # Beam divergence
    if apply_beam_divergence and model_name in BEAM_DIVERGENCE_REGISTRY:
        bd = BEAM_DIVERGENCE_REGISTRY[model_name]
        _try_set("beam_divergence_h", str(bd["horizontal"]))
        _try_set("beam_divergence_v", str(bd["vertical"]))

    # UDP Raw Packet publishing (AWSIM LidarUdpPublisher port)
    if udp_publish is not None:
        dest_ip = udp_publish.get("dest_ip", "")
        if dest_ip:
            def _set_udp(attr, val):
                try:
                    blueprint.set_attribute(attr, val)
                except RuntimeError:
                    _warnings.warn(
                        f"UDP attribute '{attr}' unavailable in this build; "
                        f"UDP publishing will be inactive.",
                        stacklevel=2,
                    )

            _set_udp("rgl_udp_enabled", "true")
            _set_udp("rgl_udp_dest_ip", dest_ip)
            _set_udp("rgl_udp_dest_port",
                     str(udp_publish.get("dest_port", 2368)))
            _set_udp("rgl_udp_source_ip",
                     udp_publish.get("source_ip", "0.0.0.0"))
            _set_udp("rgl_udp_hesai_enable_udp_sequence",
                     "true" if udp_publish.get("enable_hesai_udp_sequence",
                                               False) else "false")
            _set_udp("rgl_udp_hesai_blockage_detection",
                     "true" if udp_publish.get("enable_hesai_blockage_detection",
                                               False) else "false")
            _set_udp("rgl_udp_hesai_pandar_driver_compat",
                     "true" if udp_publish.get("ensure_hesai_pandar_driver_compat",
                                               False) else "false")


def set_azimuth_fov(blueprint, sections):
    """Set azimuth FOV sections (whitelist). Rays outside all sections are masked.

    Args:
        blueprint: carla.ActorBlueprint for sensor.lidar.rgl
        sections: list of [start, end] pairs (degrees, 0-360).
            Max 5 sections. Wrap-around supported (start > end).
    Example:
        set_azimuth_fov(bp, [[0, 120], [240, 360]])
    """
    if len(sections) > 5:
        raise ValueError("Maximum 5 azimuth sections supported")
    for s, e in sections:
        if not (0 <= s <= 360 and 0 <= e <= 360):
            raise ValueError(f"Azimuth must be 0-360, got ({s}, {e})")
    blueprint.set_attribute("ray_mask_azimuth",
                            ";".join(f"{s},{e}" for s, e in sections))


def set_disabled_rings(blueprint, ring_ids):
    """Disable specific ring IDs (fault injection).

    Args:
        blueprint: carla.ActorBlueprint for sensor.lidar.rgl
        ring_ids: list of ring IDs to disable
    """
    for r in ring_ids:
        if not isinstance(r, int) or r < 0:
            raise ValueError(f"Ring ID must be non-negative integer, got {r}")
    blueprint.set_attribute("ray_mask_rings",
                            ",".join(str(r) for r in ring_ids))


def set_mask_rectangles(blueprint, rects):
    """Mask rectangular regions (self-occlusion).

    Args:
        blueprint: carla.ActorBlueprint for sensor.lidar.rgl
        rects: list of dicts with keys: az_start, az_end, el_start, el_end
            azimuth: 0-360 degrees, elevation: -90 to 90 degrees.
            Rays inside any rectangle are masked.
    Example:
        set_mask_rectangles(bp, [
            {"az_start": 150, "az_end": 210, "el_start": -25, "el_end": 5},
        ])
    """
    parts = []
    for r in rects:
        for key in ("az_start", "az_end", "el_start", "el_end"):
            if key not in r:
                raise ValueError(f"Rectangle missing key: {key}")
        parts.append(f"{r['az_start']},{r['az_end']},{r['el_start']},{r['el_end']}")
    blueprint.set_attribute("ray_mask_rects", ";".join(parts))


def set_raw_mask(blueprint, channel_mask):
    """Set per-channel raw mask (advanced/fault injection).

    Args:
        blueprint: carla.ActorBlueprint for sensor.lidar.rgl
        channel_mask: list of int (1=enabled, 0=masked), one per channel.
            Applied to all horizontal steps of each channel.
    """
    blueprint.set_attribute("ray_mask_raw", _encode_int_array(channel_mask))


def set_noise(blueprint, angular_type=None, angular_mean=None, angular_stddev=None,
              distance_mean=None, distance_stddev_base=None, distance_stddev_rise=None,
              angular_axis=None):
    """Override individual noise parameters.

    Args:
        blueprint: carla.ActorBlueprint for sensor.lidar.rgl
        angular_type: "ray" or "hitpoint" (None = don't change)
        angular_mean: degrees (None = don't change)
        angular_stddev: degrees (None = don't change)
        distance_mean: meters (None = don't change)
        distance_stddev_base: meters (None = don't change)
        distance_stddev_rise: meters per meter (None = don't change)
        angular_axis: "X", "Y", or "Z" (None = don't change)
    """
    if angular_type is not None:
        if angular_type not in ("ray", "hitpoint"):
            raise ValueError(f"angular_type must be 'ray' or 'hitpoint', got '{angular_type}'")
        blueprint.set_attribute("noise_angular_type", angular_type)
    if angular_mean is not None:
        blueprint.set_attribute("noise_angular_mean", str(angular_mean))
    if angular_stddev is not None:
        if angular_stddev < 0:
            raise ValueError(f"angular_stddev must be >= 0, got {angular_stddev}")
        blueprint.set_attribute("noise_angular_stddev", str(angular_stddev))
    if distance_mean is not None:
        blueprint.set_attribute("noise_distance_mean", str(distance_mean))
    if distance_stddev_base is not None:
        if distance_stddev_base < 0:
            raise ValueError(f"distance_stddev_base must be >= 0, got {distance_stddev_base}")
        blueprint.set_attribute("noise_distance_stddev_base", str(distance_stddev_base))
    if distance_stddev_rise is not None:
        if distance_stddev_rise < 0:
            raise ValueError(f"distance_stddev_rise must be >= 0, got {distance_stddev_rise}")
        blueprint.set_attribute("noise_distance_stddev_rise", str(distance_stddev_rise))
    if angular_axis is not None:
        if angular_axis not in ("X", "Y", "Z"):
            raise ValueError(f"angular_axis must be 'X', 'Y', or 'Z', got '{angular_axis}'")
        blueprint.set_attribute("noise_angular_axis", angular_axis)


def disable_noise(blueprint):
    """Disable all noise (set all noise parameters to zero)."""
    blueprint.set_attribute("noise_angular_type", "")
    blueprint.set_attribute("noise_angular_mean", "0.0")
    blueprint.set_attribute("noise_angular_stddev", "0.0")
    blueprint.set_attribute("noise_distance_mean", "0.0")
    blueprint.set_attribute("noise_distance_stddev_base", "0.0")
    blueprint.set_attribute("noise_distance_stddev_rise", "0.0")
    blueprint.set_attribute("noise_angular_axis", "Y")


def set_beam_divergence(blueprint, horizontal, vertical):
    """Set beam divergence (degrees). Both must be >0 or both 0.

    Args:
        blueprint: carla.ActorBlueprint for sensor.lidar.rgl
        horizontal: horizontal beam divergence in degrees
        vertical: vertical beam divergence in degrees
    """
    if (horizontal > 0) != (vertical > 0):
        raise ValueError(
            f"Both horizontal and vertical must be >0 or both 0, got h={horizontal} v={vertical}")
    if horizontal < 0 or vertical < 0:
        raise ValueError(
            f"Beam divergence must be >= 0, got h={horizontal} v={vertical}")
    blueprint.set_attribute("beam_divergence_h", str(horizontal))
    blueprint.set_attribute("beam_divergence_v", str(vertical))


def disable_beam_divergence(blueprint):
    """Disable beam divergence (set both to 0)."""
    blueprint.set_attribute("beam_divergence_h", "0.0")
    blueprint.set_attribute("beam_divergence_v", "0.0")


def set_return_mode(blueprint, mode):
    """Set return mode.

    Single return modes (no beam divergence required):
        "first", "second", "last", "strongest"
    Dual return modes (require beam_divergence > 0):
        "first_last", "first_strongest", "first_second",
        "last_strongest", "strongest_second_strongest"

    Args:
        blueprint: carla.ActorBlueprint for sensor.lidar.rgl
        mode: return mode string
    """
    valid = {"first", "second", "last", "strongest",
             "first_last", "first_strongest", "first_second",
             "last_strongest", "strongest_second_strongest"}
    if mode not in valid:
        raise ValueError(f"Invalid return mode '{mode}'. Valid: {sorted(valid)}")
    blueprint.set_attribute("return_mode", mode)
